#include "cpcc2_tiago/parallel_croc_solver.hpp"

void read_params() {
  n_joints_ = params_.joints.size();
  joints_names_.reserve(n_joints_);
  joints_names_ = params_.joints;
  OCP_solver_frequency_ = params_.OCP_solver_frequency;
}

void resize_vectors() {
  x_meas_.resize(2 * n_joints_);
  xs0_.resize(2 * n_joints_);
  xs1_.resize(2 * n_joints_);
  us_.resize(n_joints_);
  Ks_.resize(2 * n_joints_, n_joints_);
}

void init_shared_memory() {
  crocoddyl_shm_ = boost::interprocess::managed_shared_memory(
      boost::interprocess::open_only,
      "crocoddyl_shm");  // segment name

  // Find the vector using the c-string name
  x_meas_shm_ = crocoddyl_shm_.find<shared_vector>("x_meas_shm").first;
  us_shm_ = crocoddyl_shm_.find<shared_vector>("us_shm").first;
  xs0_shm_ = crocoddyl_shm_.find<shared_vector>("xs0_shm").first;
  xs1_shm_ = crocoddyl_shm_.find<shared_vector>("xs1_shm").first;
  Ks_shm_ = crocoddyl_shm_.find<shared_vector>("Ks_shm").first;
  target_smh_ = crocoddyl_shm_.find<shared_vector>("target_shm").first;
  solver_started_shm_ = crocoddyl_shm_.find<bool>("solver_started_shm").first;
  is_first_update_done_shm_ =
      crocoddyl_shm_.find<bool>("is_first_update_done_shm").first;
  start_sending_cmd_shm_ =
      crocoddyl_shm_.find<bool>("start_sending_cmd_shm").first;
}

Eigen::VectorXd read_controller_x() {
  mutex_.lock();
  Eigen::VectorXd x =
      Eigen::Map<Eigen::VectorXd>(x_meas_shm_->data(), x_meas_shm_->size());
  mutex_.unlock();
  return x;
}

Eigen::Vector3d read_controller_target() {
  mutex_.lock();
  Eigen::Vector3d target =
      Eigen::Map<Eigen::Vector3d>(target_smh_->data(), target_smh_->size());
  mutex_.unlock();
  return target;
}

void send_controller_result(Eigen::VectorXd us, Eigen::VectorXd xs0,
                            Eigen::VectorXd xs1, Eigen::MatrixXd Ks) {
  mutex_.lock();
  us_shm_->assign(us.data(), us.data() + us.size());
  xs0_shm_->assign(xs0.data(), xs0.data() + xs0.size());
  xs1_shm_->assign(xs1.data(), xs1.data() + xs1.size());
  for (int i = 0; i < Ks_.rows(); i++) {  // to have the right order
    for (int j = 0; j < Ks_.cols(); j++) {
      Ks_shm_->at(i * Ks_.cols() + j) = Ks(i, j);
    }
  }
  mutex_.unlock();
}

int main() {
  read_params();

  while (true) {
    if (mutex_.try_lock()) {
      break;
    }

    if (!mutex_.timed_lock(boost::get_system_time() +
                           boost::posix_time::milliseconds(10))) {
      mutex_.unlock();
    }
  }

  mutex_.unlock();

  init_shared_memory();

  // Build the model from the urdf
  model_ = model_builder::build_model(joints_names_);

  data_ = Data(model_);

  // create the OCP object
  OCP_tiago_ = tiago_OCP::OCP(model_, data_);

  VectorXd x0 = VectorXd::Zero(model_.nq + model_.nv);
  OCP_tiago_.setX0(x0);

  lh_id_ = model_.getFrameId("hand_tool_joint");
  OCP_tiago_.setLhId(lh_id_);

  OCP_horizon_length_ = params_.OCP_horizon_length;
  OCP_time_step_ = params_.OCP_time_step;
  OCP_solver_iterations_ = params_.OCP_solver_iterations;
  OCP_tiago_.setHorizonLength(OCP_horizon_length_);
  OCP_tiago_.setTimeStep(OCP_time_step_);
  OCP_tiago_.setSolverIterations(OCP_solver_iterations_);

  OCP_tiago_.setTarget(Eigen::Vector3d::Zero());

  std::map<std::string, double> costs_weights{{"lh_goal_weight", 1e2},
                                              {"xReg_weight", 1e-3},
                                              {"uReg_weight", 1e-4},
                                              {"xBounds_weight", 1}};

  VectorXd w_hand(6);

  w_hand << VectorXd::Constant(3, 1), VectorXd::Constant(3, 0.0001);

  VectorXd w_x(2 * model_.nv);

  w_x << VectorXd::Zero(3), VectorXd::Constant(3, 10.0),
      VectorXd::Constant(model_.nv - 6, 0.01),
      VectorXd::Constant(model_.nv, 10.0);

  OCP_tiago_.buildCostsModel(costs_weights, w_hand, w_x);
  OCP_tiago_.buildSolver();

  OCP_tiago_.printCosts();

  std::cout << "Solver started at: " << OCP_solver_frequency_ << " Hz"
            << std::endl;

  std::cout << "Solver iterations: " << OCP_solver_iterations_ << std::endl;

  std::cout << std::endl;

  sleep(1);

  while (true) {
    //  don't start solving until the first crocoddyl controller update is
    //  done and the first target is received

    if (is_first_update_done_ == false) {
      mutex_.lock();
      is_first_update_done_ = *is_first_update_done_shm_;
      mutex_.unlock();
      continue;
    }

    x_meas_ = read_controller_x();
    target_ = read_controller_target();

    if (is_first_update_done_ == true && solved_first_ == false) {
      OCP_tiago_.changeTarget(target_);
      std::cout << "First target: " << target_.transpose() << std::endl;
      // OCP_tiago_.solveFirst(x_meas_);
      solved_first_ = true;
      mutex_.lock();
      *solver_started_shm_ = true;
      mutex_.unlock();
      std::cout << "First solve done" << std::endl;
      continue;
    }

    if (target_ != OCP_tiago_.get_target()) {
      OCP_tiago_.changeTarget(target_);
    }

    diff_ = std::chrono::high_resolution_clock::now() - last_solving_time_;
    if (diff_.count() * 1e-9 < 1 / OCP_solver_frequency_) {
      continue;
    }

    start_solving_time_ = std::chrono::high_resolution_clock::now();

    OCP_tiago_.solve(x_meas_);

    us_ = OCP_tiago_.get_us()[0];
    xs0_ = OCP_tiago_.get_xs()[0];
    xs1_ = OCP_tiago_.get_xs()[1];
    Ks_ = OCP_tiago_.get_gains();

    send_controller_result(us_, xs0_, xs1_, Ks_);

    if (start_sending_cmd_ == false) {
      mutex_.lock();
      *start_sending_cmd_shm_ = true;
      mutex_.unlock();
      start_sending_cmd_ = true;
    }

    current_t_ = std::chrono::high_resolution_clock::now();

    if (current_t_.time_since_epoch().count() % 10 == 0) {
      std::cout << "Solver frequency: " << std::fixed << std::setprecision(2)
                << 1 / (diff_.count() * 1e-9) << " Hz, solving time: "
                << std::chrono::duration_cast<std::chrono::microseconds>(
                       current_t_ - start_solving_time_)
                           .count() /
                       1000.0

                << " ms" << std::endl;

      std::cout << "\x1b[A";
    }

    last_solving_time_ = current_t_;
  }
}