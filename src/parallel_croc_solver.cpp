#include "cpcc2_tiago/parallel_croc_solver.hpp"

void read_params() {
  n_joints_ = params_.joints.size();
  joints_names_.reserve(n_joints_);
  joints_names_ = params_.joints;
}

void resize_vectors() {
  x_meas_.resize(2 * n_joints_);
  xs_.resize(2 * n_joints_);
  us_.resize(n_joints_);
  Ks_.resize(n_joints_, 2 * n_joints_);
}

void init_shared_memory() {
  crocoddyl_shm_ = boost::interprocess::managed_shared_memory(
      boost::interprocess::open_only,
      "crcoddyl_shm"); // segment name

  // Find the vector using the c-string name
  x_meas_shm_ = crocoddyl_shm_.find<shared_vector>("x_meas_shm").first;
  us_shm_ = crocoddyl_shm_.find<shared_vector>("us_shm").first;
  xs_shm_ = crocoddyl_shm_.find<shared_vector>("xs_shm").first;
  Ks_shm_ = crocoddyl_shm_.find<shared_vector>("Ks_shm").first;
  target_smh_ = crocoddyl_shm_.find<shared_vector>("target_shm").first;
  solver_started_shm_ = crocoddyl_shm_.find<bool>("solver_started_shm").first;
  is_first_update_done_shm_ =
      crocoddyl_shm_.find<bool>("is_first_update_done_shm").first;
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

void send_controller_result(Eigen::VectorXd us, Eigen::VectorXd xs,
                            Eigen::MatrixXd Ks) {
  mutex_.lock();
  us_shm_->assign(us.data(), us.data() + us.size());
  xs_shm_->assign(xs.data(), xs.data() + us.size());
  Ks_shm_->assign(Ks.data(), Ks.data() + us.size());
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

  OCP_horizon_length_ = params_.horizon_length;
  OCP_time_step_ = params_.time_step;
  OCP_tiago_.setHorizonLength(OCP_horizon_length_);
  OCP_tiago_.setTimeStep(OCP_time_step_);

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
  OCP_tiago_.buildDiffActModel();
  OCP_tiago_.buildSolver();

  OCP_tiago_.printCosts();

  std::cout << "Solver started" << std::endl;

  while (true) {

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
      std::cout << "First target:" << target_.transpose() << std::endl;
      OCP_tiago_.solveFirst(x_meas_);
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

    OCP_tiago_.solve(x_meas_);

    us_ = OCP_tiago_.get_us();
    xs_ = OCP_tiago_.get_xs();
    Ks_ = OCP_tiago_.get_gains();

    send_controller_result(us_, xs_, Ks_);
  }
}