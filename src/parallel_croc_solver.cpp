#include <cpcc2_tiago/parallel_croc_solver.hpp>

namespace parallel_croc_solver {

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
  boost::interprocess::shared_memory_object::remove("crocoddyl_shm");

  crocoddyl_shm_ = boost::interprocess::managed_shared_memory(
      boost::interprocess::create_only, "crocoddyl_shm", 65536);

  // Initialize shared memory STL-compatible allocator
  const shm_allocator alloc_inst(crocoddyl_shm_.get_segment_manager());

  // constuct all the shared memory segments
  x_meas_shm_ =
      crocoddyl_shm_.construct<shared_vector>("x_meas_shm")  // object name
      (alloc_inst);                                          //
  us_shm_ = crocoddyl_shm_.construct<shared_vector>("us_shm")(alloc_inst);
  xs0_shm_ = crocoddyl_shm_.construct<shared_vector>("xs0_shm")(alloc_inst);
  xs1_shm_ = crocoddyl_shm_.construct<shared_vector>("xs1_shm")(alloc_inst);
  Ks_shm_ = crocoddyl_shm_.construct<shared_vector>("Ks_shm")(alloc_inst);
  target_shm_ =
      crocoddyl_shm_.construct<shared_vector>("target_shm")(alloc_inst);

  solver_started_shm_ =
      crocoddyl_shm_.construct<bool>("solver_started_shm")(false);
  is_first_update_done_shm_ =
      crocoddyl_shm_.construct<bool>("is_first_update_done_shm")(false);

  start_sending_cmd_shm_ =
      crocoddyl_shm_.construct<bool>("start_sending_cmd_shm")(false);

  current_t_shm_ = crocoddyl_shm_.construct<double>("current_t_shm")(0.0);

  // resize all the shared memory segments and fill them with zeros
  // to avoid reading uninitialized memory
  x_meas_shm_->resize(x_meas_.size());
  std::fill(x_meas_shm_->begin(), x_meas_shm_->end(), 0.0);

  us_shm_->resize(us_.size());
  std::fill(us_shm_->begin(), us_shm_->end(), 0.0);

  xs0_shm_->resize(xs0_.size());
  std::fill(xs0_shm_->begin(), xs0_shm_->end(), 0.0);

  xs1_shm_->resize(xs1_.size());
  std::fill(xs1_shm_->begin(), xs1_shm_->end(), 0.0);

  Ks_shm_->resize(Ks_.size());
  std::fill(Ks_shm_->begin(), Ks_shm_->end(), 0.0);

  target_shm_->resize(3);
}

double read_current_t() {
  mutex_.lock();
  double current_t = *current_t_shm_;
  mutex_.unlock();
  return current_t;
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
      Eigen::Map<Eigen::Vector3d>(target_shm_->data(), target_shm_->size());
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

}  // namespace parallel_croc_solver

int main() {
  using namespace parallel_croc_solver;

  read_params();
  resize_vectors();

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

  //  don't start solving until the first crocoddyl controller update is
  //  done and the first target is received

  std::cout << "Waiting for first update" << std::endl;
  while (is_first_update_done_ == false) {
    mutex_.lock();
    is_first_update_done_ = *is_first_update_done_shm_;
    mutex_.unlock();
  }

  std::cout << "First update done" << std::endl;

  x_meas_ = read_controller_x();

  // Build the model from the urdf
  model_ = model_builder::build_model(params_.urdf_path, joints_names_);

  data_ = Data(model_);

  std::cout << "Model built" << std::endl;

  // create the OCP object
  OCP_tiago_ = tiago_OCP::OCP(model_, data_);

  OCP_tiago_.setX0(x_meas_);

  // set the hand frame id
  lh_id_ = model_.getFrameId("hand_tool_joint");
  OCP_tiago_.setLhId(lh_id_);

  // set the OCP parameters
  OCP_horizon_length_ = params_.OCP_horizon_length;
  OCP_time_step_ = params_.OCP_time_step;
  OCP_solver_frequency_ = params_.OCP_solver_frequency;
  OCP_solver_iterations_ = params_.OCP_solver_iterations;

  OCP_tiago_.setHorizonLength(OCP_horizon_length_);
  OCP_tiago_.setTimeStep(OCP_time_step_);
  OCP_tiago_.setSolverIterations(OCP_solver_iterations_);

  std::cout << "OCP settings set." << std::endl;

  // set the OCP costs weights and activation weights
  std::map<std::string, double> costs_weights{{"lh_goal_weight", 1e2},
                                              {"xReg_weight", 1e-3},
                                              {"uReg_weight", 1e-4},
                                              {"xBounds_weight", 1}};

  Eigen::VectorXd w_hand(6);

  w_hand << Eigen::VectorXd::Constant(3, 5),
      Eigen::VectorXd::Constant(3, 0.0001);

  Eigen::VectorXd w_x(2 * model_.nv);

  w_x << Eigen::VectorXd::Zero(3), Eigen::VectorXd::Constant(3, 10.0),
      Eigen::VectorXd::Constant(model_.nv - 6, 0.01),
      Eigen::VectorXd::Constant(model_.nv, 10.0);

  OCP_tiago_.setCostsWeights(costs_weights);
  OCP_tiago_.setCostsActivationWeights(w_hand, w_x);

  // read the first target, it should be the current position
  target_ = read_controller_target();

  std::cout << "First target: " << target_.transpose() << std::endl;

  OCP_tiago_.buildSolver(x_meas_, target_);

  std::cout << "Solver started at: " << OCP_solver_frequency_ << " Hz"
            << std::endl;

  std::cout << "Solver iterations: " << OCP_solver_iterations_ << std::endl;

  OCP_tiago_.solveFirst(x_meas_);
  mutex_.lock();
  *solver_started_shm_ = true;
  mutex_.unlock();

  OCP_tiago_.printCosts();

  us_ = OCP_tiago_.get_us()[0];
  xs0_ = OCP_tiago_.get_xs()[0];
  xs1_ = OCP_tiago_.get_xs()[1];
  Ks_ = OCP_tiago_.get_gains();

  send_controller_result(us_, xs0_, xs1_, Ks_);

  std::cout << "First solve done" << std::endl;

  // after the first solve, start sending the commands to the robot
  mutex_.lock();
  *start_sending_cmd_shm_ = true;
  mutex_.unlock();

  // start the solver loop
  while (true) {
    /* to synchronize the time with the controller we use the ROS time
     * for the solver, for an accurate time reading, the controllers have to
     * run at a higher frequency than the solver
     */
    current_t_ = read_current_t();

    diff_ = current_t_ - last_solving_time_;
    if (diff_ * 1e-9 < 1 / OCP_solver_frequency_) {
      continue;
    }

    x_meas_ = read_controller_x();
    target_ = read_controller_target();

    if (target_ != OCP_tiago_.get_target()) {
      OCP_tiago_.changeTarget(target_);
      std::cout << "\x1b[A";
      std::cout << "New target: " << target_.transpose() << "            "
                << std::endl;
    }

    start_solving_time_ = read_current_t();

    OCP_tiago_.solve(x_meas_);

    us_ = OCP_tiago_.get_us()[0];
    xs0_ = OCP_tiago_.get_xs()[0];
    xs1_ = OCP_tiago_.get_xs()[1];
    Ks_ = OCP_tiago_.get_gains();

    send_controller_result(us_, xs0_, xs1_, Ks_);

    current_t_ = read_current_t();
    solving_time_ = current_t_ - start_solving_time_;
    solver_freq_ = 1 / (diff_ * 1e-9);

    solving_time_vector_.circular_append(solving_time_);
    solver_freq_vector_.circular_append(solver_freq_);

    std::cout << "Solv freq: " << std::setprecision(2)
              << solver_freq_vector_.vector.mean() << " Hz, in "
              << solving_time_vector_.vector.mean() * 10e-6 << " ms     "
              << std::endl;
    std::cout << "\x1b[A";

    last_solving_time_ = current_t_;
  }
}
