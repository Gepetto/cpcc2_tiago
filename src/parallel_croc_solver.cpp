#include <cpcc2_tiago/parallel_croc_solver.hpp>

// STL
#include <thread>

namespace cpcc2_tiago {

void ParallelCrocSolver::read_params() {
  Params params;
  n_joints_ = params.joints.size();
  joints_names_.reserve(n_joints_);
  joints_names_ = params.joints;
  OCP_solver_frequency_ = params.OCP_solver_frequency;
}

void ParallelCrocSolver::resize_vectors() {
  x_meas_.unlock()->resize(2 * n_joints_);
  us_.unlock()->resize(n_joints_);
  xs0_.unlock()->resize(2 * n_joints_);
  xs1_.unlock()->resize(2 * n_joints_);
  Ks_.unlock()->resize(2 * n_joints_, n_joints_);
}

Eigen::VectorXd ParallelCrocSolver::read_controller_x() { return x_meas_; }

Eigen::Vector3d ParallelCrocSolver::read_controller_target() { return target_; }

void ParallelCrocSolver::send_controller_result(const Eigen::VectorXd &us,
                                                const Eigen::VectorXd &xs0,
                                                const Eigen::VectorXd &xs1,
                                                const Eigen::MatrixXd &Ks) {
  auto [uus, uxs0, uxs1, uKs] = get_results();
  uus = us;
  uxs0 = xs0;
  uxs1 = xs1;
  uKs = Ks;
}

ParallelCrocSolver::~ParallelCrocSolver() {
  running_ = false;
  if (thread_.joinable()) thread_.join();
}

void ParallelCrocSolver::init_model(const std::string &urdf_xml) {
  read_params();
  resize_vectors();

  //  don't start solving until the first crocoddyl controller update is
  //  done and the first target is received

  model_ = model_builder::build_model(urdf_xml, joints_names_);
  data_ = pin::Data(model_);
  std::cout << "Model built" << std::endl;

  // create the OCP object
  OCP_tiago_ = tiago_OCP::OCP(model_, data_);

  // set the hand frame id
  lh_id_ = model_.getFrameId("hand_tool_joint");
  OCP_tiago_.setLhId(lh_id_);

  Params params;

  // set the OCP parameters
  OCP_horizon_length_ = params.OCP_horizon_length;
  OCP_time_step_ = params.OCP_time_step;
  OCP_solver_frequency_ = params.OCP_solver_frequency;
  OCP_solver_iterations_ = params.OCP_solver_iterations;

  OCP_tiago_.setHorizonLength(OCP_horizon_length_);
  OCP_tiago_.setTimeStep(OCP_time_step_);
  OCP_tiago_.setSolverIterations(OCP_solver_iterations_);

  std::cout << "OCP settings set." << std::endl;
}

void ParallelCrocSolver::start_thread() {
  x_meas_ = read_controller_x();

  OCP_tiago_.setX0(x_meas_);

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
  const Eigen::Vector3d target = read_controller_target();

  std::cout << "First target: " << target.transpose() << std::endl;

  OCP_tiago_.buildSolver(x_meas_, target);

  std::cout << "Solver started at: " << OCP_solver_frequency_ << " Hz"
            << std::endl;

  std::cout << "Solver iterations: " << OCP_solver_iterations_ << std::endl;

  OCP_tiago_.solveFirst(x_meas_);

  OCP_tiago_.printCosts();

  auto us = OCP_tiago_.get_us()[0];
  auto xs0 = OCP_tiago_.get_xs()[0];
  auto xs1 = OCP_tiago_.get_xs()[1];
  auto Ks = OCP_tiago_.get_gains();

  send_controller_result(us, xs0, xs1, Ks);

  std::cout << "First solve done" << std::endl;

  last_current_time_ = current_time_;

  thread_ = std::thread{[this]() {
    while (this->running_) {
      this->update();
      this->wait();
    }
  }};
}

void ParallelCrocSolver::update() {
  x_meas_ = read_controller_x();
  const Eigen::Vector3d target = read_controller_target();

  if (target != OCP_tiago_.get_target()) {
    OCP_tiago_.changeTarget(target);
    std::cout << "\x1b[A"
              << "New target: " << target.transpose() << "            \n";
  }

  OCP_tiago_.solve(x_meas_);

  auto us = OCP_tiago_.get_us()[0];
  auto xs0 = OCP_tiago_.get_xs()[0];
  auto xs1 = OCP_tiago_.get_xs()[1];
  auto Ks = OCP_tiago_.get_gains();

  send_controller_result(us, xs0, xs1, Ks);
}

void ParallelCrocSolver::wait() {
  /* to synchronize the time with the controller we use the ROS time
   * for the solver, for an accurate time reading, the controllers have to
   * run at a higher frequency than the solver
   */
  using namespace std::chrono_literals;

  std::cout << "Solv freq: " << std::setprecision(2)
            << solver_freq_vector_.mean() << " Hz, in "
            << solving_time_vector_.mean() * 1e-6 << " ms     " << std::endl
            << "\x1b[A";

  const double target_time = 1e9 / OCP_solver_frequency_;
  while (current_time_ - last_current_time_ < target_time && running_)
    std::this_thread::sleep_for(100'000ns);

  const double current_time = current_time_;
  const double delta_time = current_time - last_current_time_;
  last_current_time_ = current_time;

  solving_time_vector_.append(delta_time);
  solver_freq_vector_.append(1e9 / delta_time);
}

}  // namespace cpcc2_tiago
