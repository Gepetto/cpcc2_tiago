#include <cpcc2_tiago/tiago_OCP.hpp>

namespace tiago_OCP {

OCP::OCP() {}

OCP::OCP(const pin::Model model, const pin::Data data) {
  model_ = model;
  data_ = data;
  initOCPParms();
}

void OCP::initOCPParms() {
  state_ = boost::make_shared<croc::StateMultibody>(
      boost::make_shared<pin::Model>(model_));

  state_nv_ = state_->get_nv();
  state_nq_ = state_->get_nq();
  actuation_ = boost::make_shared<croc::ActuationModelFull>(state_);
  actuation_nu_ = actuation_->get_nu();
  contacts_ =
      boost::make_shared<croc::ContactModelMultiple>(state_, actuation_nu_);
}

boost::shared_ptr<croc::ActionModelAbstract> OCP::buildRunningModel() {
  boost::shared_ptr<croc::CostModelSum> costs =
      boost::make_shared<croc::CostModelSum>(state_, actuation_->get_nu());

  // define the task for the running nodes
  defineHandTask(costs, w_hand_, costs_weights_["lh_goal_weight"]);
  defineXReg(costs, w_x_, costs_weights_["xReg_weight"]);
  defineUReg(costs, costs_weights_["uReg_weight"]);
  defineXbounds(costs, costs_weights_["xBounds_weight"]);

  boost::shared_ptr<croc::DifferentialActionModelContactFwdDynamics>
      diff_act_model =
          boost::make_shared<croc::DifferentialActionModelContactFwdDynamics>(
              state_, actuation_, contacts_, costs);

  boost::shared_ptr<croc::IntegratedActionModelEuler> running_model =
      boost::make_shared<croc::IntegratedActionModelEuler>(diff_act_model,
                                                           time_step_);

  return running_model;
}

boost::shared_ptr<croc::ActionModelAbstract> OCP::buildTerminalModel() {
  boost::shared_ptr<croc::CostModelSum> costs =
      boost::make_shared<croc::CostModelSum>(state_, actuation_->get_nu());

  // define the task for the last node
  defineHandTask(costs, w_hand_, costs_weights_["lh_goal_weight"]);
  defineXReg(costs, w_x_, costs_weights_["xReg_weight"]);
  defineUReg(costs, costs_weights_["uReg_weight"]);
  defineXbounds(costs, costs_weights_["xBounds_weight"]);

  boost::shared_ptr<croc::DifferentialActionModelContactFwdDynamics>
      diff_act_model =
          boost::make_shared<croc::DifferentialActionModelContactFwdDynamics>(
              state_, actuation_, contacts_, costs);

  boost::shared_ptr<croc::IntegratedActionModelEuler> term_model =
      boost::make_shared<croc::IntegratedActionModelEuler>(diff_act_model, 0.0);

  return term_model;
}

Eigen::VectorXd OCP::computeBalancingTorques(Eigen::VectorXd x0) {
  Eigen::VectorXd balancing_torques;
  Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(state_nq_ + state_nv_);

  x_ref.head(state_nq_) = x0.head(state_nq_);

  balancing_torques.resize(actuation_nu_);
  iam(0)->quasiStatic(ada(0), balancing_torques, x0);

  return balancing_torques;
}

void OCP::buildSolver(Eigen::VectorXd x0, Eigen::Vector3d target) {
  // Creating all the running models
  std::vector<boost::shared_ptr<croc::ActionModelAbstract>> running_models =
      std::vector<boost::shared_ptr<croc::ActionModelAbstract>>(
          horizon_length_);

  for (size_t node_id = 0; node_id < horizon_length_; node_id++) {
    running_models[node_id] = buildRunningModel();
  }

  boost::shared_ptr<croc::ActionModelAbstract> term_model =
      buildTerminalModel();

  problem_ = boost::make_shared<croc::ShootingProblem>(x0_, running_models,
                                                       term_model);

  solver_ = boost::make_shared<croc::SolverFDDP>(problem_);

  balancing_torques_.resize(actuation_nu_);
  balancing_torques_ = computeBalancingTorques(x0);
  setTarget(target);

  for (size_t node_id = 0; node_id < horizon_length_ + 1; node_id++) {
    costs(node_id)->get_costs().at("lh_goal")->active = false;
  }

  solveFirst(x0);

  costs(horizon_length_)->get_costs().at("lh_goal")->active = true;
}

void OCP::setTarget(Eigen::Vector3d target) {
  target_ = target;
  lh_Mref_ = pin::SE3(Eigen::Matrix3d::Identity(), target_);
  // change the target for all node
  for (size_t node_id = 0; node_id < horizon_length_ + 1; node_id++) {
    boost::static_pointer_cast<croc::ResidualModelFramePlacement>(
        costs(node_id)->get_costs().at("lh_goal")->cost->get_residual())
        ->set_reference(lh_Mref_);
  }
};

void OCP::changeTarget(Eigen::Vector3d target) {
  target_ = target;
  lh_Mref_ = pin::SE3(Eigen::Matrix3d::Identity(), target_);
  // only change the cost of the terminal node

  boost::static_pointer_cast<croc::ResidualModelFramePlacement>(
      costs(horizon_length_)->get_costs().at("lh_goal")->cost->get_residual())
      ->set_reference(lh_Mref_);
}

void OCP::solveFirst(Eigen::VectorXd measured_x) {
  std::vector<Eigen::VectorXd> xs_init;
  std::vector<Eigen::VectorXd> us_init;

  for (std::size_t i = 0; i < horizon_length_; i++) {
    xs_init.push_back(measured_x);
    // Gravity compensation torques
    us_init.push_back(balancing_torques_);
  }
  xs_init.push_back(measured_x);

  solver_->solve(xs_init, us_init, 500, false);
}

void OCP::recede() {
  // receide the horizon and put the first element at the end
  // its cost target will be replace by the target in the
  // updateRunModReference function
  solver_->get_problem()->circularAppend(
      solver_->get_problem()->get_runningModels()[0],
      solver_->get_problem()->get_runningDatas()[0]);
}

void OCP::updateRunModHandReference() {
  costs(horizon_length_ - 1)->get_costs().at("lh_goal")->active = true;

  boost::static_pointer_cast<croc::ResidualModelFramePlacement>(
      costs(horizon_length_ - 1)
          ->get_costs()
          .at("lh_goal")
          ->cost->get_residual())
      ->set_reference(lh_Mref_);
}

void OCP::updateRunModXRegReference(
    std::vector<Eigen::VectorXd> const& x_Xreg) {
  for (size_t i = 0; i < horizon_length_ - 1; i++) {
    boost::static_pointer_cast<croc::ResidualModelState>(
        costs(i)->get_costs().at("xReg")->cost->get_residual())
        ->set_reference(x_Xreg[i + 1]);
  }
  boost::static_pointer_cast<croc::ResidualModelState>(
      costs(horizon_length_ - 1)->get_costs().at("xReg")->cost->get_residual())
      ->set_reference(x_Xreg.back());
}

void OCP::solve(Eigen::VectorXd measured_x) {
  // recede the horizon and update the reference of the last node
  recede();
  updateRunModHandReference();
  // Does it really help ?
  // updateRunModXRegReference(solver_->get_xs());

  warm_xs_ = solver_->get_xs();
  warm_xs_.erase(warm_xs_.begin());
  warm_xs_[0] = measured_x;
  warm_xs_.push_back(warm_xs_.back());

  warm_us_ = solver_->get_us();
  warm_us_.erase(warm_us_.begin());
  warm_us_.push_back(warm_us_.back());

  solver_->get_problem()->set_x0(measured_x);
  solver_->allocateData();
  //@TODO : why when putting the warm start it does not work ?
  solver_->solve(solver_->get_xs(), solver_->get_us(), solver_iterations_);
}

boost::shared_ptr<crocoddyl::ActionModelAbstract> OCP::ama(
    const unsigned long node_id) {
  if (node_id == horizon_length_) {
    return solver_->get_problem()->get_terminalModel();
  } else {
    return solver_->get_problem()->get_runningModels()[node_id];
  }
}

boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> OCP::iam(
    const unsigned long node_id) {
  return boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(
      ama(node_id));
}

boost::shared_ptr<crocoddyl::DifferentialActionModelContactFwdDynamics>
OCP::dam(const unsigned long node_id) {
  return boost::static_pointer_cast<
      crocoddyl::DifferentialActionModelContactFwdDynamics>(
      iam(node_id)->get_differential());
}

boost::shared_ptr<crocoddyl::CostModelSum> OCP::costs(
    const unsigned long node_id) {
  return dam(node_id)->get_costs();
}

boost::shared_ptr<crocoddyl::ActionDataAbstract> OCP::ada(
    const unsigned long node_id) {
  return solver_->get_problem()->get_runningDatas()[node_id];
}

}  // namespace tiago_OCP
