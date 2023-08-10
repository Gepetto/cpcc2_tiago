#include "cpcc2_tiago/tiago_OCP.hpp"

namespace tiago_OCP {

OCP::OCP() {}

OCP::OCP(const Model model, const Data data) {
  model_ = model;
  data_ = data;
  initOCPParms();
}

void OCP::initOCPParms() {
  state_ =
      boost::make_shared<StateMultibody>(boost::make_shared<Model>(model_));

  state_nv_ = state_->get_nv();
  state_nq_ = state_->get_nq();
  actuation_ = boost::make_shared<ActuationModelFull>(state_);
  actuation_nu_ = actuation_->get_nu();
  contacts_ = boost::make_shared<ContactModelMultiple>(state_, actuation_nu_);
}

boost::shared_ptr<ActionModelAbstract> OCP::buildRunningModel() {
  boost::shared_ptr<CostModelSum> costs =
      boost::make_shared<CostModelSum>(state_, actuation_->get_nu());

  defineHandTask(costs, w_hand_, costs_weights_["lh_goal_weight"]);
  defineXReg(costs, w_x_, costs_weights_["xReg_weight"]);
  defineUReg(costs, costs_weights_["uReg_weight"]);
  defineXbounds(costs, costs_weights_["xBounds_weight"]);

  boost::shared_ptr<DifferentialActionModelContactFwdDynamics> diff_act_model =
      boost::make_shared<DifferentialActionModelContactFwdDynamics>(
          state_, actuation_, contacts_, costs);

  boost::shared_ptr<IntegratedActionModelEuler> running_model =
      boost::make_shared<IntegratedActionModelEuler>(diff_act_model,
                                                     time_step_);

  return running_model;
}

boost::shared_ptr<ActionModelAbstract> OCP::buildTerminalModel() {
  boost::shared_ptr<CostModelSum> costs =
      boost::make_shared<CostModelSum>(state_, actuation_->get_nu());

  defineHandTask(costs, w_hand_, costs_weights_["lh_goal_weight"]);
  defineXReg(costs, w_x_, costs_weights_["xReg_weight"]);
  defineUReg(costs, costs_weights_["uReg_weight"]);
  defineXbounds(costs, costs_weights_["xBounds_weight"]);

  boost::shared_ptr<DifferentialActionModelContactFwdDynamics> diff_act_model =
      boost::make_shared<DifferentialActionModelContactFwdDynamics>(
          state_, actuation_, contacts_, costs);

  boost::shared_ptr<IntegratedActionModelEuler> term_model =
      boost::make_shared<IntegratedActionModelEuler>(diff_act_model, 0.0);

  return term_model;
}

Eigen::VectorXd OCP::setBalancingTorques(Eigen::VectorXd x0) {
  Eigen::VectorXd balancing_torques;
  Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(state_nq_ + state_nv_);

  x_ref.head(state_nq_) = x0.head(state_nq_);

  balancing_torques.resize(actuation_nu_);
  iam(0)->quasiStatic(ada(0), balancing_torques, x0);

  return balancing_torques;
}

void OCP::buildSolver(Eigen::VectorXd x0) {
  // Creating all the running models
  std::vector<boost::shared_ptr<ActionModelAbstract>> running_models =
      std::vector<boost::shared_ptr<ActionModelAbstract>>(horizon_length_);

  for (size_t node_id = 0; node_id < horizon_length_; node_id++) {
    running_models[node_id] = buildRunningModel();
  }

  boost::shared_ptr<ActionModelAbstract> term_model = buildTerminalModel();

  problem_ =
      boost::make_shared<ShootingProblem>(x0_, running_models, term_model);

  solver_ = boost::make_shared<SolverFDDP>(problem_);

  balancing_torques_.resize(actuation_nu_);
  balancing_torques_ = setBalancingTorques(x0);
  std::cout << "balancing_torques: " << balancing_torques_.transpose()
            << std::endl;
}

void OCP::setTarget(Vector3d target) {
  target_ = target;
  lh_Mref_ = SE3(Matrix3d::Identity(), target_);

  for (size_t node_id = 0; node_id < horizon_length_ + 1; node_id++) {
    boost::static_pointer_cast<ResidualModelFramePlacement>(
        costs(node_id)->get_costs().at("lh_goal")->cost->get_residual())
        ->set_reference(lh_Mref_);
  }
};

void OCP::changeTarget(Vector3d target) {
  target_ = target;
  lh_Mref_ = SE3(Matrix3d::Identity(), target_);
  // only change the cost of the last running node

  boost::static_pointer_cast<ResidualModelFramePlacement>(
      costs(horizon_length_)->get_costs().at("lh_goal")->cost->get_residual())
      ->set_reference(lh_Mref_);
}

void OCP::solveFirst(VectorXd measured_x) {
  // horizon settings
  std::vector<VectorXd> xs_init;
  std::vector<VectorXd> us_init;

  for (std::size_t i = 0; i < horizon_length_; i++) {
    xs_init.push_back(measured_x);

    us_init.push_back(balancing_torques_);
  }
  xs_init.push_back(measured_x);

  solver_->solve(xs_init, us_init, 1000, false);
}

void OCP::recede() {
  solver_->get_problem()->circularAppend(
      solver_->get_problem()->get_runningModels()[0],
      solver_->get_problem()->get_runningDatas()[0]);
}

void OCP::updateRunModReference() {
  boost::static_pointer_cast<ResidualModelFramePlacement>(
      costs(horizon_length_ - 1)
          ->get_costs()
          .at("lh_goal")
          ->cost->get_residual())
      ->set_reference(lh_Mref_);
}

void OCP::solve(VectorXd measured_x) {
  recede();
  updateRunModReference();

  warm_xs_ = solver_->get_xs();
  warm_xs_.erase(warm_xs_.begin());
  warm_xs_[0] = measured_x;
  warm_xs_.push_back(warm_xs_[warm_xs_.size() - 1]);

  warm_us_ = solver_->get_us();
  warm_us_.erase(warm_us_.begin());
  warm_us_.push_back(warm_us_[warm_us_.size() - 1]);

  solver_->get_problem()->set_x0(measured_x);
  solver_->allocateData();
  solver_->solve(solver_->get_xs(), solver_->get_us(), solver_iterations_);
}

boost::shared_ptr<crocoddyl::ActionModelAbstract> OCP::ama(
    const unsigned long time) {
  if (time == horizon_length_) {
    return solver_->get_problem()->get_terminalModel();
  } else {
    return solver_->get_problem()->get_runningModels()[time];
  }
}

boost::shared_ptr<crocoddyl::IntegratedActionModelEuler> OCP::iam(
    const unsigned long time) {
  return boost::static_pointer_cast<crocoddyl::IntegratedActionModelEuler>(
      ama(time));
}

boost::shared_ptr<crocoddyl::DifferentialActionModelContactFwdDynamics>
OCP::dam(const unsigned long time) {
  return boost::static_pointer_cast<
      crocoddyl::DifferentialActionModelContactFwdDynamics>(
      iam(time)->get_differential());
}

boost::shared_ptr<crocoddyl::CostModelSum> OCP::costs(
    const unsigned long time) {
  return dam(time)->get_costs();
}

boost::shared_ptr<crocoddyl::ActionDataAbstract> OCP::ada(
    const unsigned long time) {
  return solver_->get_problem()->get_runningDatas()[time];
}

};  // namespace tiago_OCP
