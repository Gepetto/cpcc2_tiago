#include "cpcc2_tiago/tiago_OCP_maker.hpp"

namespace tiago_OCP {

OCP::OCP() {}

OCP::OCP(const Model &model) {
  model_ = model;
  initOCPParms();
}

void OCP::setTarget(Vector3d &target) {
  target_ = target;
  lh_Mref_ = SE3(Matrix3d::Identity(), target_);
}

void OCP::changeTarget(Vector3d target) {
  target_ = target;
  lh_Mref_ = SE3(Matrix3d::Identity(), target_);
  boost::static_pointer_cast<ResidualModelFramePlacement>(
      costs_->get_costs().at("lh_goal")->cost->get_residual())
      ->set_reference(lh_Mref_);
}
void OCP::setX0(VectorXd &x0) { x0_ = x0; }
void OCP::setTimeStep(double time_step) { time_step_ = time_step; }
void OCP::setLhId(FrameIndex &lh_id) { lh_id_ = lh_id; }
void OCP::setHorizonLength(int horizon_length) {
  horizon_length_ = horizon_length;
}

void OCP::initOCPParms() {
  state_ =
      boost::make_shared<StateMultibody>(boost::make_shared<Model>(model_));

  state_nv_ = state_->get_nv();
  state_nq_ = state_->get_nq();
  actuation_ = boost::make_shared<ActuationModelFull>(state_);
  actuation_nu_ = actuation_->get_nu();
  contacts_ = boost::make_shared<ContactModelMultiple>(state_, actuation_nu_);
  costs_ = boost::make_shared<CostModelSum>(state_, actuation_nu_);
}

void OCP::buildCostsModel() {
  // Hand position cost
  VectorXd w_hand(6);

  w_hand << VectorXd::Constant(3, 1), VectorXd::Constant(3, 0.0001);

  // Activation for the hand-placement cost
  boost::shared_ptr<ActivationModelWeightedQuad> activ_hand =
      boost::make_shared<ActivationModelWeightedQuad>(w_hand.cwiseAbs2());

  // Residual for the hand-placement cost
  boost::shared_ptr<ResidualModelFramePlacement> res_mod_frm_plmt =
      boost::make_shared<ResidualModelFramePlacement>(state_, lh_id_, lh_Mref_,
                                                      actuation_nu_);

  boost::shared_ptr<CostModelResidual> lh_cost =
      boost::make_shared<CostModelResidual>(state_, activ_hand,
                                            res_mod_frm_plmt);
  // Adding the cost for the left hand
  costs_->addCost("lh_goal", lh_cost, 1e2);

  // Adding state and control regularization terms
  Eigen::VectorXd w_x(2 * state_nv_);
  w_x << Eigen::VectorXd::Zero(3), Eigen::VectorXd::Constant(3, 10.0),
      Eigen::VectorXd::Constant(state_nv_ - 6, 0.01),
      Eigen::VectorXd::Constant(state_nv_, 10.0);

  boost::shared_ptr<ActivationModelWeightedQuad> act_xreg =
      boost::make_shared<ActivationModelWeightedQuad>(w_x.cwiseAbs2());

  // State regularization residual

  boost::shared_ptr<ResidualModelState> res_mod_state_xreg =
      boost::make_shared<ResidualModelState>(state_, x0_, actuation_nu_);

  // Control regularization residual

  boost::shared_ptr<ResidualModelControl> res_mod_ctrl =
      boost::make_shared<ResidualModelControl>(state_, actuation_nu_);

  // State regularization term

  boost::shared_ptr<CostModelResidual> x_reg_cost =
      boost::make_shared<CostModelResidual>(state_, act_xreg,
                                            res_mod_state_xreg);

  boost::shared_ptr<CostModelResidual> u_reg_cost =
      boost::make_shared<CostModelResidual>(state_, res_mod_ctrl);

  // Adding the regularization terms to the cost
  costs_->addCost("xReg", x_reg_cost, 1e-3);  // 1e-3
  costs_->addCost("uReg", u_reg_cost, 1e-4);  // 1e-4

  // Adding the state limits penalization
  Eigen::VectorXd x_lb(state_nq_ + state_nv_);
  x_lb << state_->get_lb().segment(1, state_nv_),
      state_->get_lb().tail(state_nv_);
  Eigen::VectorXd x_ub(state_nq_ + state_nv_);
  x_ub << state_->get_ub().segment(1, state_nv_),
      state_->get_lb().tail(state_nv_);

  boost::shared_ptr<ActivationModelQuadraticBarrier> act_xbounds =
      boost::make_shared<ActivationModelQuadraticBarrier>(
          ActivationBounds(x_lb, x_ub));

  boost::shared_ptr<ResidualModelState> res_mod_state_xbounds =
      boost::make_shared<ResidualModelState>(state_, 0 * x0_, actuation_nu_);

  boost::shared_ptr<CostModelResidual> x_bounds =
      boost::make_shared<CostModelResidual>(state_, act_xbounds,
                                            res_mod_state_xbounds);

  costs_->addCost("xBounds", x_bounds, 1);
}

void OCP::buildDiffActModel() {
  diff_act_model_ =
      boost::make_shared<DifferentialActionModelContactFwdDynamics>(
          state_, actuation_, contacts_, costs_);
}

void OCP::buildSolver() {
  // Creating a running rmodel for the target
  boost::shared_ptr<IntegratedActionModelEuler> action_model =
      boost::make_shared<IntegratedActionModelEuler>(diff_act_model_,
                                                     time_step_);

  std::vector<boost::shared_ptr<ActionModelAbstract>> running_seqs(
      horizon_length_, action_model);

  boost::shared_ptr<IntegratedActionModelEuler> mterm =
      boost::make_shared<IntegratedActionModelEuler>(diff_act_model_, 0.0);

  problem_ = boost::make_shared<ShootingProblem>(x0_, running_seqs, mterm);

  solver_ = boost::make_shared<SolverFDDP>(problem_);
}

void OCP::createCallbacks(CallbackVerbose &callbacks) {
  std::vector<boost::shared_ptr<CallbackAbstract>> shrd_callbacks = {
      boost::make_shared<CallbackVerbose>(callbacks)};
  solver_->setCallbacks(shrd_callbacks);
}

void OCP::solve(const VectorXd &measured_x) {
  solver_->get_problem()->set_x0(measured_x);
  solver_->allocateData();
  solver_->solve(DEFAULT_VECTOR, DEFAULT_VECTOR, 1);
}

void OCP::logSolverData() {
  std::cout << "Number of iterations :" << solver_->get_iter() << std::endl;
  std::cout << "Total cost :" << solver_->get_cost() << std::endl;
  std::cout << "Gradient norm :" << solver_->stoppingCriteria() << std::endl;
  std::cout << "Us[0] :" << get_us().transpose() << std::endl;
  std::cout << "k[0] :" << get_gains().transpose() << std::endl;
}

const Vector3d OCP::get_target() { return (target_); }
const double OCP::get_time_step() { return (time_step_); }
const VectorXd OCP::get_us() { return (solver_->get_us()[0]); }
const VectorXd OCP::get_xs() { return (solver_->get_xs()[0]); }
const MatrixXd OCP::get_gains() { return (solver_->get_K()[0]); }

};  // namespace tiago_OCP
