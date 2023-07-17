#include "cpcc2_tiago/tiago_OCP_maker.hpp"

namespace tiago_OCP {

OCP::OCP() {}

OCP::OCP(const Model model, const Data data) {
  model_ = model;
  data_ = data;
  initOCPParms();
}

void OCP::setTarget(Vector3d target) {
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

void OCP::setLhId(FrameIndex lh_id) { lh_id_ = lh_id; }
void OCP::setX0(VectorXd x0) { x0_ = x0; }
void OCP::setTimeStep(double time_step) { time_step_ = time_step; }
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

void OCP::buildCostsModel(std::map<std::string, double> costs_weights,
                          Eigen::VectorXd w_hand, Eigen::VectorXd w_x) {
  // Hand position cost

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
  costs_->addCost("lh_goal", lh_cost, costs_weights["lh_goal_weight"]);

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
  costs_->addCost("xReg", x_reg_cost,
                  costs_weights["xReg_weight"]); // 1e-3
  costs_->addCost("uReg", u_reg_cost,
                  costs_weights["uReg_weight"]); // 1e-4

  // Adding the state limits penalization
  Eigen::VectorXd x_lb(state_nq_ + state_nv_);
  x_lb << state_->get_lb().segment(1, state_nv_),
      state_->get_lb().tail(state_nv_);
  Eigen::VectorXd x_ub(state_nq_ + state_nv_);
  x_ub << state_->get_ub().segment(1, state_nv_),
      state_->get_ub().tail(state_nv_);

  boost::shared_ptr<ActivationModelQuadraticBarrier> act_xbounds =
      boost::make_shared<ActivationModelQuadraticBarrier>(
          ActivationBounds(x_lb, x_ub));

  boost::shared_ptr<ResidualModelState> res_mod_state_xbounds =
      boost::make_shared<ResidualModelState>(state_, 0 * x0_, actuation_nu_);

  boost::shared_ptr<CostModelResidual> x_bounds =
      boost::make_shared<CostModelResidual>(state_, act_xbounds,
                                            res_mod_state_xbounds);

  costs_->addCost("xBounds", x_bounds, costs_weights["xBounds_weight"]);
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

void OCP::solve(VectorXd measured_x) {
  warm_xs_ = solver_->get_xs();
  warm_xs_.erase(warm_xs_.begin());
  warm_xs_[0] = measured_x;
  warm_xs_.push_back(warm_xs_[warm_xs_.size() - 1]);

  warm_us_ = solver_->get_us();
  warm_us_.erase(warm_us_.begin());
  warm_us_.push_back(warm_us_[warm_us_.size() - 1]);

  solver_->get_problem()->set_x0(measured_x);
  solver_->allocateData();
  solver_->solve(warm_xs_, warm_us_, 1);
}

void OCP::logSolverData() {
  std::cout << "Total cost :" << solver_->get_cost()
            << " Stopping criteria :" << solver_->stoppingCriteria()
            << std::endl;
}

const Vector3d OCP::get_target() { return (target_); }
double OCP::get_time_step() { return (time_step_); }
int OCP::get_horizon_length() { return (horizon_length_); }
const std::vector<VectorXd> OCP::get_us() { return (solver_->get_us()); }
const std::vector<VectorXd> OCP::get_xs() { return (solver_->get_xs()); }
const Eigen::MatrixXd OCP::get_gains() { return (solver_->get_K()[0]); }

}; // namespace tiago_OCP
