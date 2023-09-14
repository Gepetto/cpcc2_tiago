#include <cpcc2_tiago/tiago_OCP.hpp>

namespace tiago_OCP {

void OCP::defineHandTask(boost::shared_ptr<crocoddyl::CostModelSum> cost,
                         Eigen::VectorXd w_hand, double lh_cost_weight) {
  // Activation for the hand-placement cost
  boost::shared_ptr<croc::ActivationModelWeightedQuad> activ_hand =
      boost::make_shared<croc::ActivationModelWeightedQuad>(w_hand.cwiseAbs2());

  // Residual for the hand-placement cost
  boost::shared_ptr<croc::ResidualModelFramePlacement> res_mod_frm_plmt =
      boost::make_shared<croc::ResidualModelFramePlacement>(
          state_, lh_id_, lh_Mref_, actuation_nu_);

  boost::shared_ptr<croc::CostModelResidual> lh_cost =
      boost::make_shared<croc::CostModelResidual>(state_, activ_hand,
                                                  res_mod_frm_plmt);
  // Adding the cost for the left hand
  cost.get()->addCost("lh_goal", lh_cost, lh_cost_weight);
}

void OCP::defineXReg(boost::shared_ptr<crocoddyl::CostModelSum> cost,
                     Eigen::VectorXd w_x, double xReg_weight) {
  boost::shared_ptr<croc::ActivationModelWeightedQuad> act_xreg =
      boost::make_shared<croc::ActivationModelWeightedQuad>(w_x.cwiseAbs2());

  // State regularization residual

  boost::shared_ptr<croc::ResidualModelState> res_mod_state_xreg =
      boost::make_shared<croc::ResidualModelState>(state_, x0_, actuation_nu_);

  // State regularization term

  boost::shared_ptr<croc::CostModelResidual> x_reg_cost =
      boost::make_shared<croc::CostModelResidual>(state_, act_xreg,
                                                  res_mod_state_xreg);

  // Adding the regularization terms to the cost
  cost.get()->addCost("xReg", x_reg_cost, xReg_weight);
}

void OCP::defineUReg(boost::shared_ptr<crocoddyl::CostModelSum> cost,
                     double uReg_weight) {
  boost::shared_ptr<croc::ResidualModelControl> res_mod_ctrl =
      boost::make_shared<croc::ResidualModelControl>(state_, actuation_nu_);

  boost::shared_ptr<croc::CostModelResidual> u_reg_cost =
      boost::make_shared<croc::CostModelResidual>(state_, res_mod_ctrl);
  // Adding the regularization terms to the cost
  cost.get()->addCost("uReg", u_reg_cost, uReg_weight);
}

void OCP::defineXbounds(boost::shared_ptr<crocoddyl::CostModelSum> cost,
                        double xBounds_weight) {
  // Adding the state limits penalization
  Eigen::VectorXd x_lb(state_nq_ + state_nv_);
  x_lb << state_->get_lb().segment(1, state_nv_),
      state_->get_lb().tail(state_nv_);
  Eigen::VectorXd x_ub(state_nq_ + state_nv_);
  x_ub << state_->get_ub().segment(1, state_nv_),
      state_->get_ub().tail(state_nv_);

  boost::shared_ptr<croc::ActivationModelQuadraticBarrier> act_xbounds =
      boost::make_shared<croc::ActivationModelQuadraticBarrier>(
          croc::ActivationBounds(x_lb, x_ub));

  boost::shared_ptr<croc::ResidualModelState> res_mod_state_xbounds =
      boost::make_shared<croc::ResidualModelState>(state_, 0 * x0_,
                                                   actuation_nu_);

  boost::shared_ptr<croc::CostModelResidual> x_bounds =
      boost::make_shared<croc::CostModelResidual>(state_, act_xbounds,
                                                  res_mod_state_xbounds);

  cost.get()->addCost("xBounds", x_bounds, xBounds_weight);
}

}  // namespace tiago_OCP
