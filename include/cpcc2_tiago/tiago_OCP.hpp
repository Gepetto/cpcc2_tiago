#ifndef TIAGO_OCP_MAKER_HPP
#define TIAGO_OCP_MAKER_HPP

#include "crocoddyl/core/activations/quadratic-barrier.hpp"
#include "crocoddyl/core/activations/weighted-quadratic.hpp"
#include "crocoddyl/core/costs/cost-sum.hpp"
#include "crocoddyl/core/costs/residual.hpp"
#include "crocoddyl/core/integrator/euler.hpp"
#include "crocoddyl/core/residuals/control.hpp"
#include "crocoddyl/core/solvers/fddp.hpp"
#include "crocoddyl/multibody/actions/contact-fwddyn.hpp"
#include "crocoddyl/multibody/actuations/full.hpp"
#include "crocoddyl/multibody/contacts/contact-6d.hpp"
#include "crocoddyl/multibody/contacts/multiple-contacts.hpp"
#include "crocoddyl/multibody/residuals/frame-placement.hpp"
#include "crocoddyl/multibody/residuals/state.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"
#include "pinocchio//spatial/fwd.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/model.hpp"

using namespace pinocchio;
using namespace crocoddyl;
using namespace Eigen;

namespace tiago_OCP {
class OCP {
private:
  boost::shared_ptr<ShootingProblem> problem_;

  boost::shared_ptr<SolverFDDP> solver_;
  Model model_;
  Data data_;

  std::vector<VectorXd> warm_xs_;
  std::vector<VectorXd> warm_us_;

  boost::shared_ptr<StateMultibody> state_;
  boost::shared_ptr<ActuationModelFull> actuation_;
  boost::shared_ptr<ContactModelMultiple> contacts_;

  boost::shared_ptr<DifferentialActionModelContactFwdDynamics> diff_act_model_;

  size_t horizon_length_;
  double time_step_;
  int solver_iterations_;

  size_t actuation_nu_;
  size_t state_nv_;
  size_t state_nq_;

  std::map<std::string, double> costs_weights_;
  VectorXd w_hand_;
  VectorXd w_x_;

  Vector3d target_;
  VectorXd x0_;
  VectorXd balancing_torques_;

  SE3 lh_Mref_;

  FrameIndex lh_id_;

public:
  OCP();
  OCP(const Model model, const Data data);

  /// @brief initialize the parameters for the OCP, state, actuation and
  /// contacts (empty)
  void initOCPParms();

  void setX0(VectorXd x0) { x0_ = x0; };
  void setTimeStep(double time_step) { time_step_ = time_step; };
  void setHorizonLength(int horizon_length) {
    horizon_length_ = horizon_length;
    warm_xs_.resize(horizon_length_ + 1);
    warm_us_.resize(horizon_length_);
  };
  void setSolverIterations(int iterations) { solver_iterations_ = iterations; };

  void setCostsWeights(std::map<std::string, double> costs_weights) {
    costs_weights_ = costs_weights;
  };
  void setCostsActivationWeights(Eigen::VectorXd w_hand, Eigen::VectorXd w_x) {
    w_hand_ = w_hand;
    w_x_ = w_x;
  };
  void setLhId(FrameIndex lh_id) { lh_id_ = lh_id; };

  /// @brief Define the cost function for the hand task
  /// @param cost cost object to add the hand task
  /// @param w_hand activation weights for the hand task
  /// @param lh_cost_weight cost weight for the hand task
  void defineHandTask(boost::shared_ptr<crocoddyl::CostModelSum> &cost,
                      Eigen::VectorXd w_hand, double lh_cost_weight);

  /// @brief Define the cost function for the state task
  /// @param cost cost object to add the state task
  /// @param w_x activation weights for the state task
  /// @param xReg_weight cost weight for the state task
  void defineXReg(boost::shared_ptr<crocoddyl::CostModelSum> &cost,
                  Eigen::VectorXd w_x, double xReg_weight);

  /// @brief Define the cost function for the control task
  /// @param cost cost object to add the control task
  /// @param uReg_weight cost weight for the control task
  void defineUReg(boost::shared_ptr<crocoddyl::CostModelSum> &cost,
                  double uReg_weight);

  /// @brief Define the cost function for the state bounds
  /// @param cost cost object to add the state bounds
  /// @param xBounds_weight cost weight for the state bounds
  void defineXbounds(boost::shared_ptr<crocoddyl::CostModelSum> &cost,
                     double xBounds_weight);

  /// @brief build a running model for the OCP
  /// @return boost::shared_ptr<ActionModelAbstract> running model
  boost::shared_ptr<ActionModelAbstract> buildRunningModel();

  /// @brief build a terminal model for the OCP
  /// @return boost::shared_ptr<ActionModelAbstract> terminal model
  boost::shared_ptr<ActionModelAbstract> buildTerminalModel();

  /// @brief recede the cost horioon, like a circular buffer
  void recede();

  /// @brief update the reference for the last running model
  void updateRunModHandReference();

  void updateRunModXRegReference(std::vector<VectorXd> x_Xreg);

  /// @brief compute the balancing torques for the current state
  /// @param x0 current state
  Eigen::VectorXd computeBalancingTorques(Eigen::VectorXd x0);

  /// @brief build the solver for the OCP
  /// @param x0 initial state
  /// @param target target for the hand task
  void buildSolver(Eigen::VectorXd x0, Eigen::Vector3d target);

  /// @brief first solve of the OCP, it tries to stay where the robot is
  /// @param measured_x current state
  void solveFirst(VectorXd measured_x);

  /// @brief solve the OCP
  /// @param measured_x current state
  void solve(VectorXd measured_x);

  /// @brief change the target for all costs of the runnning model
  void setTarget(Vector3d target);

  /// @brief change the target for the last running model
  void changeTarget(Vector3d target);

  void printCosts() {
    std::cout << "First cost: " << *(costs(0)) << std::endl;
  };

  /// @brief get the action model for the OCP of a node
  /// @param node_id  index of the node
  boost::shared_ptr<crocoddyl::ActionModelAbstract>
  ama(const unsigned long node_id);

  /// @brief get the integrated action model for the OCP of a node
  /// @param node_id  index of the node
  boost::shared_ptr<crocoddyl::IntegratedActionModelEuler>
  iam(const unsigned long node_id);

  /// @brief get the differencial action model for the OCP of a node
  /// @param node_id  index of the node
  boost::shared_ptr<crocoddyl::DifferentialActionModelContactFwdDynamics>
  dam(const unsigned long node_id);

  /// @brief get the cost model for the OCP of a node
  /// @param node_id  index of the node
  boost::shared_ptr<crocoddyl::CostModelSum> costs(const unsigned long node_id);

  /// @brief get the action data for the OCP of a node
  /// @param node_id  index of the node
  boost::shared_ptr<crocoddyl::ActionDataAbstract>
  ada(const unsigned long node_id);

  /// @brief return target
  Vector3d get_target() { return (target_); };
  /// @brief return the solver time step
  double get_time_step() { return (time_step_); };
  /// @brief return the solver horizon length
  int get_horizon_length() { return (horizon_length_); };
  /// @brief return the solver us
  std::vector<VectorXd> get_us() { return (solver_->get_us()); };
  /// @brief return the solver xs
  std::vector<VectorXd> get_xs() { return (solver_->get_xs()); };
  /// @brief return the solver gains
  Eigen::MatrixXd get_gains() { return (solver_->get_K()[0]); };
  /// @brief return the balancing torques
  Eigen::VectorXd get_balancing_torques() { return balancing_torques_; };
  /// @brief return the solver state
  StateMultibody get_state() { return *state_; }
  /// @brief return the solver actuation
  ActuationModelFull get_actuation() { return *actuation_; }
  /// @brief return the solver problem
  boost::shared_ptr<ShootingProblem> get_problem() { return problem_; }
  /// @brief return the solver
  SolverFDDP get_solver() { return *solver_; }
};
}; // namespace tiago_OCP

#endif
