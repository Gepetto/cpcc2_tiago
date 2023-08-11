#ifndef TIAGO_OCP_MAKER_HPP
#define TIAGO_OCP_MAKER_HPP

#include "crocoddyl/core/activations/quadratic-barrier.hpp"
#include "crocoddyl/core/activations/weighted-quadratic.hpp"
#include "crocoddyl/core/costs/cost-sum.hpp"
#include "crocoddyl/core/costs/residual.hpp"
#include "crocoddyl/core/integrator/euler.hpp"
#include "crocoddyl/core/mathbase.hpp"
#include "crocoddyl/core/optctrl/shooting.hpp"
#include "crocoddyl/core/residuals/control.hpp"
#include "crocoddyl/core/solvers/fddp.hpp"
#include "crocoddyl/core/utils/callbacks.hpp"
#include "crocoddyl/multibody/actions/contact-fwddyn.hpp"
#include "crocoddyl/multibody/actuations/full.hpp"
#include "crocoddyl/multibody/contacts/contact-6d.hpp"
#include "crocoddyl/multibody/contacts/multiple-contacts.hpp"
#include "crocoddyl/multibody/residuals/frame-placement.hpp"
#include "crocoddyl/multibody/residuals/state.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"
#include "pinocchio//spatial/fwd.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial/se3-tpl.hpp"

using namespace pinocchio;
using namespace crocoddyl;
using namespace Eigen;

namespace tiago_OCP {
class OCP {
private:
  void initOCPParms();

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

  void defineHandTask(boost::shared_ptr<crocoddyl::CostModelSum> &cost,
                      Eigen::VectorXd w_hand, double lh_cost_weight);
  void defineXReg(boost::shared_ptr<crocoddyl::CostModelSum> &cost,
                  Eigen::VectorXd w_x, double xReg_weight);
  void defineUReg(boost::shared_ptr<crocoddyl::CostModelSum> &cost,
                  double uReg_weight);

  void defineXbounds(boost::shared_ptr<crocoddyl::CostModelSum> &cost,
                     double xBounds_weight);

  boost::shared_ptr<ActionModelAbstract> buildRunningModel();
  boost::shared_ptr<ActionModelAbstract> buildTerminalModel();

  void recede();

  Eigen::VectorXd computeBalancingTorques(Eigen::VectorXd x0);

  void buildSolver(Eigen::VectorXd x0, Eigen::Vector3d target);

  void solveFirst(VectorXd measured_x);
  void solve(VectorXd measured_x);

  boost::shared_ptr<crocoddyl::ActionModelAbstract>
  ama(const unsigned long time);
  boost::shared_ptr<crocoddyl::IntegratedActionModelEuler>
  iam(const unsigned long time);
  boost::shared_ptr<crocoddyl::DifferentialActionModelContactFwdDynamics>
  dam(const unsigned long time);
  boost::shared_ptr<crocoddyl::CostModelSum> costs(const unsigned long time);
  boost::shared_ptr<crocoddyl::ActionDataAbstract>
  ada(const unsigned long time);

  void setTarget(Vector3d target);

  void changeTarget(Vector3d target);
  void updateRunModReference();

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

  void printProblem() { std::cout << *problem_ << std::endl; };
  void printCosts() {
    std::cout << "First cost: " << *(costs(0)) << std::endl;
  };

  const Vector3d get_target() { return (target_); };
  double get_time_step() { return (time_step_); };
  int get_horizon_length() { return (horizon_length_); };
  const std::vector<VectorXd> get_us() { return (solver_->get_us()); };
  const std::vector<VectorXd> get_xs() { return (solver_->get_xs()); };
  const Eigen::MatrixXd get_gains() { return (solver_->get_K()[0]); };
  Eigen::VectorXd get_balancing_torques() { return balancing_torques_; };
  StateMultibody get_state() { return *state_; }
  ActuationModelFull get_actuation() { return *actuation_; }
  boost::shared_ptr<ShootingProblem> get_problem() { return problem_; }
  SolverFDDP get_solver() { return *solver_; }
};
}; // namespace tiago_OCP

#endif
