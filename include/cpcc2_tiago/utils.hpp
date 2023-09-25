#pragma once

// Eigen
#include <Eigen/Dense>

namespace pinocchio {}
namespace crocoddyl {}

namespace cpcc2_tiago {

namespace pin = pinocchio;
namespace croc = crocoddyl;

// this struct help to encapsulate the ricatti command
struct RicattiCommand {
  Eigen::VectorXd u_command;
  Eigen::VectorXd x0_command;
  Eigen::VectorXd xinter_command;
  Eigen::VectorXd x1_command;
  Eigen::MatrixXd K_command;

  inline RicattiCommand() = default;

  inline RicattiCommand(int n_joints)
      : u_command(n_joints),
        x0_command(2 * n_joints),
        xinter_command(2 * n_joints),
        x1_command(2 * n_joints),
        K_command(n_joints, 2 * n_joints) {}

  inline bool operator==(const RicattiCommand &rhs) const {
    return (u_command == rhs.u_command && x0_command == rhs.x0_command &&
            x1_command == rhs.x1_command && K_command == rhs.K_command);
  }

  inline bool operator!=(const RicattiCommand &rhs) const {
    return !(*this == rhs);
  };
};

/// @brief struct to hold the current state of the robot
struct State {
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;

  inline State() = default;
  inline State(int n_joints) : position(n_joints), velocity(n_joints) {}
};

// circular vector to store values over time and calculate the rolling mean
template <Eigen::Index Size>
class CircularVector : private Eigen::Vector<double, Size> {
 public:
  using super_type = Eigen::Vector<double, Size>;
  using size_type = Eigen::Index;

  static_assert(Size > 0);

  inline CircularVector() { super_type::setZero(); }

  inline void append(double new_value) {
    super_type::operator[](end_++) = new_value;
    if (end_ > Size) end_ -= Size;
  }

  inline double mean() const { return super_type::mean(); }

 private:
  Eigen::Index end_ = 0;
};

}  // namespace cpcc2_tiago
