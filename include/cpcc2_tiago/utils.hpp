#pragma once

// Eigen
#include <Eigen/Dense>
// boost
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

namespace pinocchio {}
namespace crocoddyl {}

namespace cpcc2_tiago {

namespace pin = pinocchio;
namespace croc = crocoddyl;

using shared_double_allocator = boost::interprocess::allocator<
    double, boost::interprocess::managed_shared_memory::segment_manager>;
using shared_char_allocator = boost::interprocess::allocator<
    char, boost::interprocess::managed_shared_memory::segment_manager>;
using shared_string =
    boost::interprocess::basic_string<char, std::char_traits<char>,
                                      shared_char_allocator>;
using shared_vector =
    boost::interprocess::vector<double, shared_double_allocator>;

// this struct help to encapsulate the ricatti command
struct RicattiCommand {
  Eigen::VectorXd u_command;
  Eigen::VectorXd x0_command;
  Eigen::VectorXd xinter_command;
  Eigen::VectorXd x1_command;
  Eigen::MatrixXd K_command;

  RicattiCommand() {}

  RicattiCommand(int n_joints) {
    u_command.resize(n_joints);
    x0_command.resize(2 * n_joints);
    xinter_command.resize(2 * n_joints);
    x1_command.resize(2 * n_joints);
    K_command.resize(n_joints, 2 * n_joints);
  }

  bool operator==(const RicattiCommand &rhs) const {
    return (u_command == rhs.u_command && x0_command == rhs.x0_command &&
            x1_command == rhs.x1_command && K_command == rhs.K_command);
  }

  bool operator!=(const RicattiCommand &rhs) const {
    return (u_command != rhs.u_command || x0_command != rhs.x0_command ||
            x1_command != rhs.x1_command || K_command != rhs.K_command);
  };
};

/// @brief struct to hold the current state of the robot
struct State {
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;

  State() {}
  State(int n_joints) {
    position.resize(n_joints);
    velocity.resize(n_joints);
  }
};

// circular vector to store values over time and calculate the rolling mean
struct CircularVector {
  Eigen::VectorXd vector;

  CircularVector(int size) : vector(size) { vector.setZero(); }

  void circular_append(double new_value) {
    Eigen::VectorXd shifted_vector(vector.size());
    for (int i = 0; i < vector.size() - 1; ++i) {
      shifted_vector(i) = vector(i + 1);
    }
    shifted_vector(vector.size() - 1) = new_value;
    vector = shifted_vector;
  }
};

}  // namespace cpcc2_tiago
