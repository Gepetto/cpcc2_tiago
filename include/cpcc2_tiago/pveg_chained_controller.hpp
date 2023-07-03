#ifndef PVEG_CHAINED_CONTROLLER_HPP
#define PVEG_CHAINED_CONTROLLER_HPP

// Libraries
#include <algorithm>
#include <controller_interface/chainable_controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "cpcc2_tiago/visibility_control.h"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// auto-generated by generate_parameter_library
#include "cpcc2_tiago_parameters.hpp"

namespace cpcc2_tiago {

/// @brief Chained Controller class to send Reference Interfaces to Higher Level
/// Controller
class PvegChainedController
    : public controller_interface::ChainableControllerInterface {
 public:
  /// @brief Documentation Inherited
  CPCC2_TIAGO_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  /// @brief Documentation Inherited
  CPCC2_TIAGO_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;

  /// @brief Documentation Inherited
  CPCC2_TIAGO_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

 protected:
  /// @brief Export reference_interfaces_ to Higher level controller
  std::vector<hardware_interface::CommandInterface>
  on_export_reference_interfaces() override;

  /// @brief Setting Chained Mode for Controller CROCODDYL_PVEG_CHAINED
  /// @param chained_mode True if CROCODDYL_PVEG_CHAINED Mode Activated, else
  /// False
  /// @return Bool of CROCODDYL_PVEG_CHAINED Mode
  bool on_set_chained_mode(bool chained_mode) override;

  /// @brief Update Interfaces from subscribers. This should be using a realtime
  /// subscriber if CROCODDYL_PVEG_CHAINED mode is false
  /// @return Controller Interface Success
  controller_interface::return_type update_reference_from_subscribers()
      override;

  /// @brief Update Interface from update of High Level Controller.
  /// CROCODDYL_PVEG_CHAINED Mode is true
  /// @param time Current Time
  /// @param period Current Period
  /// @return Controller Interface Success
  controller_interface::return_type update_and_write_commands(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

  /// @brief Update method for both the methods for
  /// @return If Successful then True, else false
  bool update();

  /**
   * Derived controller have to declare parameters in this method.
   * Error handling does not have to be done. It is done in `on_init`-method of
   * this class.
   */
  void declare_parameters();

  /**
   * Derived controller have to read parameters in this method and set
   * `command_interface_types_` variable. The variable is then used to propagate
   * the command interface configuration to controller manager. The method is
   * called from `on_configure`-method of this class.
   *
   * It is expected that error handling of exceptions is done.
   *
   * \returns controller_interface::CallbackReturn::SUCCESS if parameters are
   * successfully read and their values are allowed,
   * controller_interface::CallbackReturn::ERROR otherwise.
   */

  controller_interface::CallbackReturn read_parameters();

  struct ricatti_command {
    std::vector<double> eff_command;
    std::vector<double> vel_command;
    std::vector<double> pos_command;
    double Kp_command;
    double Kv_command;
  };

 private:
  /// @brief Number of joints
  size_t n_joints_;

  /// @brief Vector of Joint Names
  std::vector<std::string> joint_names_;

  /// @brief list of all command interfaces, in this case effort for each joint
  std::vector<std::string> command_interface_types_;

  /// @brief all types of state interface, in our case effort, velocity,
  /// position
  std::vector<std::string> state_interface_types_;

  /// @brief listener object to read ros2 param
  std::shared_ptr<ParamListener> param_listener_;

  /// @brief Params object to list all parameters
  Params params_;

  /// @brief vector of the static friction coefficients for each motor
  std::vector<double> motors_static_friction_;

  /// @brief vector of the viscuous friction coefficients for each motor
  std::vector<double> motors_viscous_friction_;
  /// @brief vector of the motors current to torque coefficient
  std::vector<double> motors_K_tau_;

  /// @brief the ricatti_command is a struct to easily store the command sent by
  /// the crocoddyl controller
  ricatti_command ricatti_command_;

  /// @brief placeholder for computed effort command based on T* = T + Kv*ΔVel +
  /// Kp*ΔPos
  std::vector<double> computed_eff_command_;
  /// @brief placeholder for effort corrected for the motor's friction
  std::vector<double> corrected_eff_command_;

  sensor_msgs::msg::JointState current_state_;
  void read_joints_commands();

  void read_state_from_hardware();

  void compute_ricatti_efforts();

  void correct_efforts_for_friction();
};

template <typename T>
int sign(T val) {
  return (T(0) < val) - (val < T(0));
}

}  // namespace cpcc2_tiago

#endif
