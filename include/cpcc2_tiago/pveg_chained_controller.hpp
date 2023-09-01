#ifndef PVEG_CHAINED_CONTROLLER_HPP
#define PVEG_CHAINED_CONTROLLER_HPP

#include "boost/interprocess/containers/vector.hpp"
#include "boost/interprocess/managed_shared_memory.hpp"
#include "boost/interprocess/shared_memory_object.hpp"
#include "boost/interprocess/sync/named_mutex.hpp"
#include "boost/thread/thread_time.hpp"

#include "controller_interface/chainable_controller_interface.hpp"
#include "cpcc2_tiago/model_builder.hpp"
#include "cpcc2_tiago/visibility_control.h"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pinocchio/algorithm/parallel/aba.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// auto-generated by generate_parameter_library
#include "cpcc2_tiago_parameters.hpp"

namespace cpcc2_tiago {

/// @brief Chained Controller class to send Reference Interfaces to Higher
/// Level Controller
class PvegChainedController
    : public controller_interface::ChainableControllerInterface {
public:
  /// @brief Documentation Inherited
  CPCC2_TIAGO_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  /// @brief Documentation Inherited
  CPCC2_TIAGO_PUBLIC
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  /// @brief Documentation Inherited
  CPCC2_TIAGO_PUBLIC
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

protected:
  /// @brief Export reference_interfaces_ to Higher level controller
  std::vector<hardware_interface::CommandInterface>
  on_export_reference_interfaces() override;

  /// @brief Setting Chained Mode for Controller CROCODDYL_PVEG_CHAINED
  /// @param chained_mode True if CROCODDYL_PVEG_CHAINED Mode Activated, else
  /// False
  /// @return Bool of CROCODDYL_PVEG_CHAINED Mode
  bool on_set_chained_mode(bool chained_mode) override;

  /// @brief Update Interfaces from subscribers. This should be using a
  /// realtime subscriber if CROCODDYL_PVEG_CHAINED mode is false
  /// @return Controller Interface Success
  controller_interface::return_type
  update_reference_from_subscribers() override;

  /// @brief Update Interface from update of High Level Controller.
  /// CROCODDYL_PVEG_CHAINED Mode is true
  /// @param time Current Time
  /// @param period Current Period
  /// @return Controller Interface Success
  controller_interface::return_type
  update_and_write_commands(const rclcpp::Time &time,
                            const rclcpp::Duration &period) override;

  /// @brief Update method for both the methods for
  /// @return If Successful then True, else false
  bool update();

  /**
   * Derived controller have to declare parameters in this method.
   * Error handling does not have to be done. It is done in `on_init`-method
   * of this class.
   */
  void declare_parameters();

  /**
   * Derived controller have to read parameters in this method and set
   * `command_interface_types_` variable. The variable is then used to
   * propagate the command interface configuration to controller manager. The
   * method is called from `on_configure`-method of this class.
   *
   * It is expected that error handling of exceptions is done.
   *
   * \returns controller_interface::CallbackReturn::SUCCESS if parameters are
   * successfully read and their values are allowed,
   * controller_interface::CallbackReturn::ERROR otherwise.
   */
  controller_interface::CallbackReturn read_parameters();

private:
  /// @brief struct to hold the different Ricatti commands
  struct ricatti_command {
    Eigen::VectorXd u_command;
    Eigen::VectorXd x0_command;
    Eigen::VectorXd xinter_command;
    Eigen::VectorXd x1_command;
    Eigen::MatrixXd K_command;

    bool operator==(const ricatti_command &rhs) const {
      return (u_command == rhs.u_command && x0_command == rhs.x0_command &&
              x1_command == rhs.x1_command && K_command == rhs.K_command);
    }

    bool operator!=(const ricatti_command &rhs) const {
      return (u_command != rhs.u_command || x0_command != rhs.x0_command ||
              x1_command != rhs.x1_command || K_command != rhs.K_command);
    };
  };

  /// @brief struct to hold the current state of the robot
  struct state {
    Eigen::VectorXd position;
    Eigen::VectorXd velocity;
  };

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      ricatti_command_pub_;

  std_msgs::msg::Float64MultiArray ricatti_command_msg_;

  /// @brief shared mutex to prevent miswriting on used variable
  boost::interprocess::named_mutex mutex_{boost::interprocess::open_or_create,
                                          "crocoddyl_mutex"};

  boost::interprocess::managed_shared_memory crocoddyl_shm_;

  bool *start_sending_cmd_shm_;
  bool start_sending_cmd_ = false;

  rclcpp::Time start_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time prev_command_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  double interpolate_t_ = 0.0;

  /// @brief Pinocchio Model
  Model model_;

  /// @brief Pinocchio Data
  Data data_;

  /// @brief Eigen::VectorXd to store the current state of the robot
  Eigen::VectorXd measuredX_;

  /// @brief Eigen::VectorXd to store the computed ricatti command the robot
  Eigen::VectorXd eff_command_;

  /// @brief Eigen::VectorXd to store the command adapted to the conmmand
  /// interfaces
  Eigen::VectorXd command_;

  /// @brief Eigen::VectorXd to store the interpolated xs
  Eigen::VectorXd interpolated_xs_;

  /// @brief ricatti_command to store the last command
  ricatti_command ricatti_command_;

  /// @brief ricatti_command to store the interpolated command
  ricatti_command interpolated_ricatti_command_;

  /// @brief ricatti_command to store the last command
  ricatti_command last_ricatti_command_;

  /// @brief list of all command interfaces, in this case effort for each
  /// joint
  std::vector<std::string> command_interface_types_;

  /// @brief all types of state interface, in our case velocity,
  /// position
  std::vector<std::string> state_interface_types_;

  /// @brief listener object to read ros2 param
  std::shared_ptr<ParamListener> param_listener_;

  /// @brief Params object to list all parameters
  Params params_;

  /// @brief Number of joints
  int n_joints_;

  /// @brief state to store the current state of the robot
  state current_state_;

  /// @brief Initialize the shared memory, find the vector and tie them to
  /// variables
  void init_shared_memory();

  /// @brief Read the command from the reference interfaces
  /// @param ric_com Ricatti command to write to
  void read_joints_commands(ricatti_command &ric_com);

  /// @brief Read the state from the hardware
  /// @param curr_state Current State to write to
  void read_state_from_hardware(state &curr_state);

  /// @brief Compute the ricatti command u = u* + K*(x - x*)
  /// @param ric_cmd Ricatti command
  /// @param x Current State
  Eigen::VectorXd compute_ricatti_command(ricatti_command ric_cmd,
                                          Eigen::VectorXd x);

  /// @brief Interpolate the ricatti command q = q0 + v0*t + 1/2*a*t^2
  /// v = v0 + a*t
  /// @param x0 Initial State
  /// @param ddq Acceleration
  /// @param t Time
  Eigen::VectorXd aba_interpolate_xs(Eigen::VectorXd x0, Eigen::VectorXd ddq,
                                     double t);

  /// @brief Interpolate the ricatti command linearly between x0 and x1
  /// @param x0 Initial State
  /// @param x1 Final State
  /// @param t Time
  Eigen::VectorXd lin_interpolate_xs(Eigen::VectorXd x0, Eigen::VectorXd x1,
                                     double t);

  /// @brief Adapt the command to the command interfaces types
  /// @param eff_command Command to adapt
  /// @param ric_cmd Ricatti command to get the other command types
  Eigen::VectorXd adapt_command_to_type(Eigen::VectorXd eff_command,
                                        ricatti_command ric_cmd);

  /// @brief Set the adapted command to the command interfaces
  /// @param command Command to set, contain effort, pos or vel
  void set_command(Eigen::VectorXd command);
};

} // namespace cpcc2_tiago

#endif
