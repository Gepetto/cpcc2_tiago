#include "cpcc2_tiago/pveg_chained_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace cpcc2_tiago {
// Create a parameter listener to listen to published ros2 param
void PvegChainedController::declare_parameters() {
  param_listener_ = std::make_shared<ParamListener>(get_node());
}

// read the parameters and check if they are not empty
controller_interface::CallbackReturn PvegChainedController::read_parameters() {
  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  if (params_.joints.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.state_interfaces_name.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "'state_interfaces_name' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.pveg_command_interfaces_name.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "'command_interfaces_name' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.motors_viscous_friction.empty() ||
      params_.motors_static_friction.empty() || params_.motors_K_tau.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Some motors specs parameters were empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  for (const auto& joint : params_.joints) {
    command_interface_types_.push_back(joint + "/" +
                                       params_.pveg_command_interfaces_name);

    for (long unsigned int s_inter_id = 0;
         s_inter_id < params_.state_interfaces_name.size(); s_inter_id++) {
      state_interface_types_.push_back(
          joint + "/" + params_.state_interfaces_name[s_inter_id]);
    }
  }

  motors_viscous_friction_ = params_.motors_viscous_friction;
  motors_static_friction_ = params_.motors_static_friction;
  motors_K_tau_ = params_.motors_K_tau;
  n_joints_ = params_.joints.size();

  // Resize the vectors to have the correct size, and filling them with
  // quiet_NaN to avoid any misbehaving Reference _interfaces' size is 3 *
  // n_joints_ + 2 -> 3 value for each joint eff, vel, pos + 2 gains Kp, Kv
  reference_interfaces_.resize(3 * n_joints_ + 2,
                               std::numeric_limits<double>::quiet_NaN());

  // same for the current state
  current_state_.position.resize(n_joints_,
                                 std::numeric_limits<double>::quiet_NaN());
  current_state_.velocity.resize(n_joints_,
                                 std::numeric_limits<double>::quiet_NaN());
  current_state_.effort.resize(n_joints_,
                               std::numeric_limits<double>::quiet_NaN());

  ricatti_command_.eff_command.resize(n_joints_,
                                      std::numeric_limits<double>::quiet_NaN());
  ricatti_command_.vel_command.resize(n_joints_,
                                      std::numeric_limits<double>::quiet_NaN());
  ricatti_command_.pos_command.resize(n_joints_,
                                      std::numeric_limits<double>::quiet_NaN());

  corrected_eff_command_.resize(n_joints_,
                                std::numeric_limits<double>::quiet_NaN());
  computed_eff_command_.resize(n_joints_,
                               std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(get_node()->get_logger(),
              "motors parameters loaded successfully");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
cpcc2_tiago::PvegChainedController::on_init() {
  // In the init we try do read the parameters
  try {
    declare_parameters();
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n",
            e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  auto ret = this->read_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
cpcc2_tiago::PvegChainedController::command_interface_configuration() const {
  // Configuration of the command interfaces, be it names and types, stored in
  // the cpcc2_tiago_parameters.yaml file
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names = command_interface_types_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
cpcc2_tiago::PvegChainedController::state_interface_configuration() const {
  // Configuration of the state interfaces, be it names and types, stored in the
  // cpcc2_tiago_parameters.yaml file
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = state_interface_types_;

  state_interfaces_config.names = state_interface_types_;

  return state_interfaces_config;
}

std::vector<hardware_interface::CommandInterface>
cpcc2_tiago::PvegChainedController::on_export_reference_interfaces() {
  RCLCPP_INFO(get_node()->get_logger(), "export_reference_interfaces");

  std::vector<hardware_interface::CommandInterface> reference_interfaces;

  std::string reference_interface_name_Kp = "Kp";
  std::string reference_interface_name_Kv = "Kv";

  // Exporting reference interfaces to Higher Level Controller
  // Note: Name (or prefix name) of the controller should be name of the
  // controller itself. In our case it's "cpcc2_tiago"
  for (size_t i = 0; i < n_joints_; i++) {
    reference_interfaces.push_back(hardware_interface::CommandInterface(
        get_node()->get_name(),
        params_.joints[i] + "/" + hardware_interface::HW_IF_EFFORT,
        &reference_interfaces_[3 * i]));
    reference_interfaces.push_back(hardware_interface::CommandInterface(
        get_node()->get_name(),
        params_.joints[i] + "/" + hardware_interface::HW_IF_VELOCITY,
        &reference_interfaces_[3 * i + 1]));
    reference_interfaces.push_back(hardware_interface::CommandInterface(
        get_node()->get_name(),
        params_.joints[i] + "/" + hardware_interface::HW_IF_POSITION,
        &reference_interfaces_[3 * i + 2]));
  }

  reference_interfaces.push_back(hardware_interface::CommandInterface(
      std::string(get_node()->get_name()), reference_interface_name_Kp,
      &reference_interfaces_[3 * n_joints_]));

  reference_interfaces.push_back(hardware_interface::CommandInterface(
      std::string(get_node()->get_name()), reference_interface_name_Kv,
      &reference_interfaces_[3 * n_joints_ + 1]));

  return reference_interfaces;
}

bool cpcc2_tiago::PvegChainedController::on_set_chained_mode(
    bool chained_mode) {
  RCLCPP_INFO(get_node()->get_logger(), "CHAINED MODE ACTIVE YOUHOUUU");

  chained_mode = true;

  return chained_mode;
}

controller_interface::return_type
cpcc2_tiago::PvegChainedController::update_reference_from_subscribers() {
  RCLCPP_INFO_ONCE(get_node()->get_logger(),
                   "update_reference_from_subscribers");
  return update() ? controller_interface::return_type::OK
                  : controller_interface::return_type::ERROR;
}

controller_interface::return_type
cpcc2_tiago::PvegChainedController::update_and_write_commands(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  RCLCPP_INFO_ONCE(get_node()->get_logger(), "update_and_write_commands");
  return update() ? controller_interface::return_type::OK
                  : controller_interface::return_type::ERROR;
}

bool cpcc2_tiago::PvegChainedController::update() {
  // first we read the current state of the robot
  read_state_from_hardware();
  // then gather the commands from the reference interface
  read_joints_commands();
  // compute the torque base on the command and the current state
  compute_ricatti_efforts();
  // correct for the actuators' friction
  correct_efforts_for_friction();

  for (size_t joint_ind = 0; joint_ind < n_joints_; ++joint_ind) {
    command_interfaces_[joint_ind].set_value(corrected_eff_command_[joint_ind]);
    RCLCPP_INFO(get_node()->get_logger(), (std::to_string(joint_ind)).c_str());
    RCLCPP_INFO(get_node()->get_logger(),
                (std::to_string(corrected_eff_command_[joint_ind]) +
                 std::to_string(999) + std::to_string(joint_ind))
                    .c_str());
  }
  return true;
}

void PvegChainedController::read_joints_commands() {
  double command_eff;
  double command_vel;
  double command_pos;
  double command_Kp;
  double command_Kv;

  for (size_t i = 0; i < n_joints_; i++) {
    command_eff = reference_interfaces_[3 * i];      // arm_i_joint/effort
    command_vel = reference_interfaces_[3 * i + 1];  // arm_i_joint/velocity
    command_pos = reference_interfaces_[3 * i + 2];  // arm_i_joint/position

    // check if NaN, if nan set to current state to avoid large jump in torque
    ricatti_command_.eff_command[i] =
        (command_eff == command_eff) ? command_eff : current_state_.effort[i];
    ricatti_command_.vel_command[i] =
        (command_vel == command_vel) ? command_vel : current_state_.velocity[i];
    ricatti_command_.pos_command[i] =
        (command_pos == command_pos) ? command_pos : current_state_.position[i];
  }

  command_Kp = reference_interfaces_[3 * n_joints_];
  command_Kv = reference_interfaces_[3 * n_joints_ + 1];

  // check if NaN , if nan set to 0
  ricatti_command_.Kp_command = (command_Kp == command_Kp) ? command_Kp : 0;
  ricatti_command_.Kv_command = (command_Kv == command_Kv) ? command_Kv : 0;
}

void PvegChainedController::read_state_from_hardware() {
  // Here we read the state of the robot directly from the hardware interface.
  // We have access to their name so we can sort and find each one
  // Even though we know the states order, this solution add a layer of
  // robustness
  for (size_t i = 0; i < n_joints_; ++i) {
    std::string joint_name = params_.joints[i];
    auto position_state = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(),
        [&joint_name](
            const hardware_interface::LoanedStateInterface& interface) {
          return interface.get_prefix_name() == joint_name &&
                 interface.get_interface_name() ==
                     hardware_interface::HW_IF_POSITION;
        });
    current_state_.position[i] = position_state->get_value();

    auto velocity_state = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(),
        [&joint_name](
            const hardware_interface::LoanedStateInterface& interface) {
          return interface.get_prefix_name() == joint_name &&
                 interface.get_interface_name() ==
                     hardware_interface::HW_IF_VELOCITY;
        });
    current_state_.velocity[i] = velocity_state->get_value();

    auto effort_state = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(),
        [&joint_name](
            const hardware_interface::LoanedStateInterface& interface) {
          return interface.get_prefix_name() == joint_name &&
                 interface.get_interface_name() ==
                     hardware_interface::HW_IF_EFFORT;
        });
    current_state_.effort[i] = effort_state->get_value();
  }
}

void PvegChainedController::compute_ricatti_efforts() {
  for (size_t i = 0; i < n_joints_; i++) {
    computed_eff_command_[i] =
        ricatti_command_.eff_command[i] +
        ricatti_command_.Kv_command *
            (ricatti_command_.vel_command[i] - current_state_.velocity[i]) +
        ricatti_command_.Kp_command *
            (ricatti_command_.pos_command[i] - current_state_.position[i]);
  }
}

void PvegChainedController::correct_efforts_for_friction() {
  for (size_t i = 0; i < n_joints_; i++) {
    corrected_eff_command_[i] =
        computed_eff_command_[i] +
        motors_static_friction_[i] * sign(current_state_.velocity[i]) +
        motors_viscous_friction_[i] * current_state_.velocity[i];
  }
}

}  // namespace cpcc2_tiago

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(cpcc2_tiago::PvegChainedController,
                       controller_interface::ChainableControllerInterface)
