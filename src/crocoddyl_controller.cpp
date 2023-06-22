#include "cpcc2_tiago/crocoddyl_controller.hpp"

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

void CrocoddylController::declare_parameters() {
  param_listener_ = std::make_shared<ParamListener>(get_node());
}

controller_interface::CallbackReturn CrocoddylController::read_parameters() {
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

  for (const auto& joint : params_.joints) {
    for (long unsigned int s_inter_id = 0;
         s_inter_id < params_.state_interfaces_name.size(); s_inter_id++) {
      state_interface_types_.push_back(
          joint + "/" + params_.state_interfaces_name[s_inter_id]);
    }
  }
  n_joints_ = params_.joints.size();

  // same for the current state
  current_state.position.resize(n_joints_,
                                std::numeric_limits<double>::quiet_NaN());
  current_state.velocity.resize(n_joints_,
                                std::numeric_limits<double>::quiet_NaN());
  current_state.effort.resize(n_joints_,
                              std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(get_node()->get_logger(),
              "motors parameters loaded successfully");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
cpcc2_tiago::CrocoddylController::on_init() {
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

  if (n_joints_ == 0) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "List of joint names is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
cpcc2_tiago::CrocoddylController::command_interface_configuration() const {
  RCLCPP_INFO(get_node()->get_logger(),
              "Command Interface CrocoddylController");

  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;

  for (size_t i = 0; i < n_joints_; i++) {
    // Claiming Command interface exported as reference interface by
    // PvegContrller
    command_interfaces_config.names.push_back("pveg_chained_controller/" +
                                              params_.joints[i] + "/" +
                                              hardware_interface::HW_IF_EFFORT);
    command_interfaces_config.names.push_back(
        "pveg_chained_controller/" + params_.joints[i] + "/" +
        hardware_interface::HW_IF_VELOCITY);
    command_interfaces_config.names.push_back(
        "pveg_chained_controller/" + params_.joints[i] + "/" +
        hardware_interface::HW_IF_POSITION);
  }

  command_interfaces_config.names.push_back("pveg_chained_controller/Kp");
  command_interfaces_config.names.push_back("pveg_chained_controller/Kv");

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
cpcc2_tiago::CrocoddylController::state_interface_configuration() const {
  RCLCPP_INFO(get_node()->get_logger(), "State Interface CrocoddylController.");

  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = state_interface_types_;

  state_interfaces_config.names = state_interface_types_;
  return state_interfaces_config;
}

controller_interface::return_type cpcc2_tiago::CrocoddylController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  std::vector<double> eff = {0, 0, 0, 0, 0, 0, 0};
  std::vector<double> vel = {0, 0, 0, 0, 0, 0, 0};
  std::vector<double> pos = {1.5, 1, 1, 1, 1, 1, 1};
  set_eff_command(eff);
  set_vel_command(vel);
  set_pos_command(pos);
  set_gains_command(10, 1);
  return controller_interface::return_type::OK;
}

void CrocoddylController::read_state_from_hardware() {
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

void CrocoddylController::set_eff_command(std::vector<double> command_eff) {
  for (size_t joint_ind = 0; joint_ind < n_joints_; ++joint_ind) {
    command_interfaces_[3 * joint_ind].set_value(command_eff[joint_ind]);
  }
}
void CrocoddylController::set_vel_command(std::vector<double> command_vel) {
  for (size_t joint_ind = 0; joint_ind < n_joints_; ++joint_ind) {
    command_interfaces_[3 * joint_ind + 1].set_value(command_vel[joint_ind]);
  }
}
void CrocoddylController::set_pos_command(std::vector<double> command_pos) {
  for (size_t joint_ind = 0; joint_ind < n_joints_; ++joint_ind) {
    command_interfaces_[3 * joint_ind + 2].set_value(command_pos[joint_ind]);
  }
}

void CrocoddylController::set_gains_command(double command_Kp,
                                            double command_Kv) {
  command_interfaces_[3 * n_joints_].set_value(command_Kp);
  command_interfaces_[3 * n_joints_ + 1].set_value(command_Kv);
}

}  // namespace cpcc2_tiago

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(cpcc2_tiago::CrocoddylController,
                       controller_interface::ControllerInterface)
