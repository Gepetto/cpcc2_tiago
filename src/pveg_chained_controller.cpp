#include "cpcc2_tiago/pveg_chained_controller.hpp"

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

  if (params_.arm_motors_viscous_friction.empty() ||
      params_.arm_motors_static_friction.empty() ||
      params_.arm_motors_K_tau.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Some motors specs parameters were empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  for (int i = 0; i < params_.joints.size(); i++) {
    command_interface_types_.push_back(params_.joints[i] + "/" +
                                       params_.pveg_joints_command_type[i]);
  }

  for (auto state_inter_ : params_.state_interfaces_name) {
    for (auto joint : params_.joints) {
      state_interface_types_.push_back(joint + "/" + state_inter_);
    }
  }

  arm_motors_viscous_friction_ = params_.arm_motors_viscous_friction;
  arm_motors_static_friction_ = params_.arm_motors_static_friction;
  arm_motors_K_tau_ = params_.arm_motors_K_tau;

  n_joints_ = params_.joints.size();

  // Resize the vectors to have the correct size, and filling them with
  // quiet_NaN to avoid any misbehaving Reference _interfaces' size is 3 value
  // for each joint eff, vel, pos
  //  n_joints_ * 2  * n_joints_ gains K the 2 comes from pos and vel

  reference_interfaces_.resize(3 * n_joints_ + n_joints_ * 2 * n_joints_,
                               std::numeric_limits<double>::quiet_NaN());

  // same for the current state
  current_state_.position.resize(n_joints_);
  current_state_.velocity.resize(n_joints_);

  ricatti_command_.u_command.resize(n_joints_);
  ricatti_command_.x_command.resize(2 * n_joints_);
  ricatti_command_.K_command.resize(n_joints_, 2 * n_joints_);

  measuredX_.resize(2 * n_joints_);

  corrected_eff_command_.resize(n_joints_);
  command_.resize(n_joints_);

  RCLCPP_INFO(get_node()->get_logger(),
              "motors parameters loaded successfully");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
cpcc2_tiago::PvegChainedController::on_init() {
  // In the init we try do read the parameters
  try {
    declare_parameters();
  } catch (const std::exception &e) {
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
  // Exporting reference interfaces to Higher Level Controller
  // Note: Name (or prefix name) of the controller should be name of the
  // controller itself. In our case it's "cpcc2_tiago"
  for (int i = 0; i < n_joints_; i++) {
    reference_interfaces.push_back(hardware_interface::CommandInterface(
        get_node()->get_name(),
        params_.joints[i] + "/" + hardware_interface::HW_IF_EFFORT,
        &reference_interfaces_[i]));
  }
  for (int i = 0; i < n_joints_; i++) {
    reference_interfaces.push_back(hardware_interface::CommandInterface(
        get_node()->get_name(),
        params_.joints[i] + "/" + hardware_interface::HW_IF_POSITION,
        &reference_interfaces_[n_joints_ + i]));
  }
  for (int i = 0; i < n_joints_; i++) {
    reference_interfaces.push_back(hardware_interface::CommandInterface(
        get_node()->get_name(),
        params_.joints[i] + "/" + hardware_interface::HW_IF_VELOCITY,
        &reference_interfaces_[2 * n_joints_ + i]));
  }
  for (int i = 0; i < n_joints_; i++) { // all the gains
    for (int j = 0; j < 2 * n_joints_; j++) {
      reference_interfaces.push_back(hardware_interface::CommandInterface(
          get_node()->get_name(),
          params_.joints[i] + "/" + "gain" + std::to_string(i).c_str() + "_" +
              std::to_string(j).c_str(),
          &reference_interfaces_[3 * n_joints_ + i * 2 * n_joints_ + j]));
    }
  }

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
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  RCLCPP_INFO_ONCE(get_node()->get_logger(), "update_and_write_commands");
  return update() ? controller_interface::return_type::OK
                  : controller_interface::return_type::ERROR;
}

bool cpcc2_tiago::PvegChainedController::update() {
  // first we read the current state of the robot
  read_state_from_hardware();

  measuredX_ << current_state_.position, current_state_.velocity;

  // then gather the commands from the reference interface
  read_joints_commands();
  // compute the torque base on the command and the current state

  // correct for the actuators' friction
  // correct_efforts_for_friction();

  eff_command_ =
      ricatti_command_.u_command +
      ricatti_command_.K_command * (ricatti_command_.x_command - measuredX_);

  compute_command_from_type(eff_command_);

  set_command(command_);

  return true;
}

void PvegChainedController::read_joints_commands() {
  double command_u;
  double command_q;
  double command_v;
  double command_K;

  for (int i = 0; i < n_joints_; i++) {
    command_u = reference_interfaces_[i]; // arm_i_joint/effort
    // check if NaN, if nan set to current state to avoid large jump in torque
    ricatti_command_.u_command[i] = (command_u == command_u) ? command_u : 0;

    command_q = reference_interfaces_[n_joints_ + i]; // arm_i_joint/pos
    ricatti_command_.x_command[i] =
        (command_q == command_q) ? command_q : current_state_.position[i];

    command_v = reference_interfaces_[2 * n_joints_ + i]; // arm_i_joint/vel
    ricatti_command_.x_command[n_joints_ + i] =
        (command_v == command_v) ? command_v : current_state_.velocity[i];

    for (int j = 0; j < 2 * n_joints_; j++) {
      command_K = reference_interfaces_[3 * n_joints_ + i * 2 * n_joints_ + j];
      ricatti_command_.K_command(i, j) =
          (command_K == command_K) ? command_K : 0;
    }
  }
}

void PvegChainedController::read_state_from_hardware() {
  for (int i = 0; i < n_joints_; ++i) {
    current_state_.position[i] = state_interfaces_[i].get_value();
    current_state_.velocity[i] = state_interfaces_[n_joints_ + i].get_value();
  }
}

void PvegChainedController::correct_efforts_for_friction() {
  for (int i = 0; i < n_joints_; i++) {
    corrected_eff_command_[i] =
        ricatti_command_.u_command[i] +
        arm_motors_static_friction_[i] * sign(current_state_.velocity[i]) +
        arm_motors_viscous_friction_[i] * current_state_.velocity[i];
  }
}

void PvegChainedController::compute_command_from_type(
    Eigen::VectorXd eff_command) {
  for (int i = 0; i < n_joints_; i++) {
    if (params_.pveg_joints_command_type[i] == "effort") {
      command_[i] = eff_command[i];
    } else if (params_.pveg_joints_command_type[i] == "position") {
      command_[i] = ricatti_command_.x_command[i];
    } else if (params_.pveg_joints_command_type[i] == "velocity") {
      command_[i] = ricatti_command_.x_command[n_joints_ + i];
    }
  }
}

void PvegChainedController::set_command(Eigen::VectorXd command) {
  for (int i = 0; i < n_joints_; i++) {
    command_interfaces_[i].set_value(command[i]);
  }
}

} // namespace cpcc2_tiago

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(cpcc2_tiago::PvegChainedController,
                       controller_interface::ChainableControllerInterface)
