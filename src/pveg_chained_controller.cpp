#include "cpcc2_tiago/pveg_chained_controller.hpp"

namespace cpcc2_tiago {

void PvegChainedController::init_shared_memory() {
  crocoddyl_shm_ = boost::interprocess::managed_shared_memory(
      boost::interprocess::open_only,
      "crocoddyl_shm"); // segment name

  start_sending_cmd_shm_ =
      crocoddyl_shm_.find<bool>("start_sending_cmd_shm").first;
  // Find the vector using the c-string name
}

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

  n_joints_ = params_.joints.size();

  for (int i = 0; i < n_joints_; i++) {
    command_interface_types_.push_back(params_.joints[i] + "/" +
                                       params_.pveg_joints_command_type[i]);
  }

  for (auto state_inter_ : params_.state_interfaces_name) {
    for (auto joint : params_.joints) {
      state_interface_types_.push_back(joint + "/" + state_inter_);
    }
  }

  reference_interfaces_.resize(
      n_joints_ + 2 * n_joints_ + n_joints_ * 2 * n_joints_ + 2 * n_joints_, 0);

  // same for the current state
  current_state_.position.resize(n_joints_);
  current_state_.velocity.resize(n_joints_);

  ricatti_command_.u_command.resize(n_joints_);
  ricatti_command_.u_command.setZero();

  ricatti_command_.x0_command.resize(2 * n_joints_);
  ricatti_command_.x0_command.setZero();

  ricatti_command_.xinter_command.resize(2 * n_joints_);
  ricatti_command_.xinter_command.setZero();

  ricatti_command_.x1_command.resize(2 * n_joints_);
  ricatti_command_.x1_command.setZero();

  ricatti_command_.K_command.resize(n_joints_, 2 * n_joints_);
  ricatti_command_.K_command.setZero();

  measuredX_.resize(2 * n_joints_);
  measuredX_.setZero();

  eff_command_.resize(n_joints_);
  eff_command_.setZero();

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

  while (true) {
    if (mutex_.try_lock()) {
      break;
    }

    if (!mutex_.timed_lock(boost::get_system_time() +
                           boost::posix_time::milliseconds(10))) {
      mutex_.unlock();
    }
  }

  init_shared_memory();

  model_ = model_builder::build_model(params_.joints);

  data_ = Data(model_);

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

  // x0
  for (int i = 0; i < n_joints_; i++) {
    reference_interfaces.push_back(hardware_interface::CommandInterface(
        get_node()->get_name(),
        params_.joints[i] + "/" + hardware_interface::HW_IF_POSITION + "_0",
        &reference_interfaces_[n_joints_ + i]));
  }
  for (int i = 0; i < n_joints_; i++) {
    reference_interfaces.push_back(hardware_interface::CommandInterface(
        get_node()->get_name(),
        params_.joints[i] + "/" + hardware_interface::HW_IF_VELOCITY + "_0",
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

  // x1
  for (int i = 0; i < n_joints_; i++) {
    reference_interfaces.push_back(hardware_interface::CommandInterface(
        get_node()->get_name(),
        params_.joints[i] + "/" + hardware_interface::HW_IF_POSITION + "_1",
        &reference_interfaces_[3 * n_joints_ + n_joints_ * 2 * n_joints_ +
                               +i]));
  }
  for (int i = 0; i < n_joints_; i++) {
    reference_interfaces.push_back(hardware_interface::CommandInterface(
        get_node()->get_name(),
        params_.joints[i] + "/" + hardware_interface::HW_IF_VELOCITY + "_1",
        &reference_interfaces_[3 * n_joints_ + n_joints_ * 2 * n_joints_ +
                               +n_joints_ + i]));
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

  if (start_sending_cmd_ == false) {
    mutex_.lock();
    start_sending_cmd_ = *start_sending_cmd_shm_;
    mutex_.unlock();
    return true;
  }

  read_state_from_hardware(current_state_);

  measuredX_ << current_state_.position, current_state_.velocity;

  model_builder::updateReducedModel(measuredX_, model_, data_);

  last_ricatti_command_ = ricatti_command_;

  read_joints_commands(ricatti_command_);
  ricatti_command_.xinter_command = ricatti_command_.x0_command;

  if (ricatti_command_ != last_ricatti_command_) {
    prev_command_time_ = rclcpp::Clock(RCL_ROS_TIME).now();
    // if the commands have changed we need to recompute the ricatti command

    interpolated_ricatti_command_ = ricatti_command_;

    // compute the ricatti command
    eff_command_ = compute_ricatti_command(ricatti_command_, measuredX_);

  } else {
    //    interpolate
    aba(model_, data_, measuredX_.head(model_.nq), measuredX_.tail(model_.nv),
        eff_command_); // compute ddq

    interpolate_t_ = (rclcpp::Clock(RCL_ROS_TIME).now() - prev_command_time_)
                         .to_chrono<std::chrono::nanoseconds>()
                         .count();

    interpolated_xs_ = tau_interpolate_xs(ricatti_command_.x0_command,
                                          data_.ddq, interpolate_t_ * 1e-9);

    interpolated_ricatti_command_.xinter_command = interpolated_xs_;

    eff_command_ =
        compute_ricatti_command(interpolated_ricatti_command_, measuredX_);
  }

  command_ = adapt_command_to_type(eff_command_, interpolated_ricatti_command_);

  set_command(command_);

  return true;
}

void PvegChainedController::read_joints_commands(ricatti_command &ric_cmd) {
  double command_u;
  double command_q0;
  double command_v0;
  double command_q1;
  double command_v1;
  double command_K;

  for (int i = 0; i < n_joints_; i++) {
    command_u = reference_interfaces_[i]; // arm_i_joint/effort
    // check if NaN, if nan set to current state to avoid large jump in torque
    ric_cmd.u_command[i] = (command_u == command_u) ? command_u : 0;

    command_q0 = reference_interfaces_[n_joints_ + i]; // arm_i_joint/pos0
    ric_cmd.x0_command[i] =
        (command_q0 == command_q0) ? command_q0 : current_state_.position[i];

    command_v0 = reference_interfaces_[2 * n_joints_ + i]; // arm_i_joint/vel0
    ric_cmd.x0_command[n_joints_ + i] =
        (command_v0 == command_v0) ? command_v0 : current_state_.velocity[i];

    for (int j = 0; j < 2 * n_joints_; j++) {
      command_K = reference_interfaces_[3 * n_joints_ + i * 2 * n_joints_ + j];
      ric_cmd.K_command(i, j) = (command_K == command_K) ? command_K : 0;
    }

    command_q1 =
        reference_interfaces_[3 * n_joints_ + n_joints_ * 2 * n_joints_ +
                              i]; // arm_i_joint/pos1
    ric_cmd.x1_command[i] =
        (command_q1 == command_q1) ? command_q1 : command_q0;

    command_v1 =
        reference_interfaces_[3 * n_joints_ + n_joints_ * 2 * n_joints_ +
                              n_joints_ + i];
    ric_cmd.x1_command[n_joints_ + i] =
        (command_v1 == command_v1) ? command_v1 : command_v0;
  }
}

void PvegChainedController::read_state_from_hardware(state &curr_state) {
  for (int i = 0; i < n_joints_; ++i) {
    curr_state.position[i] = state_interfaces_[i].get_value();
    curr_state.velocity[i] = state_interfaces_[n_joints_ + i].get_value();
  }
}

Eigen::VectorXd
PvegChainedController::compute_ricatti_command(ricatti_command ric_cmd,
                                               Eigen::VectorXd x) {
  return ric_cmd.u_command + ric_cmd.K_command * (ric_cmd.xinter_command - x);
}

Eigen::VectorXd PvegChainedController::tau_interpolate_xs(Eigen::VectorXd x0,
                                                          Eigen::VectorXd ddq,
                                                          double t) {
  Eigen::VectorXd q(model_.nq);
  Eigen::VectorXd v(model_.nv);
  Eigen::VectorXd x(model_.nq + model_.nv);

  v = x0.tail(model_.nv) + ddq * t;
  q = x0.head(model_.nq) + x0.tail(model_.nv) * t + 0.5 * ddq * t * t;

  x << q, v;

  return x;
}

Eigen::VectorXd
PvegChainedController::adapt_command_to_type(Eigen::VectorXd eff_command,
                                             ricatti_command ric_cmd) {
  Eigen::VectorXd command(n_joints_);
  for (int i = 0; i < n_joints_; i++) {
    if (params_.pveg_joints_command_type[i] == "effort") {
      command[i] = eff_command[i];
    } else if (params_.pveg_joints_command_type[i] == "velocity") {
      command[i] = ric_cmd.x1_command[n_joints_ + i];
    } else if (params_.pveg_joints_command_type[i] == "position") {
      command[i] = ric_cmd.x1_command[i];
    }
  }
  return command;
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
