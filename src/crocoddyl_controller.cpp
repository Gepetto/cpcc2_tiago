#include "cpcc2_tiago/crocoddyl_controller.hpp"

namespace cpcc2_tiago {

Eigen::VectorXd CrocoddylController::interpolate_xs(Eigen::VectorXd x0,
                                                    Eigen::VectorXd x1,
                                                    double t) {
  Eigen::VectorXd x = Eigen::VectorXd::Zero(2 * n_joints_);
  x = x0 + (x1 - x0) * t / OCP_time_step_;
  return x;
}

void CrocoddylController::update_target_from_subscriber(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() != 3) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Target message has wrong size, should be 3");
    return;
  }
  Vector3d new_hand_target;
  new_hand_target << msg->data[0], msg->data[1], msg->data[2];
  OCP_tiago_.changeTarget(new_hand_target);
}

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

  for (auto state_inter_ : params_.state_interfaces_name) {
    for (auto joint : params_.joints) {
      state_interface_types_.push_back(joint + "/" + state_inter_);
    }
  }
  n_joints_ = params_.joints.size();

  // same for the current state
  current_state_.position.resize(n_joints_);
  current_state_.velocity.resize(n_joints_);
  current_state_.effort.resize(n_joints_);

  measuredX_.resize(2 * n_joints_);

  RCLCPP_INFO(get_node()->get_logger(),
              "motors parameters loaded successfully");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CrocoddylController::on_init() {
  RCLCPP_INFO(get_node()->get_logger(), "Initializing CrocoddylController.");
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

  if (n_joints_ == 0) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "List of joint names is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Build the model from the urdf
  model_ = model_builder::build_model(params_.joints);

  data_ = Data(model_);

  // create the OCP object
  OCP_tiago_ = tiago_OCP::OCP(model_, data_);

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(model_.nq + model_.nv);
  OCP_tiago_.setX0(x0);

  FrameIndex lh_id = model_.getFrameId("hand_tool_joint");
  OCP_tiago_.setLhId(lh_id);

  OCP_tiago_.setTarget(hand_target_);

  std::cout << "Set target to: " << hand_target_.transpose() << std::endl;

  OCP_horizon_length_ = 20;
  OCP_time_step_ = 5e-2;
  OCP_tiago_.setHorizonLength(OCP_horizon_length_);
  OCP_tiago_.setTimeStep(OCP_time_step_);

  OCP_tiago_.buildCostsModel();
  OCP_tiago_.buildDiffActModel();
  OCP_tiago_.buildSolver();

  OCP_tiago_.printCosts();

  command_subscriber_ =
      get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
          "~/target", 10,
          std::bind(&CrocoddylController::update_target_from_subscriber, this,
                    std::placeholders::_1));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CrocoddylController::command_interface_configuration() const {
  RCLCPP_INFO(get_node()->get_logger(),
              "Command Interface CrocoddylController");

  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  // Claiming Command interface exported as reference interface by
  // PvegContrller
  for (int i = 0; i < n_joints_; i++) {
    command_interfaces_config.names.push_back("pveg_chained_controller/" +
                                              params_.joints[i] + "/" +
                                              hardware_interface::HW_IF_EFFORT);
  }
  for (int i = 0; i < n_joints_; i++) {
    command_interfaces_config.names.push_back(
        "pveg_chained_controller/" + params_.joints[i] + "/" +
        hardware_interface::HW_IF_POSITION);
  }
  for (int i = 0; i < n_joints_; i++) {
    command_interfaces_config.names.push_back(
        "pveg_chained_controller/" + params_.joints[i] + "/" +
        hardware_interface::HW_IF_VELOCITY);
  }

  for (int i = 0; i < n_joints_; i++) {  // all the gains
    for (int j = 0; j < 2 * n_joints_; j++) {
      command_interfaces_config.names.push_back(
          "pveg_chained_controller/" + params_.joints[i] + "/" + "gain" +
          std::to_string(i).c_str() + "_" + std::to_string(j).c_str());
    }
  }
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
CrocoddylController::state_interface_configuration() const {
  RCLCPP_INFO(get_node()->get_logger(), "State Interface CrocoddylController.");

  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = state_interface_types_;

  state_interfaces_config.names = state_interface_types_;
  return state_interfaces_config;
}

controller_interface::return_type CrocoddylController::update(
    const rclcpp::Time & /*time*/
    ,
    const rclcpp::Duration & /*period*/) {
  start_update_time_ = rclcpp::Clock(RCL_ROS_TIME).now();
  update_frequency_ = 1 / ((start_update_time_ - prev_update_time_)
                               .to_chrono<std::chrono::microseconds>()
                               .count() *
                           1e-6);

  prev_update_time_ = start_update_time_;

  diff_ = (start_update_time_ - prev_solving_time_)
              .to_chrono<std::chrono::microseconds>();

  if ((diff_.count() + solving_time_) * 1e-6 > OCP_time_step_) {
    start_solving_time_ = start_update_time_;

    read_state_from_hardware();

    measuredX_ << current_state_.position, current_state_.velocity;

    OCP_tiago_.solve(measuredX_);

    us_ = OCP_tiago_.get_us();
    xs_ = OCP_tiago_.get_xs();
    gs_ = OCP_tiago_.get_gains();

    set_u_command(us_[0]);
    set_x_command(xs_[0]);
    set_K_command(gs_);

    end_solving_time_ = rclcpp::Clock(RCL_ROS_TIME).now();

    solving_time_ = (end_solving_time_ - start_solving_time_)
                        .to_chrono<std::chrono::microseconds>()
                        .count();

    if ((end_solving_time_ - prev_log_time_).seconds() >= 0.5) {
      // Log the current state
      std::cout << "Solving frequency: "
                << 1 / ((start_solving_time_ - prev_solving_time_)
                            .to_chrono<std::chrono::microseconds>()
                            .count() *
                        1e-6)
                << " Hz, solving time: " << solving_time_ << "us "
                << "Update frequency: " << update_frequency_ << " Hz"
                << std::endl;
      // Update the last log time
      prev_log_time_ = end_solving_time_;
    }

    prev_solving_time_ = start_solving_time_;

  } else {
    interpolate_t_ = (rclcpp::Clock(RCL_ROS_TIME).now() - prev_solving_time_)
                         .to_chrono<std::chrono::microseconds>()
                         .count() -
                     solving_time_;
    interpolated_xs_ = interpolate_xs(xs_[0], xs_[1], interpolate_t_ * 1e-6);
    set_x_command(interpolated_xs_);
  }

  return controller_interface::return_type::OK;
}

void CrocoddylController::read_state_from_hardware() {
  for (int i = 0; i < n_joints_; ++i) {
    current_state_.effort[i] = state_interfaces_[i].get_value();
    current_state_.position[i] = state_interfaces_[n_joints_ + i].get_value();
    current_state_.velocity[i] =
        state_interfaces_[2 * n_joints_ + i].get_value();
  }
}

void CrocoddylController::set_u_command(Eigen::VectorXd command_u) {
  for (int i = 0; i < n_joints_; i++) {
    command_interfaces_[i].set_value(command_u[i]);
  }
}
void CrocoddylController::set_x_command(Eigen::VectorXd command_x) {
  for (int i = 0; i < 2 * n_joints_; i++) {
    command_interfaces_[n_joints_ + i].set_value(command_x[i]);
    command_interfaces_[2 * n_joints_ + i].set_value(command_x[n_joints_ + i]);
  }
}
void CrocoddylController::set_K_command(Eigen::MatrixXd command_K) {
  for (int i = 0; i < n_joints_; ++i) {
    for (int j = 0; j < 2 * n_joints_; j++) {
      command_interfaces_[3 * n_joints_ + i * 2 * n_joints_ + j].set_value(
          command_K(i, j));
    }
  }
}
}  // namespace cpcc2_tiago

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(cpcc2_tiago::CrocoddylController,
                       controller_interface::ControllerInterface)
