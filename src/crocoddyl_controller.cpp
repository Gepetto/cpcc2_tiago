#include "cpcc2_tiago/crocoddyl_controller.hpp"

namespace cpcc2_tiago {

void CrocoddylController::build_model() {
  const std::string urdf_filename = std::string(
      "/opt/openrobots/include/example-robot-data/robots/tiago_description/"
      "robots/tiago.urdf");

  // Load the urdf model
  Model full_model;
  pinocchio::urdf::buildModel(urdf_filename, full_model);

  std::vector<std::string> actuatedJointNames = {"universe"};

  actuatedJointNames.insert(actuatedJointNames.end(), params_.joints.begin(),
                            params_.joints.end());

  std::vector<std::string> allJointNames = full_model.names;

  // Create a list of joints to lock
  std::vector<std::string> jointsToLock;

  // Copy all elements from allJointNames that are not in actuatedJointNames
  // to jointsToLock

  std::copy_if(allJointNames.begin(), allJointNames.end(),
               std::back_inserter(jointsToLock),
               [&actuatedJointNames](const std::string& s) {
                 return std::find(actuatedJointNames.begin(),
                                  actuatedJointNames.end(),
                                  s) == actuatedJointNames.end();
               });

  for (auto s : actuatedJointNames) {
    std::cout << s << std::endl;
  }

  std::vector<FrameIndex> jointsToLockIDs = {};

  for (std::string jn : jointsToLock) {
    if (full_model.existJointName(jn)) {
      jointsToLockIDs.push_back(full_model.getJointId(jn));
    } else {
      std::cout << "Joint " << jn << " not found in the model" << std::endl;
    }
  };

  // Random configuration for the reduced model

  Eigen::VectorXd q_rand = randomConfiguration(full_model);

  model_ = buildReducedModel(full_model, jointsToLockIDs, q_rand);
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

  for (const auto& joint : params_.joints) {
    for (long unsigned int s_inter_id = 0;
         s_inter_id < params_.state_interfaces_name.size(); s_inter_id++) {
      state_interface_types_.push_back(
          joint + "/" + params_.state_interfaces_name[s_inter_id]);
    }
  }
  n_joints_ = params_.joints.size();

  // same for the current state
  current_state_.position.resize(n_joints_,
                                 std::numeric_limits<double>::quiet_NaN());
  current_state_.velocity.resize(n_joints_,
                                 std::numeric_limits<double>::quiet_NaN());
  current_state_.effort.resize(n_joints_,
                               std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(get_node()->get_logger(),
              "motors parameters loaded successfully");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CrocoddylController::on_init() {
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

  build_model();

  OCP_tiago_ = tiago_OCP::OCP(model_);

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(model_.nq + model_.nv);
  OCP_tiago_.setX0(x0);

  FrameIndex lh_id = model_.getJointId("hand_tool_joint");
  OCP_tiago_.setLhId(lh_id);

  OCP_tiago_.setTarget(hand_target_);
  RCLCPP_INFO(get_node()->get_logger(), "Set target to: %s",
              (std::to_string(OCP_tiago_.get_target()[0]) + std::string(" ") +
               std::to_string(OCP_tiago_.get_target()[1]) + std::string(" ") +
               std::to_string(OCP_tiago_.get_target()[2]))
                  .c_str());

  OCP_tiago_.setHorizonLength(20);
  OCP_tiago_.setTimeStep(5e-2);

  OCP_tiago_.buildCostsModel();
  OCP_tiago_.buildDiffActModel();
  OCP_tiago_.buildSolver();

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
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  auto current_t = rclcpp::Clock(RCL_ROS_TIME).now();
  auto diff =
      (current_t - prev_solve_time_).to_chrono<std::chrono::microseconds>();

  if (diff.count() + approx_solving_t_us_ > OCP_tiago_.get_time_step() * 1e6) {
    RCLCPP_INFO(get_node()->get_logger(), "C");
    prev_solve_time_ = rclcpp::Clock(RCL_ROS_TIME).now();

    read_state_from_hardware();

    Eigen::VectorXd measuredq = Eigen::Map<const Eigen::VectorXd>(
        current_state_.position.data(), current_state_.position.size());

    Eigen::VectorXd measuredv = Eigen::Map<const Eigen::VectorXd>(
        current_state_.velocity.data(), current_state_.velocity.size());

    Eigen::VectorXd measuredX(model_.nq + model_.nv);
    measuredX << measuredq, measuredv;

    OCP_tiago_.solve(measuredX);

    Eigen::VectorXd us = OCP_tiago_.get_us();
    Eigen::VectorXd xs = OCP_tiago_.get_xs();
    Eigen::MatrixXd gs = OCP_tiago_.get_gains();

    // std::cout << "gs: " << command_interfaces_.size() << std::endl;

    set_u_command(us);
    set_x_command(xs);
    set_K_command(gs);
  }
  return controller_interface::return_type::OK;
}

void CrocoddylController::read_state_from_hardware() {
  for (int i = 0; i < n_joints_; ++i) {
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
