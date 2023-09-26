#include <cpcc2_tiago/crocoddyl_controller.hpp>

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

  for (auto state_inter_ : params_.state_interfaces_name) {
    for (auto joint : params_.joints) {
      state_interface_types_.push_back(joint + "/" + state_inter_);
    }
  }

  n_joints_ = params_.joints.size();
  joints_names_.reserve(n_joints_);

  joints_names_ = params_.joints;

  end_effector_ = params_.end_effector;

  x_meas_.resize(2 * n_joints_);
  xs0_.resize(2 * n_joints_);
  xs1_.resize(2 * n_joints_);
  us_.resize(n_joints_);
  Ks_.resize(n_joints_, 2 * n_joints_);

  // same for the current state
  current_state_ = State(n_joints_);

  real_effort_.resize(n_joints_);

  x_meas_.resize(2 * n_joints_);

  RCLCPP_INFO(get_node()->get_logger(),
              "motors parameters loaded successfully");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CrocoddylController::on_init() {
  RCLCPP_INFO(get_node()->get_logger(), "Initializing CrocoddylController.");

  // Read the parameters
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

  start_pub_ = get_node()->create_publisher<std_msgs::msg::Empty>(
      "/cpcc2_tiago_start", rclcpp::QoS{1}.reliable().transient_local());

  urdf_sub_ = get_node()->create_subscription<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS{1}.reliable().transient_local(),
      [this](const std_msgs::msg::String& msg) {
        RCLCPP_INFO(get_node()->get_logger(),
                    "Successfully received urdf from /robot_description");
        model_ = model_builder::build_model(msg.data, joints_names_);
        data_ = pin::Data(model_);
        lh_id_ = model_.getFrameId(this->end_effector_);
        pcs_.init_model(msg.data);
        this->urdf_sub_.reset();
      });

  // Initialize the topic to receive new targets
  target_subscriber_ =
      get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
          "~/target", 10,
          std::bind(&CrocoddylController::update_target_from_subscriber, this,
                    std::placeholders::_1));

  if (params_.enable_file_logging) {
    auto current_time = rclcpp::Clock(RCL_ROS_TIME).now();

    auto timestamp = std::chrono::nanoseconds(current_time.nanoseconds());

    std::time_t timestamp_sec =
        std::chrono::duration_cast<std::chrono::seconds>(timestamp).count();

    struct std::tm timeinfo;
    localtime_r(&timestamp_sec, &timeinfo);

    char buffer[80];
    strftime(buffer, sizeof(buffer), "%d-%m-%Y %H:%M:%S", &timeinfo);
    std::string formattedDate(buffer);

    // Initialize the rosbag writer
    writer_ = std::make_unique<rosbag2_cpp::Writer>();

    // Set the rosbag parameters
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = params_.log_folder + formattedDate + ".mcap";
    storage_options.storage_id = "mcap";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    writer_->open(storage_options, converter_options);

    writer_->create_topic({"/end_effect_pos_error",
                           "std_msgs/msg/Float64MultiArray",
                           rmw_get_serialization_format(), ""});

    writer_->create_topic({"/end_effect_pos", "std_msgs/msg/Float64MultiArray",
                           rmw_get_serialization_format(), ""});

    writer_->create_topic({"/effort_command", "std_msgs/msg/Float64MultiArray",
                           rmw_get_serialization_format(), ""});

    writer_->create_topic({"/real_effort", "std_msgs/msg/Float64MultiArray",
                           rmw_get_serialization_format(), ""});

    writer_->create_topic({"/x_meas", "std_msgs/msg/Float64MultiArray",
                           rmw_get_serialization_format(), ""});
  }

  if (params_.enable_live_logging) {
    // Initialize the publishers for real time
    // logging
    end_effect_pos_error_pub_ =
        get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
            "~/end_effect_pos_error", 10);

    end_effect_pos_pub_ =
        get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
            "~/end_effect_pos", 10);

    effort_command_pub_ =
        get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
            "~/effort_command", 10);

    real_effort_pub_ =
        get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
            "~/real_effort", 10);

    x_meas_pub_ =
        get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
            "~/x_meas", 10);
  }

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
  // PvegController

  // effort interface
  for (int i = 0; i < n_joints_; i++) {
    command_interfaces_config.names.push_back("pveg_chained_controller/" +
                                              joints_names_[i] + "/" +
                                              hardware_interface::HW_IF_EFFORT);
  }

  // position interfaces
  for (int i = 0; i < n_joints_; i++) {
    command_interfaces_config.names.push_back(
        "pveg_chained_controller/" + joints_names_[i] + "/" +
        hardware_interface::HW_IF_POSITION + "_0");
  }

  // velocity interfaces
  for (int i = 0; i < n_joints_; i++) {
    command_interfaces_config.names.push_back(
        "pveg_chained_controller/" + joints_names_[i] + "/" +
        hardware_interface::HW_IF_VELOCITY + "_0");
  }

  // gain interfaces
  for (int i = 0; i < n_joints_; i++) {
    for (int j = 0; j < 2 * n_joints_; j++) {
      command_interfaces_config.names.push_back(
          "pveg_chained_controller/" + joints_names_[i] + "/" + "gain" +
          std::to_string(i).c_str() + "_" + std::to_string(j).c_str());
    }
  }

  // next position interfaces
  for (int i = 0; i < n_joints_; i++) {
    command_interfaces_config.names.push_back(
        "pveg_chained_controller/" + joints_names_[i] + "/" +
        hardware_interface::HW_IF_POSITION + "_1");
  }

  // next velocity interfaces
  for (int i = 0; i < n_joints_; i++) {
    command_interfaces_config.names.push_back(
        "pveg_chained_controller/" + joints_names_[i] + "/" +
        hardware_interface::HW_IF_VELOCITY + "_1");
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
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
  if (urdf_sub_) return controller_interface::return_type::OK;

  // const rclcpp::Time time = rclcpp::Clock(RCL_ROS_TIME).now();
  pcs_.set_current_time(time.nanoseconds());

  current_state_ = read_state_from_hardware();

  real_effort_ = read_effort_from_hardware();

  x_meas_ << current_state_.position, current_state_.velocity;

  // for first solver iteration, send the measured state to the solver
  pcs_.set_x_meas(x_meas_);

  // update the model with the new state
  model_builder::update_reduced_model(x_meas_, model_, data_);

  // get the end effector position
  end_effector_pos_ =
      model_builder::get_end_effector_SE3(data_, lh_id_).translation();

  pos_error_ = end_effector_target_ - end_effector_pos_;

  // update the target
  if (is_first_update_) {
    is_first_update_ = false;
    // we first set the target to the current end effector pos to
    // set the balancing torque
    end_effector_target_ = end_effector_pos_;
    pcs_.set_target(end_effector_target_);
    pcs_.start_thread();
    start_pub_->publish(std_msgs::msg::Empty{});
    RCLCPP_INFO(get_node()->get_logger(), "Successfully did first update");
    return controller_interface::return_type::OK;
  }

  read_solver_results();

  set_u_command(us_);
  set_x0_command(xs0_);
  set_K_command(Ks_);
  set_x1_command(xs1_);

  // log the data to the ros2 bag and live topics
  log_msg_err_.data.assign(pos_error_.data(),
                           pos_error_.data() + pos_error_.size());

  log_msg_pos_.data.assign(end_effector_pos_.data(),
                           end_effector_pos_.data() + end_effector_pos_.size());

  log_msg_eff_.data.assign(us_.data(), us_.data() + us_.size());

  log_msg_real_eff_.data.assign(real_effort_.data(),
                                real_effort_.data() + real_effort_.size());

  log_msg_x_meas_.data.assign(x_meas_.data(), x_meas_.data() + x_meas_.size());

  if (params_.enable_file_logging) {
    writer_->write(log_msg_err_, "/end_effect_pos_error", time);
    writer_->write(log_msg_pos_, "/end_effect_pos", time);
    writer_->write(log_msg_eff_, "/effort_command", time);
    writer_->write(log_msg_real_eff_, "/real_effort", time);
    writer_->write(log_msg_x_meas_, "/x_meas", time);
  }

  if (params_.enable_live_logging) {
    end_effect_pos_error_pub_->publish(log_msg_err_);
    end_effect_pos_pub_->publish(log_msg_pos_);
    effort_command_pub_->publish(log_msg_eff_);
    real_effort_pub_->publish(log_msg_real_eff_);
    x_meas_pub_->publish(log_msg_x_meas_);
  }

  // print update frequency
  std::cout << "Controllers update freq: " << update_freq_vector_.mean()
            << " Hz           " << std::endl
            << "\x1b[A";

  // compute the update frequency
  const double update_freq = 1e9 / (time - last_update_time_).nanoseconds();
  update_freq_vector_.append(update_freq);
  last_update_time_ = time;

  return controller_interface::return_type::OK;
}

void CrocoddylController::update_target_from_subscriber(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() != 3) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Target message has wrong size, should be 3");
    return;
  }
  end_effector_target_ << msg->data[0], msg->data[1], msg->data[2];
  // write the target to the shared memory
  pcs_.set_target(end_effector_target_);
}

void CrocoddylController::read_solver_results() {
  // read the solver results from the shared memory
  auto [uus, uxs0, uxs1, uKs] = pcs_.get_results();
  us_ = uus;
  xs0_ = uxs0;
  xs1_ = uxs1;
  Ks_ = uKs;
}

State CrocoddylController::read_state_from_hardware() {
  State current_state(n_joints_);

  // read the state from the hardware
  // the state interfaces are as is :
  // /joint_name0/position
  // /joint_name1/position
  // ...
  // /joint_nameN/position

  // /joint_name0/velocity
  // /joint_name1/velocity
  // ...
  // /joint_nameN/velocity

  // /joint_name0/effort
  // /joint_name1/effort
  // ...
  // /joint_nameN/effort

  for (int i = 0; i < n_joints_; ++i) {
    current_state.position[i] = state_interfaces_[i].get_value();
    current_state.velocity[i] = state_interfaces_[n_joints_ + i].get_value();
  }

  return current_state;
}

Eigen::VectorXd CrocoddylController::read_effort_from_hardware() {
  Eigen::VectorXd effort(n_joints_);
  // read the effort from the hardware
  for (int i = 0; i < n_joints_; ++i) {
    effort[i] = state_interfaces_[2 * n_joints_ + i].get_value();
  }
  return effort;
}

void CrocoddylController::set_u_command(const Eigen::VectorXd& command_u) {
  // send the effort command to pveg controller
  for (int i = 0; i < n_joints_; i++) {
    command_interfaces_[i].set_value(command_u[i]);
  }
}

void CrocoddylController::set_x0_command(const Eigen::VectorXd& command_x) {
  // send the position and velocity command to pveg controller
  for (int i = 0; i < n_joints_; i++) {
    command_interfaces_[n_joints_ + i].set_value(command_x[i]);
    command_interfaces_[2 * n_joints_ + i].set_value(command_x[n_joints_ + i]);
  }
}

void CrocoddylController::set_K_command(const Eigen::MatrixXd& command_K) {
  // send the gain command to pveg controller
  for (int i = 0; i < n_joints_; ++i) {
    for (int j = 0; j < 2 * n_joints_; j++) {
      command_interfaces_[3 * n_joints_ + i * 2 * n_joints_ + j].set_value(
          command_K(i, j));
    }
  }
}

void CrocoddylController::set_x1_command(const Eigen::VectorXd& command_x) {
  // send the next position and velocity command to pveg controller
  for (int i = 0; i < n_joints_; i++) {
    command_interfaces_[3 * n_joints_ + n_joints_ * 2 * n_joints_ + i]
        .set_value(command_x[i]);

    command_interfaces_[3 * n_joints_ + n_joints_ * 2 * n_joints_ + n_joints_ +
                        i]
        .set_value(command_x[n_joints_ + i]);
  }
}

}  // namespace cpcc2_tiago

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cpcc2_tiago::CrocoddylController,
                       controller_interface::ControllerInterface)