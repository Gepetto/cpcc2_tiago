#include <cpcc2_tiago/model_builder.hpp>

// rclcpp
#include <rclcpp/logging.hpp>
#include <rclcpp/message_info.hpp>
#include <rclcpp/wait_set.hpp>
// msg
#include <std_msgs/msg/string.hpp>

namespace cpcc2_tiago::model_builder {

pin::Model build_model(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                       std::vector<std::string> joints) {
  // Load the urdf model
  if (!node) std::abort();

  static std_msgs::msg::String::SharedPtr robot_description{nullptr};
  if (!robot_description) {
    robot_description = std::make_shared<std_msgs::msg::String>();
    auto sub = node->create_subscription<std_msgs::msg::String>(
        "/robot_description", 1, [&node](const std_msgs::msg::String &) {
          RCLCPP_INFO(node->get_logger(), "echo from /robot_description");
        });

    RCLCPP_INFO(node->get_logger(),
                "Trying to get urdf from /robot_description");

    rclcpp::WaitSet wait_set;
    wait_set.add_subscription(sub);
    RCPPUTILS_SCOPE_EXIT(wait_set.remove_subscription(sub););
    using namespace std::chrono_literals;
    auto ret = wait_set.wait(10s);
    rclcpp::MessageInfo info;
    if (ret.kind() != rclcpp::WaitResultKind::Ready ||
        !sub->take(*robot_description, info)) {
      RCLCPP_ERROR(node->get_logger(),
                   "Could not get urdf from /robot_description");
      std::abort();
    }

    RCLCPP_INFO(node->get_logger(),
                "Successfully got urdf from /robot_description");
  } else
    RCLCPP_INFO(node->get_logger(), "Used urdf in cache");

  pin::Model full_model;
  pin::urdf::buildModelFromXML(robot_description->data, full_model);

  std::vector<std::string> actuatedJointNames = {"universe"};

  actuatedJointNames.insert(actuatedJointNames.end(), joints.begin(),
                            joints.end());

  std::vector<std::string> allJointNames = full_model.names;

  std::vector<std::string> jointsToLock;

  // Copy all elements from allJointNames that are not in actuatedJointNames
  // to jointsToLock

  std::copy_if(allJointNames.begin(), allJointNames.end(),
               std::back_inserter(jointsToLock),
               [&actuatedJointNames](const std::string &s) {
                 return std::find(actuatedJointNames.begin(),
                                  actuatedJointNames.end(),
                                  s) == actuatedJointNames.end();
               });

  std::cout << "Actuated joints: " << std::endl;

  for (auto s : actuatedJointNames) {
    std::cout << s << std::endl;
  }

  std::vector<pin::FrameIndex> jointsToLockIDs;

  for (std::string jn : jointsToLock) {
    if (full_model.existJointName(jn)) {
      jointsToLockIDs.push_back(full_model.getJointId(jn));
    } else {
      std::cout << "Joint " << jn << " not found in the model" << std::endl;
    }
  };

  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(full_model.nq);

  return buildReducedModel(full_model, jointsToLockIDs, q0);
}

void update_reduced_model(const Eigen::Ref<const Eigen::VectorXd> &x,
                          pin::Model &model, pin::Data &data) {
  // x is the reduced posture, or contains the reduced posture in the first
  // elements
  pin::forwardKinematics(model, data, x.head(model.nq));
  pin::updateFramePlacements(model, data);
}

pin::SE3 get_end_effector_SE3(pin::Data &data,
                              pin::FrameIndex &end_effector_id) {
  return data.oMf[end_effector_id];
}

}  // namespace cpcc2_tiago::model_builder
