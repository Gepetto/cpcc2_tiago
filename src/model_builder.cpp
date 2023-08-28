#include "cpcc2_tiago/model_builder.hpp"

namespace model_builder {

Model build_model(std::string urdf_path, std::vector<std::string> joints) {

  // Load the urdf model
  Model full_model;
  pinocchio::urdf::buildModel(urdf_path, full_model);

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

  std::vector<FrameIndex> jointsToLockIDs = {};

  for (std::string jn : jointsToLock) {
    if (full_model.existJointName(jn)) {
      jointsToLockIDs.push_back(full_model.getJointId(jn));
    } else {
      std::cout << "Joint " << jn << " not found in the model" << std::endl;
    }
  };

  // Random configuration for the reduced model

  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(full_model.nq);

  return buildReducedModel(full_model, jointsToLockIDs, q0);
}

void update_reduced_model(const Eigen::Ref<const Eigen::VectorXd> &x,
                          Model &model, Data &data) {
  /** x is the reduced posture, or contains the reduced posture in the first
   * elements */
  pinocchio::forwardKinematics(model, data, x.head(model.nq));
  pinocchio::updateFramePlacements(model, data);
}

SE3 get_end_effector_SE3(Data &data, FrameIndex &end_effector_id) {
  return data.oMf[end_effector_id];
}

} // namespace model_builder
