#include <cpcc2_tiago/model_builder.hpp>

namespace cpcc2_tiago::model_builder {

pin::Model build_model(const std::string &urdf,
                       const std::vector<std::string> &joints) {
  // Load the urdf model
  pin::Model full_model;
  pin::urdf::buildModelFromXML(urdf, full_model);

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

  std::vector<pin::FrameIndex> jointsToLockIDs;

  for (std::string jn : jointsToLock)
    if (full_model.existJointName(jn))
      jointsToLockIDs.push_back(full_model.getJointId(jn));

  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(full_model.nq);

  return buildReducedModel(full_model, jointsToLockIDs, q0);
}

void update_reduced_model(const Eigen::Ref<const Eigen::VectorXd> &x,
                          const pin::Model &model, pin::Data &data) {
  // x is the reduced posture, or contains the reduced posture in the first
  // elements
  pin::forwardKinematics(model, data, x.head(model.nq));
  pin::updateFramePlacements(model, data);
}

pin::SE3 get_end_effector_SE3(const pin::Data &data,
                              pin::FrameIndex end_effector_id) {
  return data.oMf[end_effector_id];
}

}  // namespace cpcc2_tiago::model_builder
