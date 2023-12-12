#ifndef TESSERACT_ROBOTRACONTEUR_COMMON_CONV_H
#define TESSERACT_ROBOTRACONTEUR_COMMON_CONV_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <RobotRaconteur.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/joint_state.h>

#include "robotraconteur_generated.h"

namespace rr_common = experimental::tesseract_robotics::common;

namespace tesseract_robotraconteur
{
namespace conv
{
    rr_common::ManipulatorInfoPtr ManipulatorInfoToRR(const tesseract_common::ManipulatorInfo& manip_info);

    tesseract_common::ManipulatorInfo ManipulatorInfoFromRR(const rr_common::ManipulatorInfoPtr& manip_info);

    rr_common::JointStatePtr JointStateToRR(const tesseract_common::JointState& joint_state);

    tesseract_common::JointState JointStateFromRR(const rr_common::JointStatePtr& joint_state);

    rr_common::JointTrajectoryPtr JointTrajectoryToRR(const tesseract_common::JointTrajectory& joint_trajectory);

    tesseract_common::JointTrajectory JointTrajectoryFromRR(const rr_common::JointTrajectoryPtr& joint_trajectory);

} // namespace conv
} // namespace tesseract_robotraconteur


#endif // TESSERACT_ROBOTRACONTEUR_COMMON_CONV_H