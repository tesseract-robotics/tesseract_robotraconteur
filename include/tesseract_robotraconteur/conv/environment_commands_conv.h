#ifndef TESSERACT_ROBOTRACONTEUR_ENVIRONMENT_COMMANDS_CONV_H
#define TESSERACT_ROBOTRACONTEUR_ENVIRONMENT_COMMANDS_CONV_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <RobotRaconteur.h>
#include <RobotRaconteurCompanion/StdRobDef/StdRobDefAll.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "robotraconteur_generated.h"

#include <tesseract_environment/commands.h>
#include "scene_graph_conv.h"

namespace rr_env_cmd = experimental::tesseract_robotics::environment::commands;
namespace rr_sg = experimental::tesseract_robotics::scene_graph;

namespace tesseract_robotraconteur
{
namespace environment_conv
{
    // AddContactManagersPluginInfoCommand
    rr_env_cmd::AddContactManagersPluginInfoCommandPtr AddContactManagersPluginInfoCommandToRR(const tesseract_environment::AddContactManagersPluginInfoCommand::ConstPtr& cmd);

    tesseract_environment::AddContactManagersPluginInfoCommand::Ptr AddContactManagersPluginInfoCommandFromRR(const rr_env_cmd::AddContactManagersPluginInfoCommandPtr& cmd);

    // AddKinematicsInformationCommand
    rr_env_cmd::AddKinematicsInformationCommandPtr AddKinematicsInformationCommandToRR(const tesseract_environment::AddKinematicsInformationCommand::ConstPtr& cmd);

    tesseract_environment::AddKinematicsInformationCommand::Ptr AddKinematicsInformationCommandFromRR(const rr_env_cmd::AddKinematicsInformationCommandPtr& cmd);

    // AddLinkCommand
    rr_env_cmd::AddLinkCommandPtr AddLinkCommandToRR(const tesseract_environment::AddLinkCommand::ConstPtr& cmd);

    tesseract_environment::AddLinkCommand::Ptr AddLinkCommandFromRR(const rr_env_cmd::AddLinkCommandPtr& cmd);

    // AddSceneGraphCommand
    rr_env_cmd::AddSceneGraphCommandPtr AddSceneGraphCommandToRR(const tesseract_environment::AddSceneGraphCommand::ConstPtr& cmd);

    tesseract_environment::AddSceneGraphCommand::Ptr AddSceneGraphCommandFromRR(const rr_env_cmd::AddSceneGraphCommandPtr& cmd);

    // AddTrajectoryLinkCommand
    rr_env_cmd::AddTrajectoryLinkCommandPtr AddTrajectoryLinkCommandToRR(const tesseract_environment::AddTrajectoryLinkCommand::ConstPtr& cmd);

    tesseract_environment::AddTrajectoryLinkCommand::Ptr AddTrajectoryLinkCommandFromRR(const rr_env_cmd::AddTrajectoryLinkCommandPtr& cmd);

    // ChangeCollisionMarginsCommand
    rr_env_cmd::ChangeCollisionMarginsCommandPtr ChangeCollisionMarginsCommandToRR(const tesseract_environment::ChangeCollisionMarginsCommand::ConstPtr& cmd);

    tesseract_environment::ChangeCollisionMarginsCommand::Ptr ChangeCollisionMarginsCommandFromRR(const rr_env_cmd::ChangeCollisionMarginsCommandPtr& cmd);

    // ChangeJointAccelerationLimitsCommand
    rr_env_cmd::ChangeJointAccelerationLimitsCommandPtr ChangeJointAccelerationLimitsCommandToRR(const tesseract_environment::ChangeJointAccelerationLimitsCommand::ConstPtr& cmd);

    tesseract_environment::ChangeJointAccelerationLimitsCommand::Ptr ChangeJointAccelerationLimitsCommandFromRR(const rr_env_cmd::ChangeJointAccelerationLimitsCommandPtr& cmd);

    // ChangeJointOriginCommand
    rr_env_cmd::ChangeJointOriginCommandPtr ChangeJointOriginCommandToRR(const tesseract_environment::ChangeJointOriginCommand::ConstPtr& cmd);

    tesseract_environment::ChangeJointOriginCommand::Ptr ChangeJointOriginCommandFromRR(const rr_env_cmd::ChangeJointOriginCommandPtr& cmd);

    // ChangeJointPositionLimitsCommand
    rr_env_cmd::ChangeJointPositionLimitsCommandPtr ChangeJointPositionLimitsCommandToRR(const tesseract_environment::ChangeJointPositionLimitsCommand::ConstPtr& cmd);

    tesseract_environment::ChangeJointPositionLimitsCommand::Ptr ChangeJointPositionLimitsCommandFromRR(const rr_env_cmd::ChangeJointPositionLimitsCommandPtr& cmd);

    // ChangeJointVelocityLimitsCommand
    rr_env_cmd::ChangeJointVelocityLimitsCommandPtr ChangeJointVelocityLimitsCommandToRR(const tesseract_environment::ChangeJointVelocityLimitsCommand::ConstPtr& cmd);

    tesseract_environment::ChangeJointVelocityLimitsCommand::Ptr ChangeJointVelocityLimitsCommandFromRR(const rr_env_cmd::ChangeJointVelocityLimitsCommandPtr& cmd);

    // ChangeLinkCollisionEnabledCommand
    rr_env_cmd::ChangeLinkCollisionEnabledCommandPtr ChangeLinkCollisionEnabledCommandToRR(const tesseract_environment::ChangeLinkCollisionEnabledCommand::ConstPtr& cmd);

    tesseract_environment::ChangeLinkCollisionEnabledCommand::Ptr ChangeLinkCollisionEnabledCommandFromRR(const rr_env_cmd::ChangeLinkCollisionEnabledCommandPtr& cmd);

    // ChangeLinkOriginCommand
    rr_env_cmd::ChangeLinkOriginCommandPtr ChangeLinkOriginCommandToRR(const tesseract_environment::ChangeLinkOriginCommand::ConstPtr& cmd);

    tesseract_environment::ChangeLinkOriginCommand::Ptr ChangeLinkOriginCommandFromRR(const rr_env_cmd::ChangeLinkOriginCommandPtr& cmd);

    // ChangeLinkVisibilityCommand
    rr_env_cmd::ChangeLinkVisibilityCommandPtr ChangeLinkVisibilityCommandToRR(const tesseract_environment::ChangeLinkVisibilityCommand::ConstPtr& cmd);

    tesseract_environment::ChangeLinkVisibilityCommand::Ptr ChangeLinkVisibilityCommandFromRR(const rr_env_cmd::ChangeLinkVisibilityCommandPtr& cmd);

    // ModifyAllowedCollisionsCommand
    rr_env_cmd::ModifyAllowedCollisionsCommandPtr ModifyAllowedCollisionsCommandToRR(const tesseract_environment::ModifyAllowedCollisionsCommand::ConstPtr& cmd);

    tesseract_environment::ModifyAllowedCollisionsCommand::Ptr ModifyAllowedCollisionsCommandFromRR(const rr_env_cmd::ModifyAllowedCollisionsCommandPtr& cmd);

    // MoveJointCommand
    rr_env_cmd::MoveJointCommandPtr MoveJointCommandToRR(const tesseract_environment::MoveJointCommand::ConstPtr& cmd);

    tesseract_environment::MoveJointCommand::Ptr MoveJointCommandFromRR(const rr_env_cmd::MoveJointCommandPtr& cmd);

    // MoveLinkCommand
    rr_env_cmd::MoveLinkCommandPtr MoveLinkCommandToRR(const tesseract_environment::MoveLinkCommand::ConstPtr& cmd);

    tesseract_environment::MoveLinkCommand::Ptr MoveLinkCommandFromRR(const rr_env_cmd::MoveLinkCommandPtr& cmd);

    // RemoveAllowedCollisionLinkCommand
    rr_env_cmd::RemoveAllowedCollisionLinkCommandPtr RemoveAllowedCollisionLinkCommandToRR(const tesseract_environment::RemoveAllowedCollisionLinkCommand::ConstPtr& cmd);

    tesseract_environment::RemoveAllowedCollisionLinkCommand::Ptr RemoveAllowedCollisionLinkCommandFromRR(const rr_env_cmd::RemoveAllowedCollisionLinkCommandPtr& cmd);

    // RemoveJointCommand
    rr_env_cmd::RemoveJointCommandPtr RemoveJointCommandToRR(const tesseract_environment::RemoveJointCommand::ConstPtr& cmd);

    tesseract_environment::RemoveJointCommand::Ptr RemoveJointCommandFromRR(const rr_env_cmd::RemoveJointCommandPtr& cmd);

    // RemoveLinkCommand
    rr_env_cmd::RemoveLinkCommandPtr RemoveLinkCommandToRR(const tesseract_environment::RemoveLinkCommand::ConstPtr& cmd);

    tesseract_environment::RemoveLinkCommand::Ptr RemoveLinkCommandFromRR(const rr_env_cmd::RemoveLinkCommandPtr& cmd);

    // ReplaceJointCommand
    rr_env_cmd::ReplaceJointCommandPtr ReplaceJointCommandToRR(const tesseract_environment::ReplaceJointCommand::ConstPtr& cmd);

    tesseract_environment::ReplaceJointCommand::Ptr ReplaceJointCommandFromRR(const rr_env_cmd::ReplaceJointCommandPtr& cmd);

    // SetActiveContinuousContactManagerCommand
    rr_env_cmd::SetActiveContinuousContactManagerCommandPtr SetActiveContinuousContactManagerCommandToRR(const tesseract_environment::SetActiveContinuousContactManagerCommand::ConstPtr& cmd);

    tesseract_environment::SetActiveContinuousContactManagerCommand::Ptr SetActiveContinuousContactManagerCommandFromRR(const rr_env_cmd::SetActiveContinuousContactManagerCommandPtr& cmd);

    // SetActiveDiscreteContactManagerCommand
    rr_env_cmd::SetActiveDiscreteContactManagerCommandPtr SetActiveDiscreteContactManagerCommandToRR(const tesseract_environment::SetActiveDiscreteContactManagerCommand::ConstPtr& cmd);

    tesseract_environment::SetActiveDiscreteContactManagerCommand::Ptr SetActiveDiscreteContactManagerCommandFromRR(const rr_env_cmd::SetActiveDiscreteContactManagerCommandPtr& cmd);

    // Command
    RobotRaconteur::RRValuePtr CommandToRR(const tesseract_environment::Command::ConstPtr& cmd);

    tesseract_environment::Command::Ptr CommandFromRR(const RobotRaconteur::RRValuePtr& cmd);

    // Commands
    rr_env_cmd::CommandsPtr CommandsToRR(const tesseract_environment::Commands& commands);

    tesseract_environment::Commands CommandsFromRR(const rr_env_cmd::CommandsPtr& commands);

    


  
} // namespace environment_conv
} // namespace tesseract_robotraconteur

#endif // TESSERACT_ROBOTRACONTEUR_ENVIRONMENT_COMMANDS_CONV_H
