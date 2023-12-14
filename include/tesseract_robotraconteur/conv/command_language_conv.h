/**
 * @file command_language_conv.h
 *
 * @author John Wason, PhD
 *
 * @copyright Copyright 2023 Wason Technology, LLC
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * @par
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TESSERACT_ROBOTRACONTEUR_COMMAND_LANGUAGE_CONV_H
#define TESSERACT_ROBOTRACONTEUR_COMMAND_LANGUAGE_CONV_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <RobotRaconteur.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/joint_state.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/move_instruction.h>

#include "robotraconteur_generated.h"

namespace rr_common = experimental::tesseract_robotics::common;
namespace rr_command = experimental::tesseract_robotics::command_language;

namespace tesseract_robotraconteur
{
namespace conv
{
rr_command::JointWaypointPtr JointWaypointToRR(const tesseract_planning::JointWaypointPoly& joint_waypoint);

tesseract_planning::JointWaypoint JointWaypointFromRR(const rr_command::JointWaypointPtr& joint_waypoint);

rr_command::CartesianWaypointPtr CartesianWaypointToRR(const tesseract_planning::CartesianWaypointPoly& cartesian_waypoint);

tesseract_planning::CartesianWaypoint CartesianWaypointFromRR(const rr_command::CartesianWaypointPtr& cartesian_waypoint);

rr_command::StateWaypointPtr StateWaypointToRR(const tesseract_planning::StateWaypointPoly& state_waypoint);

tesseract_planning::StateWaypoint StateWaypointFromRR(const rr_command::StateWaypointPtr& state_waypoint);

rr_command::MoveInstructionPtr MoveInstructionToRR(const tesseract_planning::MoveInstructionPoly& move_instruction);

tesseract_planning::MoveInstructionPoly MoveInstructionFromRR(const rr_command::MoveInstructionPtr& move_instruction);

rr_command::CompositeInstructionPtr CompositeInstructionToRR(const tesseract_planning::CompositeInstruction& composite_instruction);

tesseract_planning::CompositeInstruction CompositeInstructionFromRR(const rr_command::CompositeInstructionPtr& composite_instruction);

RobotRaconteur::RRValuePtr WaypointPolyToRR(const tesseract_planning::WaypointPoly& waypoint);

RobotRaconteur::RRValuePtr InstructionPolyToRR(const tesseract_planning::InstructionPoly& instruction);

tesseract_planning::InstructionPoly InstructionPolyFromRR(const RobotRaconteur::RRValuePtr& instruction);

tesseract_planning::WaypointPoly WaypointPolyFromRR(const RobotRaconteur::RRValuePtr& waypoint);

} // namespace conv
} // namespace tesseract_robotraconteur

#endif // TESSERACT_ROBOTRACONTEUR_COMMAND_LANGUAGE_CONV_H