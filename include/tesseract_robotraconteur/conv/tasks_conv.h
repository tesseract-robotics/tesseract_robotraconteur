/**
 * @file tasks_conv.h
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

#ifndef TESSERACT_ROBOTRACONTEUR_CONV_TASKS_CONV_H
#define TESSERACT_ROBOTRACONTEUR_CONV_TASKS_CONV_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <RobotRaconteur.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_task_composer/core/task_composer_problem.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/planning/planning_task_composer_problem.h>
#include <tesseract_environment/commands.h>

#include "robotraconteur_generated.h"

namespace rr_tasks = experimental::tesseract_robotics::tasks;

namespace tesseract_robotraconteur
{
namespace conv
{
    tesseract_planning::TaskComposerProblem::Ptr TaskComposerProblemFromRR(const RobotRaconteur::RRValuePtr& problem, std::function<tesseract_environment::Environment::ConstPtr(const std::string&)> get_environment_fn);

    tesseract_planning::PlanningTaskComposerProblem::Ptr PlanningTaskComposerProblemFromRR(const rr_tasks::planning::PlanningTaskComposerProblemPtr& problem, std::function<tesseract_environment::Environment::ConstPtr(const std::string&)> get_environment_fn);

    tesseract_common::AnyPoly TaskPolyFromRR(const RobotRaconteur::RRValuePtr& data);

    RobotRaconteur::RRValuePtr TaskPolyToRR(const tesseract_common::AnyPoly& data);

    RobotRaconteur::RRMapPtr<std::string,RobotRaconteur::RRValue> TaskComposerDataStorageToRR(const tesseract_planning::TaskComposerDataStorage::Ptr& data_storage);

    tesseract_planning::TaskComposerDataStorage::Ptr TaskComposerDataStorageFromRR(const RobotRaconteur::RRMapPtr<std::string,RobotRaconteur::RRValue>& data_storage);
} // namespace conv
} // namespace tesseract_robotraconteur

#endif // TESSERACT_ROBOTRACONTEUR_CONV_TASKS_CONV_H
