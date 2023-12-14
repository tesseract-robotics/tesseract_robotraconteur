/**
 * @file tasks_conv.cpp
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

#include "tesseract_robotraconteur/conv/tasks_conv.h"
#include "tesseract_robotraconteur/conv/common_conv.h"
#include "tesseract_robotraconteur/conv/command_language_conv.h"

#define RR_TASKS_PREFIX "experimental.tesseract_robotics.tasks"
#define RR_COMMAND_LANG_PREFIX "experimental.tesseract_robotics.command_language"

namespace RR=RobotRaconteur;

namespace tesseract_robotraconteur
{
namespace conv
{
    tesseract_planning::TaskComposerProblem::Ptr TaskComposerProblemFromRR(const RR::RRValuePtr& problem, std::function<tesseract_environment::Environment::ConstPtr(const std::string&)> get_environment_fn)
    {
        RR_NULL_CHECK(problem);
        auto problem_type = problem->RRType();
        if (problem_type == RR_TASKS_PREFIX "." "TaskComposerProblem")
        {
            auto ret = std::make_shared<tesseract_planning::TaskComposerProblem>();
            auto rr_problem = RR::rr_cast<rr_tasks::TaskComposerProblem>(problem);
            ret->name = rr_problem->name;
            ret->input = TaskPolyFromRR(rr_problem->input);
            return ret;
        }
        else if (problem_type == RR_TASKS_PREFIX ".planning." "PlanningTaskComposerProblem")
        {
            return PlanningTaskComposerProblemFromRR(RR_DYNAMIC_POINTER_CAST<rr_tasks::planning::PlanningTaskComposerProblem>(problem), get_environment_fn);
        }
        else
        {
            throw RR::InvalidArgumentException("Unknown task composer problem type");
        }
    }

    tesseract_common::AnyPoly TaskPolyFromRR(const RR::RRValuePtr& data)
    {
        if (!data)
        {
            return tesseract_common::AnyPoly();
        }
        RR_NULL_CHECK(data);
        auto data_type = data->RRType();
        if (data_type == RR_COMMAND_LANG_PREFIX "." "CompositeInstruction")
        {
            auto instr = RR::rr_cast<rr_command::CompositeInstruction>(data);
            return tesseract_common::AnyPoly(conv::CompositeInstructionFromRR(instr));
        }
        else
        {
            throw RR::InvalidArgumentException("Unknown task poly type");        
        }

    }

    RR::RRValuePtr TaskPolyToRR(const tesseract_common::AnyPoly& data)
    {
        if (data.isNull())
        {
            return nullptr;
        }
        auto data_type_id = data.getType();
        
            if (data_type_id == typeid(tesseract_planning::CompositeInstruction))
            {
                auto instr = data.as<tesseract_planning::CompositeInstruction>();
                return conv::CompositeInstructionToRR(instr);
            }
            else
            {
                throw std::runtime_error("Unknown task poly type");
            }
        
    }

    tesseract_planning::PlanningTaskComposerProblem::Ptr PlanningTaskComposerProblemFromRR(const rr_tasks::planning::PlanningTaskComposerProblemPtr& rr_problem, std::function<tesseract_environment::Environment::ConstPtr(const std::string&)> get_environment_fn)
    {
        if (!rr_problem)
        {
            return std::make_shared<tesseract_planning::PlanningTaskComposerProblem>();
        }
        auto ret = std::make_shared<tesseract_planning::PlanningTaskComposerProblem>();
        ret->name = rr_problem->name;
        ret->input = std::move(TaskPolyFromRR(rr_problem->input));
        ret->env = get_environment_fn(rr_problem->environment_name);
        ret->manip_info = conv::ManipulatorInfoFromRR(rr_problem->manip_info);
        // TODO: profiles ...
        ret->profiles = std::make_shared<tesseract_planning::ProfileDictionary>();
        return ret;
    }

    RR::RRMapPtr<std::string,RR::RRValue> TaskComposerDataStorageToRR(const tesseract_planning::TaskComposerDataStorage::Ptr& data_storage)
    {
        auto ret = RR::AllocateEmptyRRMap<std::string,RR::RRValue>();
        for (const auto& pair : data_storage->getData())
        {
            ret->insert(std::make_pair(pair.first, TaskPolyToRR(pair.second)));
        }
        return ret;
    }

    tesseract_planning::TaskComposerDataStorage::Ptr TaskComposerDataStorageFromRR(const RR::RRMapPtr<std::string,RR::RRValue>& data_storage)
    {
        if (!data_storage)
        {
            return nullptr;
        }
        auto ret = std::make_shared<tesseract_planning::TaskComposerDataStorage>();
        for (const auto& pair : *data_storage)
        {
            ret->setData(pair.first, TaskPolyFromRR(pair.second));
        }
        return ret;
    }
} // namespace conv    
} // namespace tesseract_robotraconteur