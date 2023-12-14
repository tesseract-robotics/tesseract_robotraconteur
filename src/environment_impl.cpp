/**
 * @file environment_impl.cpp
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

#include "tesseract_robotraconteur/environment_impl.h"

#include "tesseract_robotraconteur/conv/environment_commands_conv.h"

// NOTE: tesseract_environment::Environment is thread safe. Locks are not needed here.

namespace RR=RobotRaconteur;
namespace rr_env_cmds = experimental::tesseract_robotics::environment::commands;

namespace tesseract_robotraconteur
{

    static void EnvironmentImpl_send_applied_commands(const RR::PipeBroadcasterPtr<rr_env_cmds::CommandsPtr>& broadcaster, const tesseract_environment::Event& event)
    {   
        if (!broadcaster)
        {
            return;
        }

        if (event.type != tesseract_environment::Events::COMMAND_APPLIED)
        {
            return;
        }

        try
            {
            auto command_applied_event = static_cast<const tesseract_environment::CommandAppliedEvent*>(&event);
                        
            auto rr_commands = environment_conv::CommandsToRR(command_applied_event->commands);

            broadcaster->AsyncSendPacket(rr_commands, []() {});
            }
            catch (const std::exception& err2)
            {
                ROBOTRACONTEUR_LOG_WARNING_COMPONENTNAME(RR::RobotRaconteurNode::weak_sp(), UserService,"tesseract", "environment", -1, "Error sending applied commands: " << err2.what());
            }

    }
    
    EnvironmentImpl::EnvironmentImpl(tesseract_environment::Environment::Ptr env, RR_SHARED_PTR<TesseractRoboticsImpl> parent)
        : env_(env), parent_(parent)
    {}

    void EnvironmentImpl::Init()
    {
        std::weak_ptr<EnvironmentImpl> weak_this = shared_from_this();
        env_->addEventCallback(123, [weak_this](const tesseract_environment::Event& event) {
            auto shared_this = weak_this.lock();
            if (!shared_this)
            {
                return;
            }

            // TODO: potential thread corruption here
            auto applied_broadcaster = shared_this->rrvar_applied_commands;
            EnvironmentImpl_send_applied_commands(applied_broadcaster, event);
        });        
    }

    std::string EnvironmentImpl::get_name()
    {
        return env_->getName();
    }

    RobotRaconteur::RRListPtr<RobotRaconteur::RRArray<char>> EnvironmentImpl::get_joint_names()
    {
        auto joint_names = env_->getJointNames();
        return RR::stringVectorToRRList(joint_names);
    }

    RobotRaconteur::RRListPtr<RobotRaconteur::RRArray<char>> EnvironmentImpl::get_link_names()
    {
        auto link_names = env_->getLinkNames();
        return RR::stringVectorToRRList(link_names);
    }

    rr_sg::JointPtr EnvironmentImpl::getf_joint(const std::string& joint_name)
    {
        return environment_conv::JointToRR(env_->getJoint(joint_name));
    }

    rr_sg::LinkPtr EnvironmentImpl::getf_link(const std::string& link_name)
    {
        return environment_conv::LinkToRR(env_->getLink(link_name));
    }

    int32_t EnvironmentImpl::apply_command(const RobotRaconteur::RRValuePtr& command)
    {
        auto cmd = environment_conv::CommandFromRR(command);
        return env_->applyCommand(cmd);
    }

    int32_t EnvironmentImpl::apply_commands(const rr_env::commands::CommandsPtr& commands)
    {
        auto cmd = environment_conv::CommandsFromRR(commands);
        return env_->applyCommands(cmd);
    }

    rr_env::commands::CommandsPtr EnvironmentImpl::getf_command_history()
    {
        auto commands = env_->getCommandHistory();
        return environment_conv::CommandsToRR(commands);
    }

    tesseract_environment::Environment::Ptr EnvironmentImpl::Environment() const { return env_; }
} // namespace tesseract_robotraconteur