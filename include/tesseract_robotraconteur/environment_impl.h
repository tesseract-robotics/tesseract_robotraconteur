#ifndef TESSERACT_ROBOTRACONTEUR_ENVIRONMENT_IMPL_H
#define TESSERACT_ROBOTRACONTEUR_ENVIRONMENT_IMPL_H

#include <tesseract_environment/environment.h>

#include "robotraconteur_generated.h"

namespace rr_env = experimental::tesseract_robotics::environment;
namespace rr_sg = experimental::tesseract_robotics::scene_graph;

namespace tesseract_robotraconteur
{
    class TesseractRoboticsImpl;
    class EnvironmentImpl : public virtual rr_env::Environment_default_impl, public std::enable_shared_from_this<EnvironmentImpl>
    {
        public:

        EnvironmentImpl(tesseract_environment::Environment::Ptr env, RR_SHARED_PTR<TesseractRoboticsImpl> parent);

        void Init();

        std::string get_name() override;

        RobotRaconteur::RRListPtr<RobotRaconteur::RRArray<char>> get_joint_names() override;

        RobotRaconteur::RRListPtr<RobotRaconteur::RRArray<char>> get_link_names() override;

        rr_sg::JointPtr getf_joint(const std::string& joint_name) override;

        rr_sg::LinkPtr getf_link(const std::string& link_name) override;
        

        int32_t apply_command(const RobotRaconteur::RRValuePtr& command) override;

        int32_t apply_commands(const rr_env::commands::CommandsPtr& commands) override;

        rr_env::commands::CommandsPtr getf_command_history() override;

        tesseract_environment::Environment::Ptr Environment() const;

        protected:
        tesseract_environment::Environment::Ptr env_;
        RR_WEAK_PTR<TesseractRoboticsImpl> parent_;
    };
} // namespace tesseract_robotraconteur

#endif // TESSERACT_ROBOTRACONTEUR_ENVIRONMENT_IMPL_H
