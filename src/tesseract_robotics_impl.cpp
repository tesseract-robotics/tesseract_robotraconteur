#include "tesseract_robotraconteur/tesseract_robotics_impl.h"

#include <tesseract_task_composer/core/task_composer_plugin_factory.h>

namespace RR = RobotRaconteur;

namespace tesseract_robotraconteur
{
    TesseractRoboticsImpl::TesseractRoboticsImpl(tesseract_environment::Environment::Ptr default_env, tesseract_planning::TaskComposerExecutor::Ptr default_executor,
        std::shared_ptr<tesseract_planning::TaskComposerPluginFactory> executor_factory)
        : default_env(default_env), default_executor(default_executor), executor_factory(executor_factory)
    {
       
    }

    void TesseractRoboticsImpl::Init()
    {
        auto rr_default_executor = RR_MAKE_SHARED<TaskExecutorImpl>(default_executor, shared_from_this());
        executors.insert(std::make_pair("default", rr_default_executor));
    }

    com::robotraconteur::device::DeviceInfoPtr TesseractRoboticsImpl::get_device_info()
    {
        return nullptr;
    }

    RR::RRMapPtr<std::string, rr_env::EnvironmentInfo> TesseractRoboticsImpl::get_environments_info()
    {
        auto ret = RR::AllocateEmptyRRMap<std::string, rr_env::EnvironmentInfo>();
        boost::mutex::scoped_lock lock(this_lock);
        for (const auto& pair : environments)
        {
            rr_env::EnvironmentInfoPtr info(new rr_env::EnvironmentInfo());
            info->name = pair.first;
            ret->insert(std::make_pair(pair.first, info));
        }
        return ret;
    }

    RR::RRMapPtr<std::string, rr_tasks::TaskPipelineInfo> TesseractRoboticsImpl::get_task_pipelines_info()
    {
        boost::mutex::scoped_lock lock(this_lock);
        auto ret = RR::AllocateEmptyRRMap<std::string, rr_tasks::TaskPipelineInfo>();
        auto plugin_info_map = executor_factory->getTaskComposerNodePlugins();
        for (const auto& pair : plugin_info_map)
        {
            rr_tasks::TaskPipelineInfoPtr ret1(new rr_tasks::TaskPipelineInfo());
            ret1->name = pair.first;
            tesseract_planning::TaskComposerNode::UPtr plugin = executor_factory->createTaskComposerNode(pair.first);
            ret1->input_keys = RR::stringVectorToRRList(plugin->getInputKeys());
            ret1->output_keys = RR::stringVectorToRRList(plugin->getOutputKeys());
            ret->insert(std::make_pair(pair.first, ret1));
        }
        return ret;
    }

    RR::RRMapPtr<std::string, rr_tasks::TaskExecutorInfo> TesseractRoboticsImpl::get_task_executors_info()
    {
        auto ret = RR::AllocateEmptyRRMap<std::string, rr_tasks::TaskExecutorInfo>();
        boost::mutex::scoped_lock lock(this_lock);
        for (const auto& pair : environments)
        {
            rr_tasks::TaskExecutorInfoPtr info(new rr_tasks::TaskExecutorInfo());
            info->name = pair.first;
            ret->insert(std::make_pair(pair.first, info));
        }
        return ret;
    }

    std::string TesseractRoboticsImpl::load_environment(const std::string& environment_resource_name, const std::string& new_environment_name)
    {
        boost::mutex::scoped_lock lock(this_lock);
        if (environment_resource_name != "default")
        {
            throw RR::InvalidArgumentException("Only default environment is supported");
        }
        tesseract_environment::Environment::Ptr new_env = default_env->clone();
        new_env->setName(new_environment_name);
        auto rr_env = RR_MAKE_SHARED<EnvironmentImpl>(new_env, shared_from_this());
        environments.insert(std::make_pair(new_environment_name, rr_env));
        return new_environment_name;
    }

    std::string TesseractRoboticsImpl::clone_environment(const std::string& environment_name, const std::string& new_environment_name)
    {
        boost::mutex::scoped_lock lock(this_lock);
        auto env = environments.find(environment_name);
        if (env == environments.end())
        {
            throw RR::InvalidArgumentException("Environment not found");
        }
        tesseract_environment::Environment::Ptr new_env = env->second->Environment()->clone();
        new_env->setName(new_environment_name);
        auto rr_env = RR_MAKE_SHARED<EnvironmentImpl>(new_env, shared_from_this());
        environments.insert(std::make_pair(new_environment_name, rr_env));
        return new_environment_name;
    }

    rr_env::EnvironmentPtr TesseractRoboticsImpl::get_environments(const std::string& ind)
    {
        return GetEnvironmentImpl(ind);
    }

    rr_tasks::TaskExecutorPtr TesseractRoboticsImpl::get_task_executors(const std::string& ind)
    {
        boost::mutex::scoped_lock lock(this_lock);
        auto executor = executors.find(ind);
        if (executor == executors.end())
        {
            throw RR::InvalidArgumentException("Executor not found");
        }
        return executor->second;
    }

    boost::shared_ptr<EnvironmentImpl> TesseractRoboticsImpl::GetEnvironmentImpl(const std::string &name)
    {
        boost::mutex::scoped_lock lock(this_lock);
        auto env = environments.find(name);
        if (env == environments.end())
        {
            throw RR::InvalidArgumentException("Environment not found");
        }
        return env->second;
    }

    std::shared_ptr<tesseract_planning::TaskComposerPluginFactory> TesseractRoboticsImpl::GetExecutorFactory()
    {
        return executor_factory;
    }

} // namespace tesseract_robotraconteur