#include "tesseract_robotraconteur/tasks_impl.h"
#include "tesseract_robotraconteur/conv/tasks_conv.h"
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>

#include "tesseract_robotraconteur/task_exec_gen_impl.h"
#include "tesseract_robotraconteur/tesseract_robotics_impl.h"

namespace RR = RobotRaconteur;

namespace tesseract_robotraconteur
{
    TaskExecutorImpl::TaskExecutorImpl(tesseract_planning::TaskComposerExecutor::Ptr executor, RR_SHARED_PTR<TesseractRoboticsImpl> parent)
        : executor_(executor), parent_(parent)
    {
        
    }

    void TaskExecutorImpl::Init()
    {
        // Nothing to do here for now
    }

    int32_t TaskExecutorImpl::get_worker_count()
    {
        return executor_->getWorkerCount();
    }

    int32_t TaskExecutorImpl::get_task_count()
    {
        return executor_->getTaskCount();
    }

    RR::GeneratorPtr<rr_tasks::TaskExecutorStatusPtr, void> TaskExecutorImpl::run(const rr_tasks::TaskExecutorInputPtr& input)
    {
        RR_NULL_CHECK(input);
        auto pipeline_name = input->pipeline_name;
        RR_SHARED_PTR<TesseractRoboticsImpl> parent = parent_.lock();
        if (!parent)
        {
            throw RR::InvalidOperationException("Parent has been released");
        }
        tesseract_planning::TaskComposerProblem::Ptr problem = conv::TaskComposerProblemFromRR(input->problem, [parent](const std::string& name) { return parent->GetEnvironmentImpl(name)->Environment(); });
        tesseract_planning::TaskComposerDataStorage::Ptr data_storage = conv::TaskComposerDataStorageFromRR(input->data_storage);
        if (!data_storage)
        {
            data_storage = std::make_shared<tesseract_planning::TaskComposerDataStorage>();
        }

        auto plugin_factory = parent->GetExecutorFactory();

        tesseract_planning::TaskComposerNode::Ptr node = plugin_factory->createTaskComposerNode(pipeline_name);

        return RR_MAKE_SHARED<TaskExecGenImpl>(executor_, node, problem, data_storage);
            
    }
} // namespace tesseract_robotraconteur