#ifndef TESSERACT_ROBOTRAOUNTEUR_TASKS_IMPL_H
#define TESSERACT_ROBOTRAOUNTEUR_TASKS_IMPL_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <RobotRaconteur.h>
#include <RobotRaconteurCompanion/StdRobDef/StdRobDefAll.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "robotraconteur_generated.h"
#include <tesseract_task_composer/core/task_composer_executor.h>


namespace rr_tasks = experimental::tesseract_robotics::tasks;
namespace rr_planning = experimental::tesseract_robotics::tasks::planning;

namespace tesseract_robotraconteur
{
    class TesseractRoboticsImpl;
    class TaskExecutorImpl : public virtual rr_tasks::TaskExecutor_default_impl
    {
    public:
        TaskExecutorImpl(tesseract_planning::TaskComposerExecutor::Ptr executor, RR_SHARED_PTR<TesseractRoboticsImpl> parent);

        void Init();

        int32_t get_worker_count() override;

        int32_t get_task_count() override;

        RobotRaconteur::GeneratorPtr<rr_tasks::TaskExecutorStatusPtr, void> run(const rr_tasks::TaskExecutorInputPtr& input) override;

    protected:
        tesseract_planning::TaskComposerExecutor::Ptr executor_;
        RR_WEAK_PTR<TesseractRoboticsImpl> parent_;
    };
} // namespace tesseract_robotraconteur

#endif // TESSERACT_ROBOTRAOUNTEUR_TASKS_IMPL_H
