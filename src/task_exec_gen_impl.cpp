#include "tesseract_robotraconteur/task_exec_gen_impl.h"

#include "tesseract_robotraconteur/conv/tasks_conv.h"

namespace rr_action=com::robotraconteur::action;
namespace RR = RobotRaconteur;

namespace tesseract_robotraconteur
{
    TaskExecGenImpl::TaskExecGenImpl(tesseract_planning::TaskComposerExecutor::Ptr executor,
                    tesseract_planning::TaskComposerNode::Ptr node,
                    tesseract_planning::TaskComposerProblem::Ptr problem,
                    tesseract_planning::TaskComposerDataStorage::Ptr data_storage)
        : AsyncTaskGenerator(RR::RobotRaconteurNode::sp()),
          executor(executor),
          node(node),
          problem(problem),
          data_storage(data_storage)
    {
    }

    void TaskExecGenImpl::Init()
    {
        
    }

    void TaskExecGenImpl::StartTask()
    {
        tesseract_planning::TaskComposerFuture::Ptr fut = executor->run(*node, problem, data_storage);
        weak_fut = fut;
        auto shared_this = RR_DYNAMIC_POINTER_CAST<TaskExecGenImpl>(shared_from_this());
        boost::thread([shared_this, fut]() {
            try
            {
                fut->wait();
                rr_tasks::TaskExecutorStatusPtr status(new rr_tasks::TaskExecutorStatus());
                if (fut->context->isAborted())
                {
                    throw RR::OperationAbortedException("Task aborted");
                }
                else if (fut->context->isSuccessful())
                {
                    status->action_status = rr_action::ActionStatusCode::complete;
                }
                else
                {
                    status->action_status = rr_action::ActionStatusCode::failed;
                }
                status->data_storage = conv::TaskComposerDataStorageToRR(fut->context->data_storage);
                shared_this->SetResult(status);
                return;
            }
            catch (std::exception& e)
            {
                auto rr_exp = RR::RobotRaconteurExceptionUtil::ExceptionToSharedPtr(e);
                shared_this->SetResultException(rr_exp);
                return;
            }
        });

    }

    void TaskExecGenImpl::CloseRequested()
    {
        tesseract_planning::TaskComposerFuture::Ptr fut = weak_fut.lock();
        if (!fut)
        {
            return;
        }
       
        // TODO: check if task is running
        fut->context->abort();
        
    }

    void TaskExecGenImpl::AbortRequested()
    {
        tesseract_planning::TaskComposerFuture::Ptr fut = weak_fut.lock();
        if (!fut)
        {
            return;
        }
        
        // TODO: check if task is running
        fut->context->abort();
        
    }

} // namespace tesseract_robotraconteur