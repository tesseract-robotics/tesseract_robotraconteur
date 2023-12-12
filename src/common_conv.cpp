#include "tesseract_robotraconteur/conv/common_conv.h"
#include "RobotRaconteurCompanion/Converters/EigenConverters.h"
#include <boost/range/algorithm.hpp>

namespace RR=RobotRaconteur;
namespace RRC_Eigen=RobotRaconteur::Companion::Converters::Eigen;

namespace tesseract_robotraconteur
{
namespace conv
{
    rr_common::ManipulatorInfoPtr ManipulatorInfoToRR(const tesseract_common::ManipulatorInfo& manip_info)
    {
        rr_common::ManipulatorInfoPtr ret(new rr_common::ManipulatorInfo());
        ret->manipulator = manip_info.manipulator;
        ret->working_frame = manip_info.working_frame;
        ret->tcp_frame = manip_info.tcp_frame;
        // TODO: ret->tcp_offset = ...
        ret->manipulator_ik_solver = manip_info.manipulator_ik_solver;
        return ret;
    }

    tesseract_common::ManipulatorInfo ManipulatorInfoFromRR(const rr_common::ManipulatorInfoPtr& manip_info)
    {
        RR_NULL_CHECK(manip_info);
        tesseract_common::ManipulatorInfo ret;
        ret.manipulator = manip_info->manipulator;
        ret.working_frame = manip_info->working_frame;
        ret.tcp_frame = manip_info->tcp_frame;
        // TODO: ret.tcp_offset = ...
        ret.manipulator_ik_solver = manip_info->manipulator_ik_solver;
        return ret;
    }

    rr_common::JointStatePtr JointStateToRR(const tesseract_common::JointState& joint_state)
    {
        rr_common::JointStatePtr ret(new rr_common::JointState());
        ret->joint_names = RR::stringVectorToRRList(joint_state.joint_names);
        ret->position = RRC_Eigen::EigenToRRArray<double>(joint_state.position);
        ret->velocity = RRC_Eigen::EigenToRRArray<double>(joint_state.velocity);
        ret->acceleration = RRC_Eigen::EigenToRRArray<double>(joint_state.acceleration);
        ret->effort = RRC_Eigen::EigenToRRArray<double>(joint_state.effort);
        ret->time = joint_state.time;
        return ret;
    }

    tesseract_common::JointState JointStateFromRR(const rr_common::JointStatePtr& joint_state)
    {
        RR_NULL_CHECK(joint_state);
        tesseract_common::JointState ret;
        ret.joint_names = RR::RRListToStringVector(joint_state->joint_names);
        ret.position = RRC_Eigen::RRArrayToEigen<Eigen::VectorXd>(joint_state->position);
        ret.velocity = RRC_Eigen::RRArrayToEigen<Eigen::VectorXd>(joint_state->velocity);
        ret.acceleration = RRC_Eigen::RRArrayToEigen<Eigen::VectorXd>(joint_state->acceleration);
        ret.effort = RRC_Eigen::RRArrayToEigen<Eigen::VectorXd>(joint_state->effort);
        ret.time = joint_state->time;
        return ret;
    }

    rr_common::JointTrajectoryPtr JointTrajectoryToRR(const tesseract_common::JointTrajectory& joint_trajectory)
    {
        rr_common::JointTrajectoryPtr ret(new rr_common::JointTrajectory());
        ret->states = RR::AllocateEmptyRRList<rr_common::JointState>();
        boost::range::transform(joint_trajectory.states, std::back_inserter(*ret->states), JointStateToRR);        
        ret->description = joint_trajectory.description;
        return ret;
    }

    tesseract_common::JointTrajectory JointTrajectoryFromRR(const rr_common::JointTrajectoryPtr& joint_trajectory)
    {
        tesseract_common::JointTrajectory ret;
        if (joint_trajectory->states)
        {
            boost::range::transform(*joint_trajectory->states, std::back_inserter(ret.states), JointStateFromRR);
        }           
        ret.description = joint_trajectory->description;
        return ret;
    }
} // namespace conv
} // namespace tesseract_robotraconteur