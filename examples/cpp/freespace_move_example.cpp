#include <iostream>
#include "robotraconteur_generated.h"
#include <boost/uuid/uuid_generators.hpp>
#include <RobotRaconteurCompanion/Converters/EigenConverters.h>

namespace RR=RobotRaconteur;

namespace rr_planning = experimental::tesseract_robotics::tasks::planning;
namespace rr_tasks = experimental::tesseract_robotics::tasks;
namespace RRC_Eigen=RobotRaconteur::Companion::Converters::Eigen;
namespace rr_tesseract = experimental::tesseract_robotics;
namespace rr_common = experimental::tesseract_robotics::common;
namespace rr_command = experimental::tesseract_robotics::command_language;


// TODO: Add this function to the Robot Raconteur companion library
com::robotraconteur::uuid::UUID new_random_uuid()
{
    boost::uuids::uuid uuid = boost::uuids::random_generator()();
    com::robotraconteur::uuid::UUID ret;
    memcpy(ret.s.uuid_bytes.data(), &uuid, sizeof(uuid));
    return ret;
}

auto build_move_instr(const Eigen::Isometry3d& wp)
{
    rr_command::CartesianWaypointPtr wp_ptr(new rr_command::CartesianWaypoint());
    wp_ptr->transform = RRC_Eigen::ToTransform(wp);
    wp_ptr->lower_tolerance = RR::AllocateRRArray<double>(0);
    wp_ptr->upper_tolerance = RR::AllocateRRArray<double>(0);
    rr_command::MoveInstructionPtr move_instr(new rr_command::MoveInstruction());
    move_instr->waypoint = wp_ptr;
    move_instr->profile = "DEFAULT";
    move_instr->move_type = rr_command::MoveInstructionType::freespace;
    move_instr->uuid = new_random_uuid();
    return move_instr;
}

int main(int argc, char **argv) {

    RR::ClientNodeSetup node_setup(ROBOTRACONTEUR_SERVICE_TYPES, argc, argv);

    rr_tesseract::TesseractRoboticsPtr c = RR::rr_cast<rr_tesseract::TesseractRobotics>(RR::RobotRaconteurNode::s()->ConnectService("rr+tcp://localhost:63158?service=tesseract"));

    auto task_info = c->get_task_pipelines_info();
    std::string output_key = RR::RRArrayToString(task_info->at("FreespacePipeline")->output_keys->front());

    c->load_environment("default", "env");

    auto instr1 = build_move_instr(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.9, -0.3, 1.455) * Eigen::Quaterniond(0.70710678,0,0.70710678,0));
    auto instr2 = build_move_instr(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.9, 0.3, 1.455) * Eigen::Quaterniond(0.70710678,0,0.70710678,0));

    rr_common::ManipulatorInfoPtr manip_info(new rr_common::ManipulatorInfo());
    manip_info->manipulator = "manipulator";
    manip_info->tcp_frame = "tool0";
    manip_info->working_frame = "base_link";

    rr_command::CompositeInstructionPtr composite(new rr_command::CompositeInstruction());
    composite->order = rr_command::CompositeInstructionOrder::ordered;
    composite->instructions = RR::AllocateEmptyRRList<RR::RRValue>();
    composite->instructions->push_back(instr1);
    composite->instructions->push_back(instr2);
    composite->profile = "DEFAULT";
    composite->manipulator_info = manip_info;
    composite->uuid = new_random_uuid();

    rr_planning::PlanningTaskComposerProblemPtr problem(new rr_planning::PlanningTaskComposerProblem());
    problem->name = "example";
    problem->input = composite;
    problem->environment_name = "env";
    problem->manip_info = manip_info;

    auto executor = c->get_task_executors("default");

    rr_tasks::TaskExecutorInputPtr input(new rr_tasks::TaskExecutorInput());
    input->problem = problem;
    input->pipeline_name = "FreespacePipeline";

    auto exec_gen = executor->run(input);
    rr_tasks::TaskExecutorStatusPtr status;
    bool res;
    do
    {
        res = exec_gen->TryNext(status);
    }
    while (res);

    if (status->action_status != com::robotraconteur::action::ActionStatusCode::complete)
    {
        std::cerr << "Planning did not complete successfully" << std::endl;
        return 1;
    }

    auto output = RR::rr_cast<rr_command::CompositeInstruction>(status->data_storage->at(output_key));

    std::vector<Eigen::VectorXd> joint_trajectory;
    for (auto instr : *output->instructions)
    {
        auto move_instr = RR::rr_cast<rr_command::MoveInstruction>(instr);
        auto wp = RR::rr_cast<rr_command::StateWaypoint>(move_instr->waypoint);
        joint_trajectory.push_back(RRC_Eigen::RRArrayToEigen<Eigen::VectorXd>(wp->position));
    }

    // Print out the joint trajectory
    for (auto& j : joint_trajectory)
    {
        std::cout <<"[" << j.transpose() << "]" << std::endl;
    }

    // TODO: plot result in viewer!

    return 0;
}
