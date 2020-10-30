#include "tesseract_planner_impl.h"

#include "RobotRaconteurCompanion/Converters/EigenConverters.h"
#include "RobotRaconteurCompanion/Util/IdentifierUtil.h"
#include <tuple>
#include <tesseract_time_parameterization/iterative_spline_parameterization.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_interpolation_plan_profile.h>

using namespace trajopt;
using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_planning;
namespace geometry = com::robotraconteur::geometry;
namespace trajectory = com::robotraconteur::robotics::trajectory;
using namespace RobotRaconteur::Companion::Converters::Eigen;
using namespace RobotRaconteur::Companion::Util;


namespace tesseract_robotraconteur
{
    TesseractPlannerImpl::TesseractPlannerImpl() :
        tesseract_(std::make_shared<tesseract::Tesseract>())
    {
        
    }

    void TesseractPlannerImpl::Init(const std::string& urdf_xml_string, const std::string& srdf_xml_string, const tesseract_scene_graph::ResourceLocator::Ptr& locator)
    {        
        if (!tesseract_->init(urdf_xml_string, srdf_xml_string, locator))
            throw RR::OperationFailedException("Tesseract initialization failed. Check urdf and srdf files.");       
    }

    RR::GeneratorPtr<planning::PlanningResponsePtr,void> TesseractPlannerImpl::plan(planning::PlanningRequestPtr request)
    {
        auto gen = RR_MAKE_SHARED<PlannerGenerator>();
        gen->InitPlanner(tesseract_, request);
        return gen; 
    }

    geometry::NamedPosePtr TesseractPlannerImpl::fwdkin(identifier::IdentifierPtr robot_identifier, RR::RRArrayPtr<double> joint_position, identifier::IdentifierPtr tip_link)
    {
        if (!robot_identifier)
        {
            throw RR::InvalidArgumentException("Device must be specified!");
        }

        if (!IsIdentifierAnyUuid(robot_identifier))
        {
            throw RR::InvalidArgumentException("UUID not supported for device identifier");
        }

        std::string manipulator_name = robot_identifier->name;
        auto kin = tesseract_->getManipulatorManager()->getFwdKinematicSolver(manipulator_name);
        if (!kin)
        {
            throw RR::InvalidArgumentException("Invalid device specified");
        }

        auto joint_names = kin->getJointNames();
        auto tip_link_s = kin->getTipLinkName();
        auto base_link_s = kin->getBaseLinkName();

        if (tip_link)
        {
            if (!IsIdentifierAnyUuid(tip_link))
            {
                throw RR::InvalidArgumentException("UUID not supported for TCP identifier");
            }
            tip_link_s = tip_link->name;
        }

        Eigen::Isometry3d fwdkin_res;
        if(!kin->calcFwdKin(fwdkin_res, RRArrayToEigen<Eigen::VectorXd>(joint_position), tip_link_s))
        {
            throw RR::OperationFailedException("FwdKin failed");
        }

        geometry::NamedPosePtr res(new geometry::NamedPose());
        res->frame = CreateIdentifierFromName(tip_link_s);
        res->parent_frame = CreateIdentifierFromName(base_link_s);
        res->pose = ToPose(fwdkin_res);
        return res;        
    }

    RR::RRListPtr<planning::InvKinResult> TesseractPlannerImpl::invkin(identifier::IdentifierPtr robot_identifier, geometry::NamedPosePtr tcp_pose, RR::RRArrayPtr<double> seed)
    {
        if (!robot_identifier)
        {
            throw RR::InvalidArgumentException("Device must be specified!");
        }

        if (!IsIdentifierAnyUuid(robot_identifier))
        {
            throw RR::InvalidArgumentException("UUID not supported for device identifier");
        }

        std::string manipulator_name = robot_identifier->name;
        auto kin = tesseract_->getManipulatorManager()->getInvKinematicSolver(manipulator_name);
        if (!kin)
        {
            throw RR::InvalidArgumentException("Invalid device specified");
        }

        auto joint_names = kin->getJointNames();
        auto tip_link_s = kin->getTipLinkName();
        auto base_link_s = kin->getBaseLinkName();

        if (!tcp_pose)
        {
            throw RR::InvalidArgumentException("Target TCP pose must be specified!");
        }

        if (tcp_pose->parent_frame)
        {
            if (!IsIdentifierAnyUuid(tcp_pose->parent_frame))
            {
                throw RR::InvalidArgumentException("UUID not supported for frame identifier");
            }
            if (tcp_pose->parent_frame->name != base_link_s)
            {
                throw RR::InvalidArgumentException("Parent frame must match base link: " + base_link_s);
            }            
        }

        if (tcp_pose->frame)
        {
            if (!IsIdentifierAnyUuid(tcp_pose->frame))
            {
                throw RR::InvalidArgumentException("UUID not supported for TCP identifier");
            }
            tip_link_s = tcp_pose->frame->name;
        }

        Eigen::VectorXd solutions;

        if (!kin->calcInvKin(solutions, ToIsometry(tcp_pose->pose), RRArrayToEigen<Eigen::VectorXd>(seed)))
        {
            throw RR::OperationFailedException("InvKin failed");
        }

        size_t n_joints = joint_names.size();
        size_t n_results = solutions.size() / n_joints;

        auto res = RR::AllocateEmptyRRList<planning::InvKinResult>();
        for (size_t i = 0; i<n_results; i++)
        {
            auto res1 = RR::AllocateEmptyRRList<RR::RRArray<double> >();
            Eigen::VectorXd solution = solutions.segment(i*n_joints,n_joints);
            res1->push_back(EigenToRRArray(solution));
            planning::InvKinResultPtr res2(new planning::InvKinResult());
            res2->joints = res1;
            res->push_back(res2); 
        }
        return res;
    }



    static std::string rr_waypoint_get_profile(RR::RRMapPtr<std::string,RR::RRValue> waypoint_extended)
    {
        std::string profile = "DEFAULT";

        if (waypoint_extended)
        {
            auto e_profile = waypoint_extended->find("profile");
            if (e_profile !=waypoint_extended->end())
            {
                profile = RR::RRArrayToString(RR::rr_cast<RR::RRArray<char> >(e_profile->second));
            }
        }

        return profile;
    } 

    static PlanInstructionType rr_waypoint_get_instruction_type(planning::PlannerMotionTypeCode::PlannerMotionTypeCode rr_code)
    {
        PlanInstructionType instruction_type = PlanInstructionType::FREESPACE;
        switch (rr_code)
        {
            case planning::PlannerMotionTypeCode::default_:
            case planning::PlannerMotionTypeCode::freespace:
                break;
            case planning::PlannerMotionTypeCode::linear:
                instruction_type = PlanInstructionType::LINEAR;
                break;
            case planning::PlannerMotionTypeCode::cylindrical:
                instruction_type = PlanInstructionType::CIRCULAR;
                break;
            case planning::PlannerMotionTypeCode::start:
                instruction_type = PlanInstructionType::START;
                break;
            default:
                throw RR::InvalidArgumentException("Invalid motion type specified");
        }

        return instruction_type;
    }

    static std::tuple<PlanInstruction,planning::PlannerMotionTypeCode::PlannerMotionTypeCode> rr_waypoint_to_tesseract(RR::RRValuePtr rr_waypoint, std::vector<std::string> joint_names)
    {        
        planning::JointWaypointPtr rr_joint_waypoint = RR_DYNAMIC_POINTER_CAST<planning::JointWaypoint>(rr_waypoint);
        if (rr_joint_waypoint)
        {
            JointWaypoint joint_waypoint(joint_names, RRArrayToEigen<Eigen::VectorXd>(rr_joint_waypoint->joint_positions));
            
            std::string profile = rr_waypoint_get_profile(rr_joint_waypoint->extended);
            PlanInstructionType instruction_type = rr_waypoint_get_instruction_type(rr_joint_waypoint->motion_type);
            
            PlanInstruction plan_instruction(joint_waypoint, instruction_type, profile);
            
            return std::make_tuple(plan_instruction,rr_joint_waypoint->motion_type);
        }

        
        planning::CartesianWaypointPtr rr_cartesian_waypoint = RR_DYNAMIC_POINTER_CAST<planning::CartesianWaypoint>(rr_waypoint);
        if (rr_cartesian_waypoint)
        {    
            CartesianWaypoint cart_waypoint(ToIsometry(rr_cartesian_waypoint->position));

            std::string profile = rr_waypoint_get_profile(rr_cartesian_waypoint->extended);
            PlanInstructionType instruction_type = rr_waypoint_get_instruction_type(rr_cartesian_waypoint->motion_type);
            
            PlanInstruction plan_instruction(cart_waypoint, instruction_type, profile);
            
            return std::make_tuple(plan_instruction,rr_cartesian_waypoint->motion_type);
        }

        throw RR::InvalidArgumentException("Invalid planning waypoint type argument. Expected JointWaypoint or CartesianWaypoint.");
    }

    void PlannerGenerator::InitPlanner(std::shared_ptr<tesseract::Tesseract> tesseract, 
            planning::PlanningRequestPtr request
        )
    {
        if(!request)
        {
            throw RR::NullValueException("request must not be none");
        }

        tesseract_ = tesseract;
        rr_request_ = request;

        if (!request->device)
        {
            throw RR::InvalidArgumentException("Device must be specified!");
        }

        if (!IsIdentifierAnyUuid(request->device))
        {
            throw RR::InvalidArgumentException("UUID not supported for device identifier");
        }

        std::string manipulator_name = request->device->name;
        auto kin = tesseract->getManipulatorManager()->getFwdKinematicSolver(manipulator_name);
        if (!kin)
        {
            throw RR::InvalidArgumentException("Invalid device specified");
        }

        auto joint_names = kin->getJointNames();
        auto tip_link = kin->getTipLinkName();
        
        ManipulatorInfo manip;
        manip.manipulator = manipulator_name;
        manip.manipulator_ik_solver = "OPWInvKin";
        manip.working_frame = tesseract->getEnvironment()->getRootLinkName();
        manip.tcp = ToIsometry(request->tcp);

        size_t n_joints = joint_names.size();
        
        bool use_simple_planner = false;
        bool use_trajopt_planner = false;

        if (!request->planner_algorithm)
        {
            throw RR::InvalidArgumentException("Planner algorithm must be specified");
        }
        else
        {
            auto trajopt_identifier = CreateIdentifier("trajopt","d4b75ab6-7f3f-423d-a497-0d6678fcee12");
            auto simple_identifier = CreateIdentifier("simple","a777721c-6696-49f3-b4e9-db41250d8576");
            if (IsIdentifierMatch(simple_identifier, request->planner_algorithm))
            {
                use_simple_planner = true;
            }
            if (IsIdentifierMatch(trajopt_identifier, request->planner_algorithm))
            {
                use_trajopt_planner = true;
            }

            if (!use_simple_planner && !use_trajopt_planner)
            {
                throw RR::InvalidArgumentException("Unsupported planner algorithm");
            }

        }

        if (request->filter_algorithm)
        {
            auto time_parameterization_identifier = CreateIdentifier("iterative_spline","b4215679-665a-4aee-8e99-0a19a9db7f63");
            if (!IsIdentifierMatch(time_parameterization_identifier, request->filter_algorithm))
            {
                throw RR::InvalidArgumentException("Unsupported filter algorithm");
            }

            max_velocity_ = Eigen::VectorXd(n_joints);
            max_acceleration_ = Eigen::VectorXd(n_joints);
            for (size_t i = 0; i<n_joints; i++)
            {
                auto j_name = joint_names[i];
                auto j_limits = tesseract_->getEnvironment()->getJoint(j_name)->limits;
                if (!j_limits)
                {
                    max_velocity_[i] = 1e9;    
                    max_acceleration_[i] = 1e9;
                }
                else
                {
                    max_velocity_[i] = (j_limits->velocity != 0) ? j_limits->velocity : 1e9;
                    max_acceleration_[i] = (j_limits->acceleration != 0) ? j_limits->acceleration : 1e9;
                }
            }

            auto filter_extended = request->filter_specific;
            if (filter_extended)
            {
                auto max_velocity_e = filter_extended->find("max_velocity");
                if (max_velocity_e != filter_extended->end())
                {
                    auto max_velocity_req = RRArrayToEigen<Eigen::VectorXd>(RR::rr_cast<RR::RRArray<double> >(max_velocity_e->second));
                    if (max_velocity_req.size() != n_joints)
                    {
                        throw RR::InvalidArgumentException("max_velocity must have same number of elements as joints");
                    }
                    max_velocity_ = max_velocity_req;
                }

                auto max_acceleration_e = filter_extended->find("max_acceleration");
                if (max_acceleration_e != filter_extended->end())
                {
                    auto max_acceleration_req = RRArrayToEigen<Eigen::VectorXd>(RR::rr_cast<RR::RRArray<double> >(max_acceleration_e->second));
                    if (max_acceleration_req.size() != n_joints)
                    {
                        throw RR::InvalidArgumentException("max_acceleration must have same number of elements as joints");
                    }
                    max_acceleration_ = max_acceleration_req;
                }

                auto max_velocity_scaling_factor_e = filter_extended->find("max_velocity_scaling_factor");
                if (max_velocity_scaling_factor_e != filter_extended->end())
                {
                    auto max_velocity_scaling_factor_req = RR::RRArrayToScalar<double>(RR::rr_cast<RR::RRArray<double> >(max_velocity_scaling_factor_e->second));
                    if (max_velocity_scaling_factor_req <= 0)
                    {
                        throw RR::InvalidArgumentException("max_velocity_scaling_factor must be greater than zero");
                    }

                    max_velocity_scaling_factor_ = max_velocity_scaling_factor_req;
                }

                auto max_acceleration_scaling_factor_e = filter_extended->find("max_acceleration_scaling_factor");
                if (max_acceleration_scaling_factor_e != filter_extended->end())
                {
                    auto max_acceleration_scaling_factor_req = RR::RRArrayToScalar<double>(RR::rr_cast<RR::RRArray<double> >(max_acceleration_scaling_factor_e->second));
                    if (max_acceleration_scaling_factor_req <= 0)
                    {
                        throw RR::InvalidArgumentException("max_acceleration_scaling_factor must be greater than zero");
                    }

                    max_acceleration_scaling_factor_ = max_acceleration_scaling_factor_req;
                }
            }

            use_iterative_spline_ = true;
        }

        if (!request->waypoints)
        {
            throw RR::InvalidArgumentException("Waypoints in planning request must not be null");
        }

        if (request->waypoints->size() < 2)
        {
            throw RR::InvalidArgumentException("Request must contain at least two waypoints");
        }

        CompositeInstruction program("DEFAULT");
        for (auto e =request->waypoints->begin(); e != request->waypoints->end(); ++e)
        {
            auto& wp_i = *e; 
            auto instruction1 = rr_waypoint_to_tesseract(wp_i, joint_names);
            auto instruction = std::get<0>(instruction1);
            if (e == request->waypoints->begin())
            {
                switch(std::get<1>(instruction1))
                {
                    case planning::PlannerMotionTypeCode::default_:
                        instruction.setPlanType(PlanInstructionType::START);
                        break;
                    case planning::PlannerMotionTypeCode::start:
                        break;
                    default:
                        throw RR::InvalidArgumentException("First waypoint must be default or start motion type");
                }

                program.setStartInstruction(instruction);
            }
            else
            {
                program.push_back(instruction);
            }
        }

        program.setManipulatorInfo(manip);


        auto cur_state = tesseract->getEnvironment()->getCurrentState();

        if (use_simple_planner)
        {
            auto plan_profile = std::make_shared<SimplePlannerInterpolationPlanProfile>();
            plan_profile->setCartesianSteps(25);
            plan_profile->setFreespaceSteps(25);
            
            auto simple_planner = std::make_shared<SimpleMotionPlanner>();
            simple_planner->plan_profiles["DEFAULT"] = plan_profile;
            planner_ = simple_planner;
        }

        if (use_trajopt_planner)
        {
            auto plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
            auto composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();

            auto trajopt_planner = std::make_shared<TrajOptMotionPlanner>();
            trajopt_planner->plan_profiles["DEFAULT"] = plan_profile;
            trajopt_planner->composite_profiles["DEFAULT"] = composite_profile;
            trajopt_planner->problem_generator = &DefaultTrajoptProblemGenerator;
            planner_ = trajopt_planner;
        }

        CompositeInstruction seed = generateSeed(program, cur_state, tesseract,25,25);

        request_ = std::make_shared<PlannerRequest>();
        request_->seed = seed;
        request_->instructions = program;
        request_->tesseract = tesseract;
        request_->env_state = tesseract->getEnvironment()->getCurrentState(); 

    }

    planning::PlanningResponsePtr PlannerGenerator::Next()
    {
        boost::mutex::scoped_lock lock(this_lock);

        if (aborted) throw RR::OperationAbortedException("Planning aborted");
        if (closed) throw RR::StopIterationException("Planning closed");        

        if (request_)
        {
            
            auto request1 = request_;
            request_.reset();
            PlannerResponse response;

            auto res = planner_->solve(*request1,response);

            if (!res)
            {
                throw RR::OperationFailedException("Planning failed: " + res.message());
            }

            auto result_instructions = flatten(response.results);

            if (use_iterative_spline_)
            {
                tesseract_planning::IterativeSplineParameterization filter;
                if (!filter.compute(result_instructions,max_velocity_,max_acceleration_,
                    max_velocity_scaling_factor_,max_acceleration_scaling_factor_))
                {
                    throw RR::OperationFailedException("Iterative spline filter failed");
                }
            }

            for (auto instr1 : result_instructions)
            {
                auto& instr = instr1.get();
                
                if(instr.getType() != (int)InstructionType::MOVE_INSTRUCTION)
                {
                    throw RR::OperationFailedException("Unexpected instruction type returned from TrajOpt planner");
                }
                if(instr.cast<MoveInstruction>()->getWaypoint().getType() != (int)WaypointType::STATE_WAYPOINT)
                {
                    throw RR::OperationFailedException("Unexpected waypoint type returned from TrajOpt planner");
                }
            }



            trajectory::JointTrajectoryPtr rr_trajectory(new trajectory::JointTrajectory());
            rr_trajectory->joint_names = RR::AllocateEmptyRRList<RR::RRArray<char> >();

            

            for (auto& s : result_instructions.at(0).get().cast<MoveInstruction>()->getWaypoint().cast<StateWaypoint>()->joint_names)
            {
                rr_trajectory->joint_names->push_back(RR::stringToRRArray(s));
            }
            
            auto joint_units = RR::AllocateEmptyRRList<RR::RRArray<int32_t> >();
            for (size_t i=0; i<rr_trajectory->joint_names->size(); i++) 
                joint_units->push_back(RR::ScalarToRRArray<int32_t>(com::robotraconteur::robotics::joints::JointPositionUnits::implicit));

            rr_trajectory->joint_units = joint_units;
            rr_trajectory->waypoints = RR::AllocateEmptyRRList<trajectory::JointTrajectoryWaypoint>();

            for (size_t i=0; i<result_instructions.size(); i++)
            {
                auto& wp_i = *result_instructions.at(i).get().cast<MoveInstruction>()->getWaypoint().cast<StateWaypoint>();
                trajectory::JointTrajectoryWaypointPtr rr_waypoint(new trajectory::JointTrajectoryWaypoint());
                rr_waypoint->joint_position = EigenToRRArray<double>(wp_i.position);
                rr_waypoint->joint_velocity = EigenToRRArray<double>(wp_i.velocity);
                rr_waypoint->position_tolerance = RR::AllocateEmptyRRArray<double>(wp_i.position.size());
                rr_waypoint->velocity_tolerance = RR::AllocateEmptyRRArray<double>(wp_i.position.size());
                rr_waypoint->interpolation_mode = trajectory::InterpolationMode::default_;
                rr_waypoint->time_from_start = wp_i.time;
                rr_trajectory->waypoints->push_back(rr_waypoint);
            }
            closed=true;
            planning::PlanningResponsePtr planner_res(new planning::PlanningResponse());
            planner_res->status_code = planning::PlannerStatusCode::success;
            planner_res->joint_trajectory=rr_trajectory;

            return planner_res;
        }
        else
        {
            throw RR::StopIterationException("Planning complete");
        }
    }

    void PlannerGenerator::Close()
    {
        boost::mutex::scoped_lock lock(this_lock);
        if (!aborted)
            closed=true;
    }

    void PlannerGenerator::Abort()
    {
        boost::mutex::scoped_lock lock(this_lock);
        aborted=true;
    }
}