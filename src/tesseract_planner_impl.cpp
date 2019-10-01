#include "tesseract_planner_impl.h"

#include <tesseract/tesseract.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_rosutils/conversions.h>
#include <tesseract_rosutils/plotting.h>
#include <trajopt/problem_description.hpp>
#include <tesseract_motion_planners/trajopt/trajopt_freespace_planner.h>
#include <boost/range/algorithm.hpp>
#include <boost/range/algorithm_ext.hpp>

using namespace trajopt;
using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;
using namespace tesseract_motion_planners;
namespace geometry = com::robotraconteur::geometry;
namespace trajectory = com::robotraconteur::robotics::trajectory;

namespace tesseract_robotraconteur
{
    TesseractPlannerImpl::TesseractPlannerImpl(ros::NodeHandle nh) : nh_(nh), 
        tesseract_(std::make_shared<tesseract::Tesseract>())
    {
        
    }

    void TesseractPlannerImpl::Init(const std::string& ros_description_param_name, const std::string& ros_description_semantic_param_name)
    {
        std::string urdf_xml_string;
        std::string srdf_xml_string;

        nh_.getParam(ros_description_param_name, urdf_xml_string);
        nh_.getParam(ros_description_semantic_param_name, srdf_xml_string);

        ResourceLocatorFn locator = tesseract_rosutils::locateResource;
        if (!tesseract_->init(urdf_xml_string, srdf_xml_string, locator))
            throw RR::OperationFailedException("Tesseract initialization failed. Check urdf and srdf files.");

        plotter_ = std::make_shared<ROSPlotting>(tesseract_->getEnvironment());
    }

    RR::GeneratorPtr<planning::PlanningResponsePtr,void> TesseractPlannerImpl::plan(planning::PlanningRequestPtr request)
    {
        auto gen = RR_MAKE_SHARED<PlannerGenerator>();
        gen->InitPlanner(tesseract_, plotter_, request);
        return gen; 
    }

    static Eigen::Isometry3d rr_pose_to_eigen(const geometry::Pose& rr_pose)
    {
        Eigen::Quaterniond q(rr_pose.s.orientation.s.w,rr_pose.s.orientation.s.x,rr_pose.s.orientation.s.y,rr_pose.s.orientation.s.z);
        Eigen::Vector3d p(rr_pose.s.position.s.x, rr_pose.s.position.s.y, rr_pose.s.position.s.z);

        Eigen::Isometry3d t;
        t.fromPositionOrientationScale(p,q,Eigen::Vector3d::Ones());
        return t;
    }

    static Eigen::VectorXd rr_array_to_eigen(RR::RRArrayPtr<double> rr_array)
    {
        Eigen::VectorXd ret = Eigen::Map<Eigen::VectorXd>(&rr_array->at(0),rr_array->size());
        return ret;
    }

    static Waypoint::Ptr rr_waypoint_to_tesseract(RR::RRValuePtr rr_waypoint, std::vector<std::string> joint_names)
    {        
        planning::JointWaypointPtr rr_joint_waypoint = RR_DYNAMIC_POINTER_CAST<planning::JointWaypoint>(rr_waypoint);
        if (rr_joint_waypoint)
        {
            auto joint_waypoint = std::make_shared<JointWaypoint>(rr_array_to_eigen(rr_joint_waypoint->joint_positions), joint_names);
            joint_waypoint->setIsCritical((bool)rr_joint_waypoint->is_critical.value);
            joint_waypoint->setCoefficients(rr_array_to_eigen(rr_joint_waypoint->coeffs));
            return joint_waypoint;
        }

        planning::JointTolerancedWaypointPtr rr_joint_toleranced_waypoint = RR_DYNAMIC_POINTER_CAST<planning::JointTolerancedWaypoint>(rr_waypoint);
        if (rr_joint_toleranced_waypoint)
        {
            auto joint_toleranced_waypoint = std::make_shared<JointTolerancedWaypoint>(rr_array_to_eigen(rr_joint_toleranced_waypoint->joint_positions), joint_names);
            joint_toleranced_waypoint->setIsCritical((bool)rr_joint_toleranced_waypoint->is_critical.value);
            joint_toleranced_waypoint->setCoefficients(rr_array_to_eigen(rr_joint_toleranced_waypoint->coeffs));
            joint_toleranced_waypoint->setLowerTolerance(rr_array_to_eigen(rr_joint_toleranced_waypoint->lower_tolerance));
            joint_toleranced_waypoint->setUpperTolerance(rr_array_to_eigen(rr_joint_toleranced_waypoint->upper_tolerance));
            return joint_toleranced_waypoint;
        }

        planning::CartesianWaypointPtr rr_cartesian_waypoint = RR_DYNAMIC_POINTER_CAST<planning::CartesianWaypoint>(rr_waypoint);
        if (rr_cartesian_waypoint)
        {
            auto cartesian_waypoint = std::make_shared<CartesianWaypoint>(rr_pose_to_eigen(rr_cartesian_waypoint->cartesion_position));
            cartesian_waypoint->setIsCritical((bool)rr_cartesian_waypoint->is_critical.value);
            cartesian_waypoint->setCoefficients(rr_array_to_eigen(rr_cartesian_waypoint->coeffs));
            return cartesian_waypoint;
        }

        throw RR::InvalidArgumentException("Invalid planning waypoint type argument. Expected JointWaypoint, JointTolerancedWaypoint, or CartesianWaypoint.");
    }

    void PlannerGenerator::InitPlanner(std::shared_ptr<tesseract::Tesseract> tesseract, 
            std::shared_ptr<tesseract_rosutils::ROSPlotting> plotter,
            planning::PlanningRequestPtr request
        )
    {
        if(!request)
        {
            throw RR::NullValueException("request must not be none");
        }

        tesseract_ = tesseract;
        plotter_ = plotter;
        request_ = request;

        std::string manipulator = request->device->name;
        auto kin = tesseract->getFwdKinematicsManager()->getFwdKinematicSolver(manipulator);
        if (!kin)
        {
            throw RR::InvalidArgumentException("Invalid device specified");
        }

        auto joint_names = kin->getJointNames();
        auto tip_link = kin->getTipLinkName();
        
        auto config = std::make_shared<TrajOptFreespacePlannerConfig>();

        config->link_ = tip_link;
        config->manipulator_ = manipulator;

        config->collision_safety_margin_ = request->collision_safety_margin;
        config->collision_check_ = request->collision_check.value;

        if (!request->start_waypoint) throw RR::NullValueException("start_waypoint must not be null");
        if (!request->goal_waypoint) throw RR::NullValueException("goal_waypoint must not be null");
        

        config->start_waypoint_ = rr_waypoint_to_tesseract(request->start_waypoint, joint_names);
        config->end_waypoint_ = rr_waypoint_to_tesseract(request->goal_waypoint, joint_names);

        if (request->planner_specific)
        {
            auto e = request->planner_specific->find("num_steps");
            if (e != request->planner_specific->end())
            {
                auto num_steps = RR_DYNAMIC_POINTER_CAST<RR::RRArray<double> >(e->second);
                if (!num_steps || num_steps->size() != 1) throw RR::InvalidArgumentException("num_steps must be an int32 scalar");
                config->num_steps_ = num_steps->at(0);
            }

            auto e2 = request->planner_specific->find("collision_continuous");
            if (e2 != request->planner_specific->end())
            {
                auto collision_continuous = RR_DYNAMIC_POINTER_CAST<RR::RRArray<RR::rr_bool> >(e->second);
                if (!collision_continuous || collision_continuous->size() != 1) throw RR::InvalidArgumentException("collision_continuous must be a bool scalar");
                config->collision_continuous_ = collision_continuous->at(0).value;
            }
        }

        config->tesseract_ = tesseract_;

        this->planner_config_ = config;

    }

    planning::PlanningResponsePtr PlannerGenerator::Next()
    {
        boost::mutex::scoped_lock lock(this_lock);

        if (aborted) throw RR::OperationAbortedException("Planning aborted");
        if (closed) throw RR::StopIterationException("Planning closed");        

        if (!planner_)
        {
            planner_ = std::make_shared<TrajOptFreespacePlanner>();
            RR_NULL_CHECK(planner_config_);
            planner_->setConfiguration(*planner_config_);

            PlannerResponse response;

            auto res = planner_->solve(response);
            if (!res)
            {
                throw new RR::OperationFailedException("Planning failed: " + res.message());
            }

            trajectory::JointTrajectoryPtr rr_trajectory(new trajectory::JointTrajectory());
            rr_trajectory->joint_names = RR::AllocateEmptyRRList<RR::RRArray<char> >();
            for (auto& s : response.joint_trajectory.joint_names)
            {
                rr_trajectory->joint_names->push_back(RR::stringToRRArray(s));
            }
            
            auto joint_units = RR::AllocateEmptyRRList<RR::RRArray<int32_t> >();
            for (size_t i=0; i<rr_trajectory->joint_names->size(); i++) 
                joint_units->push_back(RR::ScalarToRRArray<int32_t>(com::robotraconteur::robotics::joints::JointPositionUnits::implicit));

            rr_trajectory->joint_units = joint_units;
            rr_trajectory->waypoints = RR::AllocateEmptyRRList<trajectory::JointTrajectoryWaypoint>();

            auto& traj = response.joint_trajectory.trajectory;
            for (size_t i=0; i<traj.rows(); i++)
            {
                trajectory::JointTrajectoryWaypointPtr rr_waypoint(new trajectory::JointTrajectoryWaypoint());
                auto pos = RR::AllocateRRArray<double>(traj.cols());
                for (size_t j=0; j<traj.cols(); j++)
                {
                    pos->at(j) = traj(i,j);
                }
                rr_waypoint->joint_position = pos;
                rr_waypoint->joint_velocity = RR::AllocateEmptyRRArray<double>(traj.cols());
                rr_waypoint->position_tolerance = RR::AllocateEmptyRRArray<double>(traj.cols());
                rr_waypoint->velocity_tolerance = RR::AllocateEmptyRRArray<double>(traj.cols());
                rr_waypoint->interpolation_mode = trajectory::InterpolationMode::default_;
                rr_waypoint->time_from_start = (double)i;
                rr_trajectory->waypoints->push_back(rr_waypoint);
            }
            closed=true;
            planning::PlanningResponsePtr planner_res(new planning::PlanningResponse());
            planner_res->status_code = planning::PlannerStatusCode::success;
            planner_res->joint_trajectory=rr_trajectory;

            plotter_->plotTrajectory(response.joint_trajectory.joint_names,response.joint_trajectory.trajectory);

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