#include <RobotRaconteur.h>
#include <RobotRaconteurCompanion/StdRobDef/StdRobDefAll.h>
#include <tesseract_environment/core/environment.h>

#include <boost/range/algorithm.hpp>
#include <boost/range/algorithm_ext.hpp>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>

#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/trajopt/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/interface_utils.h>

#pragma once

namespace tesseract_robotraconteur
{
    namespace RR = RobotRaconteur;
    namespace planning = com::robotraconteur::robotics::planning;
    namespace identifier = com::robotraconteur::identifier;
    namespace geometry = com::robotraconteur::geometry;

    class TesseractPlannerImpl : public planning::Planner_default_impl, 
        public boost::enable_shared_from_this<TesseractPlannerImpl>
    {
    public:
        TesseractPlannerImpl();

        void Init(const std::string& urdf_xml_string, const std::string& srdf_xml_string,
            const tesseract_scene_graph::ResourceLocator::Ptr& locator);
        
        virtual RR::GeneratorPtr<planning::PlanningResponsePtr,void> plan(planning::PlanningRequestPtr request) override;

        virtual geometry::NamedPosePtr fwdkin(identifier::IdentifierPtr robot_identifier, RR::RRArrayPtr<double> joint_position, identifier::IdentifierPtr tip_link) override;

        virtual RR::RRListPtr<planning::InvKinResult> invkin(identifier::IdentifierPtr robot_identifier, geometry::NamedPosePtr tcp_pose, RR::RRArrayPtr<double> seed) override;

        virtual RR::RRListPtr<planning::ContactResult> compute_contacts(planning::PlannerJointPositionsPtr joint_position, planning::ContactTestTypeCode::ContactTestTypeCode contact_test_type, double contact_distance) override;

    protected:

        std::shared_ptr<tesseract_environment::Environment> tesseract_;
    };

    using TesseractPlannerImplPtr = RR_SHARED_PTR<TesseractPlannerImpl>;

    class PlannerGenerator : public RR::SyncGenerator<planning::PlanningResponsePtr,void>
    {
    public:

        void InitPlanner(std::shared_ptr<tesseract_environment::Environment> tesseract,            
            planning::PlanningRequestPtr request
        );

        virtual planning::PlanningResponsePtr Next();

        virtual void Close();

        virtual void Abort();

    protected:

        std::shared_ptr<tesseract_environment::Environment> tesseract_;        
        planning::PlanningRequestPtr rr_request_;
        std::shared_ptr<tesseract_planning::PlannerRequest> request_;
        std::shared_ptr<tesseract_planning::MotionPlanner> planner_;
        Eigen::VectorXd max_velocity_;
        Eigen::VectorXd max_acceleration_;
        double max_velocity_scaling_factor_ = 1.0;
        double max_acceleration_scaling_factor_ = 1.0;
        bool use_iterative_spline_ = false;
        boost::mutex this_lock;
        bool closed = false;
        bool aborted = false;
    };
}