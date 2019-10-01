#include <RobotRaconteur.h>
#include "robotraconteur_generated.h"

#include <ros/ros.h>

#pragma once

namespace tesseract
{
    class Tesseract;
}

namespace tesseract_rosutils
{
    class ROSPlotting;
}

namespace tesseract_motion_planners
{
    class TrajOptFreespacePlannerConfig;
    class TrajOptFreespacePlanner;
}

namespace tesseract_robotraconteur
{
    namespace RR = RobotRaconteur;
    namespace planning = com::robotraconteur::robotics::planning;

    class TesseractPlannerImpl : public planning::Planner_default_impl, 
        public boost::enable_shared_from_this<TesseractPlannerImpl>
    {
    public:
        TesseractPlannerImpl(ros::NodeHandle nh);

        void Init(const std::string& ros_description_param_name = "robot_description", 
            const std::string& ros_description_semantic_param_name = "robot_description_semantic");
        
        virtual RR::GeneratorPtr<planning::PlanningResponsePtr,void> plan(planning::PlanningRequestPtr request);

    protected:

        std::shared_ptr<tesseract::Tesseract> tesseract_;
        ros::NodeHandle nh_;
        std::shared_ptr<tesseract_rosutils::ROSPlotting> plotter_;
    };

    using TesseractPlannerImplPtr = RR_SHARED_PTR<TesseractPlannerImpl>;

    class PlannerGenerator : public RR::SyncGenerator<planning::PlanningResponsePtr,void>
    {
    public:

        void InitPlanner(std::shared_ptr<tesseract::Tesseract> tesseract, 
            std::shared_ptr<tesseract_rosutils::ROSPlotting> plotter,
            planning::PlanningRequestPtr request
        );

        virtual planning::PlanningResponsePtr Next();

        virtual void Close();

        virtual void Abort();

    protected:

        std::shared_ptr<tesseract::Tesseract> tesseract_;        
        std::shared_ptr<tesseract_rosutils::ROSPlotting> plotter_;
        planning::PlanningRequestPtr request_;
        std::shared_ptr<tesseract_motion_planners::TrajOptFreespacePlannerConfig> planner_config_;
        std::shared_ptr<tesseract_motion_planners::TrajOptFreespacePlanner> planner_;
        boost::mutex this_lock;
        bool closed = false;
        bool aborted = false;
    };
}