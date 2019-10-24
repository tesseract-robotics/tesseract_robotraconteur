#include <RobotRaconteur.h>
#include "robotraconteur_generated.h"
#include <tesseract/tesseract.h>
#include <trajopt/problem_description.hpp>
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_freespace_config.h>
#include <boost/range/algorithm.hpp>
#include <boost/range/algorithm_ext.hpp>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>


#pragma once

namespace tesseract_robotraconteur
{
    namespace RR = RobotRaconteur;
    namespace planning = com::robotraconteur::robotics::planning;

    class TesseractPlannerImpl : public planning::Planner_default_impl, 
        public boost::enable_shared_from_this<TesseractPlannerImpl>
    {
    public:
        TesseractPlannerImpl();

        void Init(const std::string& urdf_xml_string, const std::string& srdf_xml_string,
            std::function<std::string(std::string)> locator);
        
        virtual RR::GeneratorPtr<planning::PlanningResponsePtr,void> plan(planning::PlanningRequestPtr request);

    protected:

        std::shared_ptr<tesseract::Tesseract> tesseract_;
    };

    using TesseractPlannerImplPtr = RR_SHARED_PTR<TesseractPlannerImpl>;

    class PlannerGenerator : public RR::SyncGenerator<planning::PlanningResponsePtr,void>
    {
    public:

        void InitPlanner(std::shared_ptr<tesseract::Tesseract> tesseract,            
            planning::PlanningRequestPtr request
        );

        virtual planning::PlanningResponsePtr Next();

        virtual void Close();

        virtual void Abort();

    protected:

        std::shared_ptr<tesseract::Tesseract> tesseract_;        
        planning::PlanningRequestPtr request_;
        std::shared_ptr<tesseract_motion_planners::TrajOptPlannerFreespaceConfig> planner_config_;
        std::shared_ptr<tesseract_motion_planners::TrajOptMotionPlanner> planner_;
        boost::mutex this_lock;
        bool closed = false;
        bool aborted = false;
    };
}