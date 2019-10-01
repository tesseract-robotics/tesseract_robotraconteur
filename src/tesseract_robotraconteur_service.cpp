#include <iostream>
#include <signal.h>
#include <RobotRaconteur.h>

#include "robotraconteur_generated.h"
#include "tesseract_planner_impl.h"

using namespace RobotRaconteur;
using namespace tesseract_robotraconteur;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "tesseract_planner");

    ros::NodeHandle nh;

    ServerNodeSetup node_setup(ROBOTRACONTEUR_SERVICE_TYPES, "org.swri.tesseract", 63158);

    auto obj = RR_MAKE_SHARED<TesseractPlannerImpl>(nh);
    obj->Init();

    RR::RobotRaconteurNode::s()->RegisterService("tesseract","com.robotraconteur.robotics.planning", obj);

    std::cerr << "Press Ctrl-C to quit" << std::endl;

    /*sigset_t   set;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    sigprocmask( SIG_BLOCK, &set, NULL );
    int sig;
    sigwait(&set, &sig);*/

    ros::spin();


    std::cerr << "Goodbye!" << std::endl;

    return 0;
}
