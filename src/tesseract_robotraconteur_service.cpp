/**
 * @file tesseract_robotics_service.cpp
 *
 * @author John Wason, PhD
 *
 * @copyright Copyright 2023 Wason Technology, LLC
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * @par
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <iostream>

#include <boost/program_options.hpp>
#include "tesseract_robotraconteur/tesseract_robotics_impl.h"
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <drekar_launch_process_cpp/drekar_launch_process_cpp.h>

#include "robotraconteur_generated.h"

using namespace RobotRaconteur;
using namespace tesseract_robotraconteur;

namespace po = boost::program_options;

int main(int argc, char **argv)
{
    try
    {
        std::string urdf_fname;
        std::string srdf_fname;
        std::string task_plugin_config_file;

        po::options_description generic_("Allowed options");
        generic_.add_options()
            ("help,h", "print usage message")
            ("urdf-file", po::value(&urdf_fname), "URDF filename")
            ("srdf-file", po::value(&srdf_fname), "SRDF filename")
            ("task-plugin-config-file", po::value(&task_plugin_config_file), "Task composer plugin configuration file");

        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).options(generic_).run(), vm);
        po::notify(vm);

        // RobotRaconteur::Companion::RegisterStdRobDefServiceTypes();

        auto resource_locator = std::make_shared<tesseract_common::GeneralResourceLocator>();

        tesseract_common::fs::path urdf_path(urdf_fname);
        tesseract_common::fs::path srdf_path(srdf_fname);
        tesseract_common::fs::path task_plugin_config_file_path(task_plugin_config_file);

        if (!boost::filesystem::exists(urdf_path))
        {
            std::cerr << "URDF file does not exist: " << urdf_fname << std::endl;
            return 1;
        }

        if (!boost::filesystem::exists(srdf_path))
        {
            std::cerr << "SRDF file does not exist: " << srdf_fname << std::endl;
            return 1;
        }

        if (!boost::filesystem::exists(task_plugin_config_file_path))
        {
            std::cerr << "Task plugin config file does not exist: " << task_plugin_config_file << std::endl;
            return 1;
        }

        auto env = std::make_shared<tesseract_environment::Environment>();
        env->init(urdf_path, srdf_path, resource_locator);
        if (!env->isInitialized())
        {
            std::cerr << "Failed to initialize environment." << std::endl;
            return 1;
        }

        auto executor_factory = std::make_shared<tesseract_planning::TaskComposerPluginFactory>(task_plugin_config_file_path);

        tesseract_planning::TaskComposerExecutor::Ptr default_executor = executor_factory->createTaskComposerExecutor("TaskflowExecutor");

        auto obj = RR_MAKE_SHARED<TesseractRoboticsImpl>(env, default_executor, executor_factory);
        obj->Init();

        ServerNodeSetup node_setup(ROBOTRACONTEUR_SERVICE_TYPES, "org.swri.tesseract", 63158, argc, argv);

        auto context = RobotRaconteurNode::s()->RegisterService("tesseract","experimental.tesseract_robotics", obj);
        context->AddExtraImport("experimental.tesseract_robotics.command_language");
        context->AddExtraImport("experimental.tesseract_robotics.tasks.planning");


        std::cout << "Press Ctrl-C to quit" << std::endl;
        drekar_launch_process_cpp::CWaitForExit wait_exit;
        wait_exit.WaitForExit();
    }
    catch (std::exception& exp)
    {
        std::cerr << "Error: " << exp.what() << std::endl;
        return 1;
    }

    return 0;
}