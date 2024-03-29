cmake_minimum_required(VERSION 3.15.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(yaml-cpp REQUIRED)
find_package(RobotRaconteur REQUIRED)
find_package(RobotRaconteurCompanion REQUIRED)
find_package(tesseract_task_composer REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(drekar-launch-process-cpp REQUIRED)

ROBOTRACONTEUR_GENERATE_THUNK(RR_THUNK_SRCS RR_THUNK_HDRS
    experimental.tesseract_robotics.common.robdef
    experimental.tesseract_robotics.command_language.robdef
    experimental.tesseract_robotics.environment.robdef
    experimental.tesseract_robotics.environment.commands.robdef
    experimental.tesseract_robotics.scene_graph.robdef
    experimental.tesseract_robotics.tasks.robdef
    experimental.tesseract_robotics.tasks.planning.robdef
    experimental.tesseract_robotics.robdef
	MASTER_HEADER
	INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/robdef ${RobotRaconteur_STANDARD_ROBDEF_DIRS}
	AUTO_IMPORT
	)


add_executable(${PROJECT_NAME}_service 
    src/tesseract_robotraconteur_service.cpp
    src/common_conv.cpp
    src/command_language_conv.cpp
    src/geometry_conv.cpp
    src/environment_commands_conv.cpp
    src/scene_graph_conv.cpp
    src/environment_impl.cpp
    src/tasks_impl.cpp
    src/tasks_conv.cpp
    src/task_exec_gen_impl.cpp
    src/tesseract_robotics_impl.cpp
    # src/tesseract_planner_impl.h
    # src/tesseract_planner_impl.cpp
    ${RR_THUNK_HDRS}
    ${RR_THUNK_SRCS}
)


target_link_libraries(${PROJECT_NAME}_service PRIVATE
    RobotRaconteurCore
    RobotRaconteurCompanion
    tesseract::tesseract_task_composer
    tesseract::tesseract_task_composer_planning
    tesseract::tesseract_task_composer_nodes
    tesseract::tesseract_common
    tesseract::tesseract_command_language
    tesseract::tesseract_geometry
    tesseract::tesseract_environment
    drekar-launch-process-cpp
    jsoncpp_lib
    yaml-cpp
    ${OpenCV_LIBS}    
)

target_link_libraries(${PROJECT_NAME}_service PRIVATE Boost::boost console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME}_service PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_service PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_service PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
# target_clang_tidy(${PROJECT_NAME} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_service PUBLIC VERSION ${TESSERACT_CXX_VERSION})

target_include_directories(${PROJECT_NAME}_service PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
                                                  "$<INSTALL_INTERFACE:include>")

include(GNUInstallDirs)

install(TARGETS tesseract_robotraconteur_service
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_subdirectory(examples/cpp)
