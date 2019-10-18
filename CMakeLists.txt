cmake_minimum_required(VERSION 2.8.3)
project(tesseract_robotraconteur)

set (CMAKE_CXX_STANDARD 11)

find_package(tesseract REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_kinematics REQUIRED)
#find_package(tesseract_rosutils REQUIRED)
find_package(trajopt REQUIRED)
find_package(trajopt_utils REQUIRED)
find_package(trajopt_sco REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(Bullet REQUIRED)
find_package(RobotRaconteur REQUIRED)
find_package(RobotRaconteur_standard_robdef_cpp REQUIRED)
find_package(assimp REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost COMPONENTS date_time filesystem system regex chrono atomic thread random program_options locale REQUIRED)
link_directories(${ASSIMP_LIBRARY_DIRS})

include_directories(${RobotRaconteur_INCLUDE_DIR} 
                    ${CMAKE_CURRENT_BINARY_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${tesseract_INCLUDE_DIRS}
                    ${tesseract_environment_INCLUDE_DIRS}
                    ${tesseract_kinematics_INCLUDE_DIRS}
                    ${BULLET_INCLUDE_DIRS}
                    ${trajopt_INCLUDE_DIRS}
                    ${trajopt_utils_INCLUDE_DIRS}
                    ${trajopt_sco_INCLUDE_DIRS}
                    ${YAML_CPP_INCLUDE_DIR}
                    )

ROBOTRACONTEUR_GENERATE_THUNK(RR_THUNK_SRCS RR_THUNK_HDRS
    com.robotraconteur.geometry.robdef
    com.robotraconteur.geometry.shapes.robdef
    com.robotraconteur.color.robdef
    com.robotraconteur.units.robdef
    com.robotraconteur.datetime.robdef
    com.robotraconteur.sensordata.robdef
    com.robotraconteur.uuid.robdef
    com.robotraconteur.datatype.robdef
    com.robotraconteur.device.robdef
    com.robotraconteur.signal.robdef
    com.robotraconteur.param.robdef
    com.robotraconteur.robotics.joints.robdef
    com.robotraconteur.robotics.robot.robdef
    com.robotraconteur.robotics.tool.robdef
    com.robotraconteur.robotics.payload.robdef
    com.robotraconteur.robotics.trajectory.robdef
    com.robotraconteur.robotics.planning.robdef
    com.robotraconteur.identifier.robdef
    com.robotraconteur.resource.robdef
    MASTER_HEADER
    IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR} ${RobotRaconteur_STANDARD_ROBDEF_DIRS}
)

add_executable(tesseract_robotraconteur_service 
    src/tesseract_robotraconteur_service.cpp
    src/tesseract_planner_impl.h
    src/tesseract_planner_impl.cpp
    ${RR_THUNK_HDRS}
    ${RR_THUNK_SRCS}
)

target_link_libraries(tesseract_robotraconteur_service 
    RobotRaconteurCore    
    ${catkin_LIBRARIES}
    tesseract::tesseract_motion_planners_core
    tesseract::tesseract_motion_planners_trajopt_freespace
    tesseract::tesseract
    trajopt::trajopt_sco
    jsoncpp_lib
    yaml-cpp
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}    
    ${RobotRaconteurCore_Extra_Libraries}
)

include(GNUInstallDirs)

install(TARGETS tesseract_robotraconteur_service
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})