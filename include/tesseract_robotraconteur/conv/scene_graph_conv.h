#ifndef TESSERACT_ROBOTRACONTEUR_SCENE_GRAPH_CONV_H
#define TESSERACT_ROBOTRACONTEUR_SCENE_GRAPH_CONV_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <RobotRaconteur.h>
#include <RobotRaconteurCompanion/StdRobDef/StdRobDefAll.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "robotraconteur_generated.h"

#include <tesseract_environment/commands.h>

#include "geometry_conv.h"

namespace rr_sg = experimental::tesseract_robotics::scene_graph;
namespace rr_geom = com::robotraconteur::geometry;

namespace tesseract_robotraconteur
{
namespace environment_conv
{
    // Visual
    rr_sg::VisualPtr VisualToRR(const tesseract_scene_graph::Visual& visual);

    rr_sg::VisualPtr VisualToRR(const tesseract_scene_graph::Visual::ConstPtr& visual);

    tesseract_scene_graph::Visual::Ptr VisualFromRR(const rr_sg::VisualPtr& visual);

    // LinkMaterial

    rr_shapes::MaterialPtr LinkMaterialToRR(const tesseract_scene_graph::Material& link_material);

    rr_shapes::MaterialPtr LinkMaterialToRR(const tesseract_scene_graph::Material::ConstPtr& link_material);

    tesseract_scene_graph::Material::Ptr LinkMaterialFromRR(const rr_shapes::MaterialPtr& link_material);

    // Collision

    rr_sg::CollisionPtr CollisionToRR(const tesseract_scene_graph::Collision& collision);

    rr_sg::CollisionPtr CollisionToRR(const tesseract_scene_graph::Collision::ConstPtr& collision);

    tesseract_scene_graph::Collision::Ptr CollisionFromRR(const rr_sg::CollisionPtr& collision);

    // Joint Dynamics

    rr_sg::JointDynamicsPtr JointDynamicsToRR(const tesseract_scene_graph::JointDynamics& joint_dynamics);

    rr_sg::JointDynamicsPtr JointDynamicsToRR(const tesseract_scene_graph::JointDynamics::ConstPtr joint_dynamics);

    tesseract_scene_graph::JointDynamics::Ptr JointDynamicsFromRR(const rr_sg::JointDynamicsPtr& joint_dynamics);

    // Joint Limits

    rr_sg::JointLimitsPtr JointLimitsToRR(const tesseract_scene_graph::JointLimits& joint_limits);

    rr_sg::JointLimitsPtr JointLimitsToRR(const tesseract_scene_graph::JointLimits::ConstPtr joint_limits);

    tesseract_scene_graph::JointLimits::Ptr JointLimitsFromRR(const rr_sg::JointLimitsPtr& joint_limits);

    // Joint Safety

    rr_sg::JointSafetyPtr JointSafetyToRR(const tesseract_scene_graph::JointSafety& joint_safety);

    rr_sg::JointSafetyPtr JointSafetyToRR(const tesseract_scene_graph::JointSafety::ConstPtr joint_safety);

    tesseract_scene_graph::JointSafety::Ptr JointSafetyFromRR(const rr_sg::JointSafetyPtr& joint_safety);

    // JointCalibration

    rr_sg::JointCalibrationPtr JointCalibrationToRR(const tesseract_scene_graph::JointCalibration& joint_calibration);

    rr_sg::JointCalibrationPtr JointCalibrationToRR(const tesseract_scene_graph::JointCalibration::ConstPtr joint_calibration);

    tesseract_scene_graph::JointCalibration::Ptr JointCalibrationFromRR(const rr_sg::JointCalibrationPtr& joint_calibration);

    // JointMimic

    rr_sg::JointMimicPtr JointMimicToRR(const tesseract_scene_graph::JointMimic& joint_mimic);

    rr_sg::JointMimicPtr JointMimicToRR(const tesseract_scene_graph::JointMimic::ConstPtr joint_mimic);

    tesseract_scene_graph::JointMimic::Ptr JointMimicFromRR(const rr_sg::JointMimicPtr& joint_mimic);


    // Joint
    rr_sg::JointPtr JointToRR(const tesseract_scene_graph::Joint& joint);

    rr_sg::JointPtr JointToRR(const tesseract_scene_graph::Joint::ConstPtr joint);

    tesseract_scene_graph::Joint::Ptr JointFromRR(const rr_sg::JointPtr& joint);

    // Inertial using com.robotraconteur.geometry.SpatialInertia

    rr_geom::SpatialInertia InertialToRR(const tesseract_scene_graph::Inertial& inertial);

    rr_geom::SpatialInertia InertialToRR(const tesseract_scene_graph::Inertial::ConstPtr inertial);

    tesseract_scene_graph::Inertial::Ptr InertialFromRR(const rr_geom::SpatialInertia& inertial);

    // Link
    rr_sg::LinkPtr LinkToRR(const tesseract_scene_graph::Link& link);

    rr_sg::LinkPtr LinkToRR(const tesseract_scene_graph::Link::ConstPtr link);

    tesseract_scene_graph::Link::Ptr LinkFromRR(const rr_sg::LinkPtr& link);

    // SceneGraph
    rr_sg::SceneGraphPtr SceneGraphToRR(const tesseract_scene_graph::SceneGraph& scene_graph);

    rr_sg::SceneGraphPtr SceneGraphToRR(const tesseract_scene_graph::SceneGraph::ConstPtr scene_graph);

    tesseract_scene_graph::SceneGraph::Ptr SceneGraphFromRR(const rr_sg::SceneGraphPtr& scene_graph);

  
} // namespace environment_conv
} // namespace tesseract_robotraconteur

#endif // TESSERACT_ROBOTRACONTEUR_SCENE_GRAPH_CONV_H
