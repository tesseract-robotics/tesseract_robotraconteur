/**
 * @file geometry_conv.h
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

#ifndef TESSERACT_ROBOTRACONTEUR_GEOMETRY_CONV_H
#define TESSERACT_ROBOTRACONTEUR_GEOMETRY_CONV_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <RobotRaconteur.h>
#include <RobotRaconteurCompanion/StdRobDef/StdRobDefAll.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/geometries.h>

namespace rr_geom = com::robotraconteur::geometry;
namespace rr_shapes = com::robotraconteur::geometry::shapes;

namespace tesseract_robotraconteur
{
namespace conv
{
    rr_shapes::BoxPtr BoxToRR(const tesseract_geometry::Box& box);

    tesseract_geometry::Box::Ptr BoxFromRR(const rr_shapes::BoxPtr& box);

    rr_shapes::CapsulePtr CapsuleToRR(const tesseract_geometry::Capsule& capsule);

    tesseract_geometry::Capsule::Ptr CapsuleFromRR(const rr_shapes::CapsulePtr& capsule);

    rr_shapes::ConePtr ConeToRR(const tesseract_geometry::Cone& cone);

    tesseract_geometry::Cone::Ptr ConeFromRR(const rr_shapes::ConePtr& cone);

    std::tuple<rr_shapes::MeshPtr, rr_shapes::MaterialPtr> MeshToRR(const tesseract_geometry::PolygonMesh& mesh);

    tesseract_geometry::PolygonMesh::Ptr MeshFromRR(const rr_shapes::MeshPtr& mesh);

    rr_shapes::CylinderPtr CylinderToRR(const tesseract_geometry::Cylinder& cylinder);

    tesseract_geometry::Cylinder::Ptr CylinderFromRR(const rr_shapes::CylinderPtr& cylinder);

    // TODO: octree

    rr_shapes::PlanePtr PlaneToRR(const tesseract_geometry::Plane& plane);

    tesseract_geometry::Plane::Ptr PlaneFromRR(const rr_shapes::PlanePtr& plane);

    rr_shapes::SpherePtr SphereToRR(const tesseract_geometry::Sphere& sphere);

    tesseract_geometry::Sphere::Ptr SphereFromRR(const rr_shapes::SpherePtr& sphere);

    tesseract_geometry::Geometry::Ptr GeometryFromRR(const RobotRaconteur::RRValuePtr& geom);

    RobotRaconteur::RRValuePtr GeometryToRR(const tesseract_geometry::Geometry& geom);

    rr_shapes::MaterialPtr MeshMaterialToRR(const tesseract_geometry::MeshMaterial& mesh_material);

    tesseract_geometry::MeshMaterial::Ptr MeshMaterialFromRR(const rr_shapes::MaterialPtr& mesh_material);

    rr_shapes::MeshTexturePtr MeshTextureToRR(tesseract_geometry::MeshTexture& mesh_texture);

    tesseract_geometry::MeshTexture::Ptr MeshTextureFromRR(const rr_shapes::MeshTexturePtr& mesh_texture);

} // namespace conv
} // namespace tesseract_robotraconteur

#endif // TESSERACT_ROBOTRACONTEUR_GEOMETRY_CONV_H
