#include "tesseract_robotraconteur/conv/geometry_conv.h"
#include "tesseract_robotraconteur/conv/common_conv.h"
#include "RobotRaconteurCompanion/Converters/EigenConverters.h"

#include <boost/range/algorithm.hpp>
#include <boost/numeric/conversion/cast.hpp>

namespace RR=RobotRaconteur;
namespace RRC_Eigen=RobotRaconteur::Companion::Converters::Eigen;
namespace rr_color = com::robotraconteur::color;
namespace rr_image = com::robotraconteur::image;

#define RR_SHAPES_PREFIX "com.robotraconteur.geometry.shapes"

namespace tesseract_robotraconteur
{
namespace conv
{
    rr_shapes::BoxPtr BoxToRR(const tesseract_geometry::Box& box)
    {
        rr_shapes::BoxPtr ret(new rr_shapes::Box());
        ret->x = box.getX();
        ret->y = box.getY();
        ret->z = box.getZ();
        return ret;
    }

    tesseract_geometry::Box::Ptr BoxFromRR(const rr_shapes::BoxPtr& box)
    {
        RR_NULL_CHECK(box);
        return std::make_shared<tesseract_geometry::Box>(box->x, box->y, box->z);
    }

    rr_shapes::CapsulePtr CapsuleToRR(const tesseract_geometry::Capsule& capsule)
    {
        rr_shapes::CapsulePtr ret(new rr_shapes::Capsule());
        ret->radius = capsule.getRadius();
        ret->height = capsule.getLength();
        return ret;
    }

    tesseract_geometry::Capsule::Ptr CapsuleFromRR(const rr_shapes::CapsulePtr& capsule)
    {
        RR_NULL_CHECK(capsule);
        return std::make_shared<tesseract_geometry::Capsule>(capsule->radius, capsule->height);
    }

    rr_shapes::ConePtr ConeToRR(const tesseract_geometry::Cone& cone)
    {
        rr_shapes::ConePtr ret(new rr_shapes::Cone());
        ret->radius = cone.getRadius();
        ret->height = cone.getLength();
        return ret;
    }

    tesseract_geometry::Cone::Ptr ConeFromRR(const rr_shapes::ConePtr& cone)
    {
        RR_NULL_CHECK(cone);
        return std::make_shared<tesseract_geometry::Cone>(cone->radius, cone->height);
    }

    std::tuple<rr_shapes::MeshPtr, rr_shapes::MaterialPtr> MeshToRR(const tesseract_geometry::PolygonMesh& mesh)
    {
        rr_shapes::MeshPtr ret(new rr_shapes::Mesh());
        rr_shapes::MaterialPtr ret_material;
        
        if ((static_cast<long>(mesh.getFaceCount()) * 4) != mesh.getFaces()->size())
        {
            throw RR::InvalidArgumentException("Mesh is not triangular");
        }

        ret->triangles = RR::AllocateEmptyRRNamedArray<rr_shapes::MeshTriangle>(mesh.getFaceCount());
        auto& faces = *mesh.getFaces();
        for (size_t i=0; i<mesh.getFaceCount(); ++i)
        {
            rr_shapes::MeshTriangle& triangle = ret->triangles->at(i);
            triangle.s.v1 = faces[i*4+1];
            triangle.s.v2 = faces[i*4+2];
            triangle.s.v3 = faces[i*4+3];
        }

        ret->vertices = RR::AllocateEmptyRRNamedArray<rr_geom::Point>(mesh.getVertexCount());
        auto& vertices = *mesh.getVertices();
        auto& scale = mesh.getScale();
        for (size_t i=0; i<mesh.getVertexCount(); ++i)
        {
            rr_geom::Point& point = ret->vertices->at(i);
            auto& vertex = vertices[i];
            point.s.x = vertex.x() * scale.x();
            point.s.y = vertex.y() * scale.y();
            point.s.z = vertex.z() * scale.z();
        }

        auto& normals = mesh.getNormals();
        if (normals)
        {
            ret->normals = RR::AllocateEmptyRRNamedArray<rr_geom::Vector3>(normals->size());
            for (size_t i=0; i<normals->size(); ++i)
            {
                rr_geom::Vector3& normal = ret->normals->at(i);
                auto& n = normals->at(i);
                normal.s.x = n.x();
                normal.s.y = n.y();
                normal.s.z = n.z();
            }
        }

        auto& vertex_colors = mesh.getVertexColors();
        if (vertex_colors)
        {
            ret->colors = RR::AllocateEmptyRRNamedArray<rr_color::ColorRGB>(vertex_colors->size());
            for (size_t i=0; i<vertex_colors->size(); ++i)
            {
                rr_color::ColorRGB& color = ret->colors->at(i);
                auto& c = vertex_colors->at(i);
                color.s.r = c(0);
                color.s.g = c(1);
                color.s.b = c(2);
            }
        }

        auto& mesh_textures = mesh.getTextures();
        if (mesh_textures)
        {
            ret->textures = RR::AllocateEmptyRRList<rr_shapes::MeshTexture>();
            // boost::range::transform(*mesh_textures, std::back_inserter(*ret->textures), MeshTextureToRR);
            for (size_t i=0; i<mesh_textures->size(); ++i)
            {
                ret->textures->push_back(MeshTextureToRR(*mesh_textures->at(i)));
            }
        }

        // TODO: MeshMaterial

        switch (mesh.getType())
        {
            case tesseract_geometry::GeometryType::MESH:
            case tesseract_geometry::GeometryType::POLYGON_MESH:
                ret->mesh_type = rr_shapes::MeshType::mesh;
                break;
            case tesseract_geometry::GeometryType::SDF_MESH:
                ret->mesh_type = rr_shapes::MeshType::sdf_mesh;
                break;
            case tesseract_geometry::GeometryType::CONVEX_MESH:
                ret->mesh_type = rr_shapes::MeshType::convex_mesh;
                break;
            default:
                throw RR::InvalidArgumentException("Unknown mesh type");
        }
        
        return std::make_tuple(ret, ret_material);
    }

    tesseract_geometry::PolygonMesh::Ptr MeshFromRR(const rr_shapes::MeshPtr& mesh)
    {
        RR_NULL_CHECK(mesh);
        tesseract_geometry::PolygonMesh::Ptr ret;
        auto faces = std::make_shared<Eigen::VectorXi>(mesh->triangles->size() * 4);
        auto& faces_ref = *faces;
        for (size_t i=0; i<mesh->triangles->size(); ++i)
        {
            const rr_shapes::MeshTriangle& triangle = mesh->triangles->at(i);
            faces_ref[i*4+0] = 3;
            faces_ref[i*4+1] = triangle.s.v1;
            faces_ref[i*4+2] = triangle.s.v2;
            faces_ref[i*4+3] = triangle.s.v3;
        }

        int face_count = boost::numeric_cast<const int&>(mesh->triangles->size());

        auto vertices = std::make_shared<tesseract_common::VectorVector3d>(mesh->vertices->size());
        auto& vertices_ref = *vertices;
        for (size_t i=0; i<mesh->vertices->size(); ++i)
        {
            const rr_geom::Point& point = mesh->vertices->at(i);
            vertices_ref[i] = Eigen::Vector3d(point.s.x, point.s.y, point.s.z);
        }

        Eigen::Vector3d scale(1,1,1);

        std::shared_ptr<tesseract_common::VectorVector3d> normals;
        if (mesh->normals && mesh->normals->size() > 0)
        {
            normals = std::make_shared<tesseract_common::VectorVector3d>(mesh->normals->size());
            auto& normals_ref = *normals;
            for (size_t i=0; i<mesh->normals->size(); ++i)
            {
                const rr_geom::Vector3& normal = mesh->normals->at(i);
                normals_ref[i] = Eigen::Vector3d(normal.s.x, normal.s.y, normal.s.z);
            }
        }

        std::shared_ptr<tesseract_common::VectorVector4d> vertex_colors;
        if (mesh->colors && mesh->colors->size() > 0)
        {
            vertex_colors = std::make_shared<tesseract_common::VectorVector4d>(mesh->colors->size());
            auto& vertex_colors_ref = *vertex_colors;
            for (size_t i=0; i<mesh->colors->size(); ++i)
            {
                const rr_color::ColorRGB& color = mesh->colors->at(i);
                vertex_colors_ref[i] = Eigen::Vector4d(color.s.r, color.s.g, color.s.b, 1.0);
            }
        }

        std::shared_ptr<std::vector<tesseract_geometry::MeshTexture::Ptr>> mesh_textures;
        if (mesh->textures)
        {
            mesh_textures = std::make_shared<std::vector<tesseract_geometry::MeshTexture::Ptr>>(mesh->textures->size());
            boost::range::transform(*mesh->textures, std::back_inserter(*mesh_textures), MeshTextureFromRR);
        }

        switch (mesh->mesh_type)
        {
            case rr_shapes::MeshType::mesh:
                ret = std::make_shared<tesseract_geometry::PolygonMesh>(vertices, faces, face_count, nullptr, scale, normals, vertex_colors, nullptr, mesh_textures);
                break;
            case rr_shapes::MeshType::sdf_mesh:
                ret = std::make_shared<tesseract_geometry::SDFMesh>(vertices, faces, face_count, nullptr, scale, normals, vertex_colors, nullptr, mesh_textures);
                break;
            case rr_shapes::MeshType::convex_mesh:
                ret = std::make_shared<tesseract_geometry::ConvexMesh>(vertices, faces, face_count, nullptr, scale, normals, vertex_colors, nullptr, mesh_textures);
                break;
            default:
                throw RR::InvalidArgumentException("Unknown mesh type");
        }

        return ret;        
    }

    rr_shapes::CylinderPtr CylinderToRR(const tesseract_geometry::Cylinder& cylinder)
    {
        rr_shapes::CylinderPtr ret(new rr_shapes::Cylinder());
        ret->radius = cylinder.getRadius();
        ret->height = cylinder.getLength();
        return ret;
    }

    tesseract_geometry::Cylinder::Ptr CylinderFromRR(const rr_shapes::CylinderPtr& cylinder)
    {
        RR_NULL_CHECK(cylinder);
        return std::make_shared<tesseract_geometry::Cylinder>(cylinder->radius, cylinder->height);
    }

    // TODO: octree

    rr_shapes::PlanePtr PlaneToRR(const tesseract_geometry::Plane& plane)
    {
        rr_shapes::PlanePtr ret(new rr_shapes::Plane());
        ret->a = plane.getA();
        ret->b = plane.getB();
        ret->c = plane.getC();
        ret->d = plane.getD();
        return ret;
    }

    tesseract_geometry::Plane::Ptr PlaneFromRR(const rr_shapes::PlanePtr& plane)
    {
        RR_NULL_CHECK(plane);
        return std::make_shared<tesseract_geometry::Plane>(plane->a, plane->b, plane->c, plane->d);
    }

    rr_shapes::SpherePtr SphereToRR(const tesseract_geometry::Sphere& sphere)
    {
        rr_shapes::SpherePtr ret(new rr_shapes::Sphere());
        ret->radius = sphere.getRadius();
        return ret;
    }

    tesseract_geometry::Sphere::Ptr SphereFromRR(const rr_shapes::SpherePtr& sphere)
    {
        RR_NULL_CHECK(sphere);
        return std::make_shared<tesseract_geometry::Sphere>(sphere->radius);
    }

    tesseract_geometry::Geometry::Ptr GeometryFromRR(const RobotRaconteur::RRValuePtr& geom)
    {
        RR_NULL_CHECK(geom);
        std::string rr_type = geom->RRType();
        if (rr_type == RR_SHAPES_PREFIX ".Box")
        {
            return BoxFromRR(RR::rr_cast<rr_shapes::Box>(geom));
        }
        else if (rr_type == RR_SHAPES_PREFIX ".Capsule")
        {
            return CapsuleFromRR(RR::rr_cast<rr_shapes::Capsule>(geom));
        }
        else if (rr_type == RR_SHAPES_PREFIX ".Cone")
        {
            return ConeFromRR(RR::rr_cast<rr_shapes::Cone>(geom));
        }
        else if (rr_type == RR_SHAPES_PREFIX ".Cylinder")
        {
            return CylinderFromRR(RR::rr_cast<rr_shapes::Cylinder>(geom));
        }
        else if (rr_type == RR_SHAPES_PREFIX ".Plane")
        {
            return PlaneFromRR(RR::rr_cast<rr_shapes::Plane>(geom));
        }
        else if (rr_type == RR_SHAPES_PREFIX ".Sphere")
        {
            return SphereFromRR(RR::rr_cast<rr_shapes::Sphere>(geom));
        }
        else if (rr_type == RR_SHAPES_PREFIX ".Mesh")
        {
            return MeshFromRR(RR::rr_cast<rr_shapes::Mesh>(geom));
        }
        else
        {
            throw RR::InvalidArgumentException("Unknown geometry type");
        }
    }

    RobotRaconteur::RRValuePtr GeometryToRR(const tesseract_geometry::Geometry& geom)
    {
        switch (geom.getType())
        {
            case tesseract_geometry::GeometryType::BOX:
                return BoxToRR(static_cast<const tesseract_geometry::Box&>(geom));
            case tesseract_geometry::GeometryType::CAPSULE:
                return CapsuleToRR(static_cast<const tesseract_geometry::Capsule&>(geom));
            case tesseract_geometry::GeometryType::CONE:
                return ConeToRR(static_cast<const tesseract_geometry::Cone&>(geom));
            case tesseract_geometry::GeometryType::CYLINDER:
                return CylinderToRR(static_cast<const tesseract_geometry::Cylinder&>(geom));
            case tesseract_geometry::GeometryType::PLANE:
                return PlaneToRR(static_cast<const tesseract_geometry::Plane&>(geom));
            case tesseract_geometry::GeometryType::SPHERE:
                return SphereToRR(static_cast<const tesseract_geometry::Sphere&>(geom));
            case tesseract_geometry::GeometryType::MESH:
            case tesseract_geometry::GeometryType::POLYGON_MESH:
            case tesseract_geometry::GeometryType::SDF_MESH:
            case tesseract_geometry::GeometryType::CONVEX_MESH:
                return std::get<0>(MeshToRR(static_cast<const tesseract_geometry::PolygonMesh&>(geom)));
            default:
                throw RR::InvalidArgumentException("Unknown geometry type");
        }
    }

    rr_shapes::MaterialPtr MeshMaterialToRR(const tesseract_geometry::MeshMaterial& mesh_material)
    {
        throw std::runtime_error("Not implemented");
    }

    tesseract_geometry::MeshMaterial::Ptr MeshMaterialFromRR(const rr_shapes::MaterialPtr& mesh_material)
    {
        throw std::runtime_error("Not implemented");
    }

    rr_shapes::MeshTexturePtr MeshTextureToRR(tesseract_geometry::MeshTexture& mesh_texture)
    {
        rr_shapes::MeshTexturePtr ret(new rr_shapes::MeshTexture());
        auto& uvs = mesh_texture.getUVs();
        auto& uvs_ref = *uvs;
        ret->uvs = RR::AllocateEmptyRRNamedArray<rr_geom::Vector2>(uvs->size());        
        for (size_t i=0; i<uvs->size(); ++i)
        {
            rr_geom::Vector2& point = ret->uvs->at(i);
            auto& uv = uvs_ref[i];
            point.s.x = uv.x();
            point.s.y = uv.y();
        }

        rr_image::CompressedImagePtr image(new rr_image::CompressedImage());
        auto img_res = mesh_texture.getTextureImage();
        if(!img_res)
        {
            throw RR::InvalidArgumentException("Texture image resource is null");
        }
        auto img_data = img_res->getResourceContents();
        image->data = RR::AttachRRArrayCopy(img_data.data(), img_data.size());

        rr_image::ImageInfoPtr image_info(new rr_image::ImageInfo());
        image_info->width = 0;
        image_info->height = 0;
        image_info->step = 0;
        image_info->encoding = rr_image::ImageEncoding::compressed;

        image->image_info = image_info;

        ret->image = image;

        return ret;
    }

    tesseract_geometry::MeshTexture::Ptr MeshTextureFromRR(const rr_shapes::MeshTexturePtr& mesh_texture)
    {
        RR_NULL_CHECK(mesh_texture);
        RR_NULL_CHECK(mesh_texture->image);
        RR_NULL_CHECK(mesh_texture->uvs);
        RR_NULL_CHECK(mesh_texture->image->data);

        auto uvs = std::make_shared<tesseract_common::VectorVector2d>(mesh_texture->uvs->size());
        auto& uvs_ref = *uvs;
        for (size_t i=0; i<mesh_texture->uvs->size(); ++i)
        {
            const rr_geom::Vector2& point = mesh_texture->uvs->at(i);
            uvs_ref[i] = Eigen::Vector2d(point.s.x, point.s.y);
        }

        auto& rr_data = mesh_texture->image->data;
        auto data = std::vector<uint8_t>(rr_data->data(), rr_data->data() + rr_data->size());

        auto resource = std::make_shared<tesseract_common::BytesResource>("", std::move(data));

        return std::make_shared<tesseract_geometry::MeshTexture>(resource, uvs);
    }

} // namespace conv
} // namespace tesseract_robotraconteur
