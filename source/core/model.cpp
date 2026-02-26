//
// Created by orange on 25.02.2026.
//

// tinygltf implementation — included in exactly this one translation unit.
#define TINYGLTF_IMPLEMENTATION
#define TINYGLTF_NO_STB_IMAGE_WRITE
#include <tiny_gltf.h>

#include "rose/core/model.hpp"
#include <omath/engines/opengl_engine/constants.hpp>
#include <spdlog/spdlog.h>
#include <stdexcept>
#include <cmath>
#include <map>

namespace rose::core
{
    // ---------------------------------------------------------------------------
    // Accessor helper
    // ---------------------------------------------------------------------------

    static const unsigned char* accessor_ptr(const tinygltf::Model& model,
                                             int accessor_index,
                                             size_t& out_stride)
    {
        const tinygltf::Accessor&   acc = model.accessors[accessor_index];
        const tinygltf::BufferView& bv  = model.bufferViews[acc.bufferView];
        const tinygltf::Buffer&     buf = model.buffers[bv.buffer];

        if (bv.byteStride != 0)
        {
            out_stride = bv.byteStride;
        }
        else
        {
            const size_t num_components = [&]() -> size_t {
                switch (acc.type)
                {
                case TINYGLTF_TYPE_SCALAR: return 1;
                case TINYGLTF_TYPE_VEC2:   return 2;
                case TINYGLTF_TYPE_VEC3:   return 3;
                case TINYGLTF_TYPE_VEC4:   return 4;
                case TINYGLTF_TYPE_MAT2:   return 4;
                case TINYGLTF_TYPE_MAT3:   return 9;
                case TINYGLTF_TYPE_MAT4:   return 16;
                default:                   return 0;
                }
            }();
            const size_t component_size = [&]() -> size_t {
                switch (acc.componentType)
                {
                case TINYGLTF_COMPONENT_TYPE_BYTE:
                case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:  return 1;
                case TINYGLTF_COMPONENT_TYPE_SHORT:
                case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT: return 2;
                case TINYGLTF_COMPONENT_TYPE_INT:
                case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
                case TINYGLTF_COMPONENT_TYPE_FLOAT:          return 4;
                case TINYGLTF_COMPONENT_TYPE_DOUBLE:         return 8;
                default:                                     return 0;
                }
            }();
            out_stride = num_components * component_size;
        }

        return buf.data.data() + bv.byteOffset + acc.byteOffset;
    }

    // ---------------------------------------------------------------------------
    // Node transform: scale, translation, and rotation (as quaternion xyzw)
    // ---------------------------------------------------------------------------

    struct NodeTransform
    {
        omath::Vector3<float> scale       = {1.f, 1.f, 1.f};
        omath::Vector3<float> translation = {0.f, 0.f, 0.f};
        // glTF quaternion stored as xyzw
        float qx = 0.f, qy = 0.f, qz = 0.f, qw = 1.f;
    };

    // Convert a unit quaternion (xyzw) to omath ViewAngles.
    // Assumes omath builds its rotation matrix in YXZ order (Ry * Rx * Rz).
    // Derivation of R = Ry(y) * Rx(p) * Rz(r):
    //   R[1][2] = -sin(p)              → pitch = asin(-R[1][2])
    //   R[0][2]/R[2][2] = sin(y)/cos(y) → yaw   = atan2(R[0][2], R[2][2])
    //   R[1][0]/R[1][1] = sin(r)/cos(r) → roll  = atan2(R[1][0], R[1][1])
    static omath::opengl_engine::ViewAngles quat_to_view_angles(
        float qx, float qy, float qz, float qw)
    {
        using VA = omath::opengl_engine::ViewAngles;

        // Quaternion rotation-matrix elements (column-major, but element access is same)
        // R[row][col]
        const float r02 = 2.f * (qx * qz + qw * qy);   // sin(yaw)*cos(pitch)
        const float r12 = 2.f * (qy * qz - qw * qx);   // -sin(pitch)
        const float r22 = 1.f - 2.f * (qx * qx + qy * qy); // cos(yaw)*cos(pitch)
        const float r10 = 2.f * (qx * qy + qw * qz);   // cos(pitch)*sin(roll)
        const float r11 = 1.f - 2.f * (qx * qx + qz * qz); // cos(pitch)*cos(roll)

        const float pitch_rad = std::asin(std::clamp(-r12, -1.f, 1.f));
        const float yaw_rad   = std::atan2(r02, r22);
        const float roll_rad  = std::atan2(r10, r11);

        return VA{
            decltype(VA{}.pitch)::from_radians(pitch_rad),
            decltype(VA{}.yaw  )::from_radians(yaw_rad),
            decltype(VA{}.roll )::from_radians(roll_rad),
        };
    }

    static void collect_node_transforms(
        const tinygltf::Model&       gltf,
        int                          node_idx,
        std::map<int, NodeTransform>& out)
    {
        const tinygltf::Node& node = gltf.nodes[node_idx];

        if (node.mesh >= 0)
        {
            NodeTransform t;
            if (node.scale.size() >= 3)
                t.scale = {
                    static_cast<float>(node.scale[0]),
                    static_cast<float>(node.scale[1]),
                    static_cast<float>(node.scale[2])
                };
            if (node.translation.size() >= 3)
                t.translation = {
                    static_cast<float>(node.translation[0]),
                    static_cast<float>(node.translation[1]),
                    static_cast<float>(node.translation[2])
                };
            if (node.rotation.size() >= 4)
            {
                t.qx = static_cast<float>(node.rotation[0]);
                t.qy = static_cast<float>(node.rotation[1]);
                t.qz = static_cast<float>(node.rotation[2]);
                t.qw = static_cast<float>(node.rotation[3]);
            }
            out[node.mesh] = t;
        }

        for (int child : node.children)
            collect_node_transforms(gltf, child, out);
    }

    // ---------------------------------------------------------------------------
    // Texture helper
    // ---------------------------------------------------------------------------

    static std::shared_ptr<opengl::Texture> texture_from_image(const tinygltf::Image& image)
    {
        if (image.image.empty() || image.width <= 0 || image.height <= 0)
            return nullptr;
        return std::make_shared<opengl::Texture>(
            image.width, image.height, image.component, image.image.data());
    }

    // ---------------------------------------------------------------------------
    // Build one opengl::Mesh from a glTF primitive (no world transform baked in)
    // ---------------------------------------------------------------------------

    static opengl::Mesh build_mesh(
        const tinygltf::Model&                               gltf,
        const tinygltf::Primitive&                           prim,
        const std::vector<std::shared_ptr<opengl::Texture>>& gpu_textures)
    {
        auto pos_it = prim.attributes.find("POSITION");

        const size_t vertex_count = gltf.accessors[pos_it->second].count;
        std::vector<omath::primitives::Vertex<>> vertices(vertex_count);

        // ---- Positions ----
        {
            size_t stride = 0;
            const auto* data = accessor_ptr(gltf, pos_it->second, stride);
            for (size_t i = 0; i < vertex_count; ++i)
            {
                const auto* p = reinterpret_cast<const float*>(data + i * stride);
                vertices[i].position = {p[0], p[1], p[2]};
            }
        }

        // ---- Normals ----
        auto norm_it = prim.attributes.find("NORMAL");
        if (norm_it != prim.attributes.end())
        {
            size_t stride = 0;
            const auto* data = accessor_ptr(gltf, norm_it->second, stride);
            for (size_t i = 0; i < vertex_count; ++i)
            {
                const auto* p = reinterpret_cast<const float*>(data + i * stride);
                vertices[i].normal = {p[0], p[1], p[2]};
            }
        }

        // ---- UVs ----
        auto uv_it = prim.attributes.find("TEXCOORD_0");
        if (uv_it != prim.attributes.end())
        {
            size_t stride = 0;
            const auto* data = accessor_ptr(gltf, uv_it->second, stride);
            for (size_t i = 0; i < vertex_count; ++i)
            {
                const auto* p = reinterpret_cast<const float*>(data + i * stride);
                vertices[i].uv = {p[0], p[1]};
            }
        }

        // ---- Indices → packed triangles ----
        std::vector<omath::Vector3<uint32_t>> triangles;
        {
            std::vector<uint32_t> flat;
            if (prim.indices >= 0)
            {
                const tinygltf::Accessor& idx_acc = gltf.accessors[prim.indices];
                size_t stride = 0;
                const auto* data = accessor_ptr(gltf, prim.indices, stride);
                flat.reserve(idx_acc.count);
                for (size_t i = 0; i < idx_acc.count; ++i)
                {
                    const auto* p = data + i * stride;
                    uint32_t idx = 0;
                    switch (idx_acc.componentType)
                    {
                    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
                        idx = static_cast<uint32_t>(*p); break;
                    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
                        idx = static_cast<uint32_t>(*reinterpret_cast<const uint16_t*>(p)); break;
                    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
                        idx = *reinterpret_cast<const uint32_t*>(p); break;
                    default: break;
                    }
                    flat.push_back(idx);
                }
            }
            else
            {
                flat.resize(vertex_count);
                for (uint32_t i = 0; i < static_cast<uint32_t>(vertex_count); ++i)
                    flat[i] = i;
            }
            triangles.reserve(flat.size() / 3);
            for (size_t i = 0; i + 2 < flat.size(); i += 3)
                triangles.push_back({flat[i], flat[i+1], flat[i+2]});
        }

        // ---- Material / base-colour texture ----
        std::vector<opengl::MeshTexture> mesh_textures;
        if (prim.material >= 0)
        {
            const auto& mat = gltf.materials[prim.material];
            const int base_idx = mat.pbrMetallicRoughness.baseColorTexture.index;
            if (base_idx >= 0)
            {
                const int source = gltf.textures[base_idx].source;
                if (source >= 0 && gpu_textures[static_cast<size_t>(source)])
                    mesh_textures.push_back({gpu_textures[static_cast<size_t>(source)],
                                             opengl::TextureType::BaseColor});
            }
        }

        return opengl::Mesh{
            omath::opengl_engine::Mesh{std::move(vertices), std::move(triangles)},
            std::move(mesh_textures)
        };
    }

    // ---------------------------------------------------------------------------
    // Model
    // ---------------------------------------------------------------------------

    Model::Model(const std::filesystem::path& path) { load(path); }

    void Model::draw(const opengl::ShaderProgram& shader) const
    {
        for (const auto& mesh : m_meshes)
            mesh.draw(shader);
    }

    void Model::load(const std::filesystem::path& path)
    {
        tinygltf::Model    gltf;
        tinygltf::TinyGLTF loader;
        std::string        err, warn;

        const bool ok = (path.extension() == ".glb")
            ? loader.LoadBinaryFromFile(&gltf, &err, &warn, path.string())
            : loader.LoadASCIIFromFile(&gltf, &err, &warn, path.string());

        if (!warn.empty()) spdlog::warn("Model ({}): {}", path.filename().string(), warn);
        if (!err.empty())  spdlog::error("Model ({}): {}", path.filename().string(), err);
        if (!ok)           throw std::runtime_error("Failed to load model: " + path.string());

        // Pre-load GPU textures (shared across primitives)
        std::vector<std::shared_ptr<opengl::Texture>> gpu_textures;
        gpu_textures.reserve(gltf.images.size());
        for (const auto& image : gltf.images)
            gpu_textures.push_back(texture_from_image(image));

        // Collect per-mesh-index scale + translation from the scene graph
        std::map<int, NodeTransform> transforms;
        if (!gltf.scenes.empty())
        {
            const int scene_idx = gltf.defaultScene >= 0 ? gltf.defaultScene : 0;
            for (int root : gltf.scenes[scene_idx].nodes)
                collect_node_transforms(gltf, root, transforms);
        }

        // Flat iteration over all glTF meshes — no vertex transform baking
        for (int mesh_idx = 0; mesh_idx < static_cast<int>(gltf.meshes.size()); ++mesh_idx)
        {
            for (const auto& prim : gltf.meshes[mesh_idx].primitives)
            {
                if (prim.mode != TINYGLTF_MODE_TRIANGLES) continue;
                auto mesh = build_mesh(gltf, prim, gpu_textures);
                if (mesh.cpu_mesh().m_vertex_buffer.empty()) continue;

                // Apply node transform via omath Mesh setters
                if (auto it = transforms.find(mesh_idx); it != transforms.end())
                {
                    const auto& t = it->second;
                    mesh.cpu_mesh().set_scale(t.scale);
                    mesh.cpu_mesh().set_origin(t.translation);
                    mesh.cpu_mesh().set_rotation(quat_to_view_angles(t.qx, t.qy, t.qz, t.qw));
                }

                m_meshes.push_back(std::move(mesh));
            }
        }
    }
} // namespace rose::core
