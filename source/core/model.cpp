//
// Created by orange on 25.02.2026.
//

// tinygltf implementation — included in exactly this one translation unit.
#define TINYGLTF_IMPLEMENTATION
#define TINYGLTF_NO_STB_IMAGE_WRITE
#include <tiny_gltf.h>

#include "rose/core/model.hpp"
#include <spdlog/spdlog.h>
#include <stdexcept>

namespace rose::core
{
    // ---------------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------------

    // Returns a pointer to the first byte of accessor element 0, and fills
    // out_stride with the byte distance between consecutive elements.
    static const unsigned char* accessor_ptr(const tinygltf::Model& model,
                                             int accessor_index,
                                             size_t& out_stride)
    {
        const tinygltf::Accessor&   acc = model.accessors[accessor_index];
        const tinygltf::BufferView& bv  = model.bufferViews[acc.bufferView];
        const tinygltf::Buffer&     buf = model.buffers[bv.buffer];

        // byteStride == 0 means tightly packed.
        if (bv.byteStride != 0)
        {
            out_stride = bv.byteStride;
        }
        else
        {
            const size_t num_components = [&]() -> size_t
            {
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

            const size_t component_size = [&]() -> size_t
            {
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

    // Build an OpenGL Texture from a glTF image (already decoded by tinygltf).
    static std::shared_ptr<opengl::Texture> texture_from_image(const tinygltf::Image& image)
    {
        if (image.image.empty() || image.width <= 0 || image.height <= 0)
            return nullptr;

        return std::make_shared<opengl::Texture>(
            image.width, image.height, image.component, image.image.data());
    }

    // ---------------------------------------------------------------------------
    // Model
    // ---------------------------------------------------------------------------

    Model::Model(const std::filesystem::path& path)
    {
        load(path);
    }

    void Model::draw(const opengl::ShaderProgram& shader) const
    {
        for (const auto& mesh : m_meshes)
            mesh.draw(shader);
    }

    void Model::load(const std::filesystem::path& path)
    {
        tinygltf::Model     gltf;
        tinygltf::TinyGLTF  loader;
        std::string         err, warn;

        const bool ok = (path.extension() == ".glb")
            ? loader.LoadBinaryFromFile(&gltf, &err, &warn, path.string())
            : loader.LoadASCIIFromFile(&gltf, &err, &warn, path.string());

        if (!warn.empty())
            spdlog::warn("Model ({}): {}", path.filename().string(), warn);
        if (!err.empty())
            spdlog::error("Model ({}): {}", path.filename().string(), err);
        if (!ok)
            throw std::runtime_error("Failed to load model: " + path.string());

        // Pre-load all images into shared OpenGL textures so that multiple
        // primitives referencing the same image share one GPU upload.
        std::vector<std::shared_ptr<opengl::Texture>> gpu_textures;
        gpu_textures.reserve(gltf.images.size());
        for (const auto& image : gltf.images)
            gpu_textures.push_back(texture_from_image(image));

        // Iterate meshes → primitives.
        for (const auto& gltf_mesh : gltf.meshes)
        {
            for (const auto& prim : gltf_mesh.primitives)
            {
                if (prim.mode != TINYGLTF_MODE_TRIANGLES)
                    continue;

                // ---- Positions (required) ----
                auto pos_it = prim.attributes.find("POSITION");
                if (pos_it == prim.attributes.end())
                    continue;

                const tinygltf::Accessor& pos_acc = gltf.accessors[pos_it->second];
                const size_t vertex_count = pos_acc.count;
                std::vector<opengl::Vertex> vertices(vertex_count);

                {
                    size_t stride = 0;
                    const auto* data = accessor_ptr(gltf, pos_it->second, stride);
                    for (size_t i = 0; i < vertex_count; ++i)
                    {
                        const auto* p = reinterpret_cast<const float*>(data + i * stride);
                        vertices[i].position[0] = p[0];
                        vertices[i].position[1] = p[1];
                        vertices[i].position[2] = p[2];
                    }
                }

                // ---- Normals (optional) ----
                auto norm_it = prim.attributes.find("NORMAL");
                if (norm_it != prim.attributes.end())
                {
                    size_t stride = 0;
                    const auto* data = accessor_ptr(gltf, norm_it->second, stride);
                    for (size_t i = 0; i < vertex_count; ++i)
                    {
                        const auto* p = reinterpret_cast<const float*>(data + i * stride);
                        vertices[i].normal[0] = p[0];
                        vertices[i].normal[1] = p[1];
                        vertices[i].normal[2] = p[2];
                    }
                }

                // ---- UVs — TEXCOORD_0 (optional) ----
                auto uv_it = prim.attributes.find("TEXCOORD_0");
                if (uv_it != prim.attributes.end())
                {
                    size_t stride = 0;
                    const auto* data = accessor_ptr(gltf, uv_it->second, stride);
                    for (size_t i = 0; i < vertex_count; ++i)
                    {
                        const auto* p = reinterpret_cast<const float*>(data + i * stride);
                        vertices[i].uv[0] = p[0];
                        vertices[i].uv[1] = p[1];
                    }
                }

                // ---- Indices ----
                std::vector<uint32_t> indices;
                if (prim.indices >= 0)
                {
                    const tinygltf::Accessor& idx_acc = gltf.accessors[prim.indices];
                    size_t stride = 0;
                    const auto* data = accessor_ptr(gltf, prim.indices, stride);
                    indices.reserve(idx_acc.count);

                    for (size_t i = 0; i < idx_acc.count; ++i)
                    {
                        const auto* p = data + i * stride;
                        uint32_t index = 0;
                        switch (idx_acc.componentType)
                        {
                        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
                            index = static_cast<uint32_t>(*p);
                            break;
                        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
                            index = static_cast<uint32_t>(*reinterpret_cast<const uint16_t*>(p));
                            break;
                        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
                            index = *reinterpret_cast<const uint32_t*>(p);
                            break;
                        default:
                            break;
                        }
                        indices.push_back(index);
                    }
                }
                else
                {
                    // No index buffer — generate sequential indices.
                    indices.resize(vertex_count);
                    for (uint32_t i = 0; i < static_cast<uint32_t>(vertex_count); ++i)
                        indices[i] = i;
                }

                // ---- Material textures ----
                std::vector<opengl::MeshTexture> mesh_textures;
                if (prim.material >= 0)
                {
                    const auto& mat = gltf.materials[prim.material];
                    const int base_tex_idx = mat.pbrMetallicRoughness.baseColorTexture.index;
                    if (base_tex_idx >= 0)
                    {
                        const int source = gltf.textures[base_tex_idx].source;
                        if (source >= 0 && gpu_textures[static_cast<size_t>(source)])
                        {
                            mesh_textures.push_back({
                                gpu_textures[static_cast<size_t>(source)],
                                opengl::TextureType::BaseColor
                            });
                        }
                    }
                }

                m_meshes.emplace_back(std::move(vertices), std::move(indices), std::move(mesh_textures));
            }
        }
    }
} // namespace rose::core
