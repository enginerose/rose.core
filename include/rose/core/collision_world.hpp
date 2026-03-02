//
// Created by orange on 02.03.2026.
//
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

#include <omath/collision/mesh_collider.hpp>
#include <omath/engines/opengl_engine/mesh.hpp>
#include <omath/linear_algebra/vector3.hpp>

namespace rose::core
{
    // ---------------------------------------------------------------------------
    // Axis-aligned bounding box (world space).
    // ---------------------------------------------------------------------------
    struct Aabb
    {
        omath::Vector3<float> min{};
        omath::Vector3<float> max{};

        [[nodiscard]] bool overlaps(const Aabb& o) const noexcept
        {
            return min.x <= o.max.x && max.x >= o.min.x
                && min.y <= o.max.y && max.y >= o.min.y
                && min.z <= o.max.z && max.z >= o.min.z;
        }

        // Build world-space AABB by transforming every vertex through the
        // collider's to-world matrix.
        static Aabb from_collider(
            const omath::collision::MeshCollider<omath::opengl_engine::Mesh>& c)
        {
            constexpr float inf = std::numeric_limits<float>::max();
            Aabb box{{inf, inf, inf}, {-inf, -inf, -inf}};
            for (const auto& v : c.m_mesh.m_vertex_buffer)
            {
                const auto wp = c.m_mesh.vertex_position_to_world_space(v.position);
                box.min.x = std::min(box.min.x, wp.x);
                box.min.y = std::min(box.min.y, wp.y);
                box.min.z = std::min(box.min.z, wp.z);
                box.max.x = std::max(box.max.x, wp.x);
                box.max.y = std::max(box.max.y, wp.y);
                box.max.z = std::max(box.max.z, wp.z);
            }
            return box;
        }
    };

    // ---------------------------------------------------------------------------
    // Spatial-hash collision world.
    //
    // Build once at load time with CollisionWorld::build().
    // Each MeshCollider is inserted into every chunk its AABB overlaps.
    // At runtime, query() returns only the collider indices whose chunks
    // overlap a given AABB, giving an O(1)-ish broadphase.
    // ---------------------------------------------------------------------------
    struct CollisionWorld
    {
        using Collider = omath::collision::MeshCollider<omath::opengl_engine::Mesh>;

        // World is divided into axis-aligned cubes of this side length (metres).
        static constexpr float k_chunk_size = 16.f;

        std::vector<Collider>                          colliders;
        std::vector<Aabb>                              aabbs;   // parallel to colliders
        std::unordered_map<uint64_t, std::vector<int>> chunks;

        // Takes ownership of colliders, computes AABBs, and builds the chunk grid.
        static CollisionWorld build(std::vector<Collider> in_colliders)
        {
            CollisionWorld world;
            world.colliders = std::move(in_colliders);
            world.aabbs.reserve(world.colliders.size());

            for (int i = 0; i < static_cast<int>(world.colliders.size()); ++i)
            {
                const Aabb box = Aabb::from_collider(world.colliders[i]);
                world.aabbs.push_back(box);

                // Register this collider in every chunk its AABB overlaps.
                const int ix0 = chunk_coord(box.min.x);
                const int iy0 = chunk_coord(box.min.y);
                const int iz0 = chunk_coord(box.min.z);
                const int ix1 = chunk_coord(box.max.x);
                const int iy1 = chunk_coord(box.max.y);
                const int iz1 = chunk_coord(box.max.z);

                for (int ix = ix0; ix <= ix1; ++ix)
                    for (int iy = iy0; iy <= iy1; ++iy)
                        for (int iz = iz0; iz <= iz1; ++iz)
                            world.chunks[encode(ix, iy, iz)].push_back(i);
            }
            return world;
        }

        // Fill `out` with the (sorted, deduplicated) indices of colliders that
        // live in any chunk overlapping `aabb`.
        void query(const Aabb& aabb, std::vector<int>& out) const
        {
            const int ix0 = chunk_coord(aabb.min.x);
            const int iy0 = chunk_coord(aabb.min.y);
            const int iz0 = chunk_coord(aabb.min.z);
            const int ix1 = chunk_coord(aabb.max.x);
            const int iy1 = chunk_coord(aabb.max.y);
            const int iz1 = chunk_coord(aabb.max.z);

            for (int ix = ix0; ix <= ix1; ++ix)
                for (int iy = iy0; iy <= iy1; ++iy)
                    for (int iz = iz0; iz <= iz1; ++iz)
                    {
                        const auto it = chunks.find(encode(ix, iy, iz));
                        if (it != chunks.end())
                            for (const int idx : it->second)
                                out.push_back(idx);
                    }

            // Remove duplicates (player may span multiple chunks).
            std::ranges::sort(out);
            out.erase(std::ranges::unique(out).begin(), out.end());
        }

    private:
        static int chunk_coord(float v) noexcept
        {
            return static_cast<int>(std::floor(v / k_chunk_size));
        }

        // Pack three signed 21-bit chunk coordinates into a 63-bit key.
        // Supports chunk indices in [-1 048 576, 1 048 575] per axis.
        static uint64_t encode(int ix, int iy, int iz) noexcept
        {
            constexpr uint64_t mask = 0x1FFFFFu; // 21 bits
            return ((static_cast<uint64_t>(ix) & mask) << 42)
                 | ((static_cast<uint64_t>(iy) & mask) << 21)
                 |  (static_cast<uint64_t>(iz) & mask);
        }
    };
} // namespace rose::core
