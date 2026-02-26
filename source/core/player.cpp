//
// Created by orange on 26.02.2026.
//
#include "rose/core/player.hpp"
#include <omath/3d_primitives/mesh.hpp>
#include <omath/collision/epa_algorithm.hpp>
#include <omath/collision/gjk_algorithm.hpp>
#include <cmath>

namespace rose::core
{
    // ---------------------------------------------------------------------------
    // Type aliases for GJK / EPA with the base collider interface
    // ---------------------------------------------------------------------------
    using ColliderIface = omath::collision::ColliderInterface<omath::Vector3<float>>;
    using Gjk           = omath::collision::GjkAlgorithm<ColliderIface>;
    using Epa           = omath::collision::Epa<ColliderIface>;

    // ---------------------------------------------------------------------------
    // Build an AABB collider mesh with 8 vertices in local space (centred at 0).
    // The to-world matrix (translation = m_origin) handles the world position.
    // ---------------------------------------------------------------------------
    static omath::collision::MeshCollider<omath::opengl_engine::Mesh> make_box_collider()
    {
        using V = omath::primitives::Vertex<>;

        const float hw = Player::k_half_width;
        const float hh = Player::k_half_height;
        const float hd = Player::k_half_depth;

        // 8 corners of the box, winding doesn't matter for GJK support queries
        std::vector<V> verts = {
            {omath::Vector3<float>{-hw, -hh, -hd}, {}, {}},
            {omath::Vector3<float>{ hw, -hh, -hd}, {}, {}},
            {omath::Vector3<float>{ hw,  hh, -hd}, {}, {}},
            {omath::Vector3<float>{-hw,  hh, -hd}, {}, {}},
            {omath::Vector3<float>{-hw, -hh,  hd}, {}, {}},
            {omath::Vector3<float>{ hw, -hh,  hd}, {}, {}},
            {omath::Vector3<float>{ hw,  hh,  hd}, {}, {}},
            {omath::Vector3<float>{-hw,  hh,  hd}, {}, {}},
        };
        std::vector<omath::Vector3<uint32_t>> tris = {
            {0u,1u,2u}, {0u,2u,3u}, // -Z face
            {4u,6u,5u}, {4u,7u,6u}, // +Z face
            {0u,4u,5u}, {0u,5u,1u}, // -Y face
            {3u,2u,6u}, {3u,6u,7u}, // +Y face
            {0u,3u,7u}, {0u,7u,4u}, // -X face
            {1u,5u,6u}, {1u,6u,2u}, // +X face
        };

        return omath::collision::MeshCollider<omath::opengl_engine::Mesh>{
            omath::opengl_engine::Mesh{std::move(verts), std::move(tris)}
        };
    }

    // ---------------------------------------------------------------------------
    // Player
    // ---------------------------------------------------------------------------
    Player::Player(const omath::Vector3<float>& position)
        : m_position(position)
        , m_collider(make_box_collider())
    {
        m_collider.set_origin(position);
    }

    void Player::update(
        float dt,
        const std::vector<omath::collision::MeshCollider<omath::opengl_engine::Mesh>>& map_colliders,
        const PlayerInput& input)
    {
        // --- Mouse look ---
        auto angles = m_view_angles;
        angles.yaw   -= decltype(angles.yaw)  ::from_degrees(input.mouse_dx * k_mouse_sensitivity);
        angles.pitch -= decltype(angles.pitch)::from_degrees(input.mouse_dy * k_mouse_sensitivity);
        m_view_angles = angles;

        // --- Horizontal movement (ignores pitch, FPS style) ---
        const float yaw_rad = m_view_angles.yaw.as_radians();
        const omath::Vector3<float> forward = {-std::sin(yaw_rad), 0.f, -std::cos(yaw_rad)};
        const omath::Vector3<float> right   = { std::cos(yaw_rad), 0.f, -std::sin(yaw_rad)};

        omath::Vector3<float> move_dir = {0.f, 0.f, 0.f};
        if (input.forward)  move_dir = move_dir + forward;
        if (input.backward) move_dir = move_dir - forward;
        if (input.right)    move_dir = move_dir + right;
        if (input.left)     move_dir = move_dir - right;

        // Normalise horizontal direction
        const float move_len_sq = move_dir.x * move_dir.x + move_dir.z * move_dir.z;
        if (move_len_sq > 1e-6f)
        {
            const float inv = 1.f / std::sqrt(move_len_sq);
            move_dir.x *= inv;
            move_dir.z *= inv;
        }

        m_position.x += move_dir.x * k_move_speed * dt;
        m_position.z += move_dir.z * k_move_speed * dt;

        // --- Jump & gravity ---
        if (input.jump && m_is_grounded)
        {
            m_velocity.y  = k_jump_speed;
            m_is_grounded = false;
        }
        if (!m_is_grounded)
            m_velocity.y += k_gravity * dt;

        m_position.y += m_velocity.y * dt;

        // --- Collision resolution ---
        m_collider.set_origin(m_position);
        m_is_grounded = false; // re-detected each frame by resolve_collisions
        resolve_collisions(map_colliders);
    }

    void Player::resolve_collisions(
        const std::vector<omath::collision::MeshCollider<omath::opengl_engine::Mesh>>& map_colliders)
    {
        for (const auto& map_col : map_colliders)
        {
            // --- Broad GJK check ---
            const auto hit = Gjk::is_collide_with_simplex_info(map_col, m_collider);
            if (!hit.hit || hit.simplex.size() != 4)
                continue;

            // --- EPA for penetration vector ---
            const auto result = Epa::solve(map_col, m_collider, hit.simplex);
            if (!result)
                continue;

            // penetration_vector = normal * depth; adding it to player position
            // moves the player (A) out of the map mesh (B).
            const auto& pv = result->penetration_vector;
            m_position.x += pv.x;
            m_position.y += pv.y;
            m_position.z += pv.z;
            m_collider.set_origin(m_position);

            // --- Surface classification via normalised penetration direction ---
            const float pv_len = std::sqrt(pv.x * pv.x + pv.y * pv.y + pv.z * pv.z);
            if (pv_len < 1e-6f)
                continue;

            const float up_dot = pv.y / pv_len; // +1 = pure upward push → floor

            if (up_dot > k_floor_dot)
            {
                // Floor: stop falling
                m_is_grounded = true;
                if (m_velocity.y < 0.f) m_velocity.y = 0.f;
            }
            else if (up_dot < -k_floor_dot)
            {
                // Ceiling: stop rising
                if (m_velocity.y > 0.f) m_velocity.y = 0.f;
            }
            else
            {
                // Wall: remove velocity component pointing into the wall
                const float inv = 1.f / pv_len;
                const omath::Vector3<float> n = {pv.x * inv, pv.y * inv, pv.z * inv};
                const float vdotn = m_velocity.x * n.x + m_velocity.y * n.y + m_velocity.z * n.z;
                if (vdotn < 0.f)
                {
                    m_velocity.x -= n.x * vdotn;
                    m_velocity.y -= n.y * vdotn;
                    m_velocity.z -= n.z * vdotn;
                }
            }
        }
    }

    omath::Vector3<float> Player::get_eye_position() const
    {
        return {m_position.x, m_position.y + k_eye_height, m_position.z};
    }

    const omath::opengl_engine::ViewAngles& Player::get_view_angles() const
    {
        return m_view_angles;
    }
} // namespace rose::core
