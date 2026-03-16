//
// Created by orange on 26.02.2026.
//
#include "rose/core/player.hpp"
#include <cmath>
#include <omath/3d_primitives/mesh.hpp>
#include <omath/collision/epa_algorithm.hpp>
#include <omath/collision/gjk_algorithm.hpp>

namespace rose::core
{
    static constexpr float k_look_smoothing = 25.f;

    // ---------------------------------------------------------------------------
    // Type aliases for GJK / EPA with the base collider interface
    // ---------------------------------------------------------------------------
    using ColliderIface = omath::collision::ColliderInterface<omath::Vector3<float>>;
    using Gjk = omath::collision::GjkAlgorithm<ColliderIface>;
    using Epa = omath::collision::Epa<ColliderIface>;

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
                {omath::Vector3<float>{hw, -hh, -hd}, {}, {}},
                {omath::Vector3<float>{hw, hh, -hd}, {}, {}},
                {omath::Vector3<float>{-hw, hh, -hd}, {}, {}},
                {omath::Vector3<float>{-hw, -hh, hd}, {}, {}},
                {omath::Vector3<float>{hw, -hh, hd}, {}, {}},
                {omath::Vector3<float>{hw, hh, hd}, {}, {}},
                {omath::Vector3<float>{-hw, hh, hd}, {}, {}},
        };
        std::vector<omath::Vector3<uint32_t>> tris = {
                {0u, 1u, 2u},
                {0u, 2u, 3u}, // -Z face
                {4u, 6u, 5u},
                {4u, 7u, 6u}, // +Z face
                {0u, 4u, 5u},
                {0u, 5u, 1u}, // -Y face
                {3u, 2u, 6u},
                {3u, 6u, 7u}, // +Y face
                {0u, 3u, 7u},
                {0u, 7u, 4u}, // -X face
                {1u, 5u, 6u},
                {1u, 6u, 2u}, // +X face
        };

        return omath::collision::MeshCollider<omath::opengl_engine::Mesh>{
                omath::opengl_engine::Mesh{std::move(verts), std::move(tris)}
        };
    }

    // ---------------------------------------------------------------------------
    // Player
    // ---------------------------------------------------------------------------
    Player::Player(const omath::Vector3<float>& position)
        : m_collider(make_box_collider())
    {
        m_collider.set_origin(position);
    }

    void Player::update(
            float dt,
            const CollisionWorld& world,
            const PlayerInput& input
    )
    {
        // --- Noclip toggle (edge-triggered on Q) ---
        if (input.noclip && !m_noclip_was_pressed)
        {
            m_noclip   = !m_noclip;
            m_velocity = {};
        }
        m_noclip_was_pressed = input.noclip;

        // --- Mouse look ---
        const float t = 1.f - std::exp(-k_look_smoothing * dt);
        m_smooth_dx += (input.mouse_dx - m_smooth_dx) * t;
        m_smooth_dy += (input.mouse_dy - m_smooth_dy) * t;
        m_view_angles.yaw   -= decltype(m_view_angles.yaw)::from_degrees(m_smooth_dx * k_mouse_sensitivity);
        m_view_angles.pitch -= decltype(m_view_angles.pitch)::from_degrees(m_smooth_dy * k_mouse_sensitivity);

        // --- Noclip free-cam movement ---
        if (m_noclip)
        {
            const float yaw_rad   = m_view_angles.yaw.as_radians();
            const float pitch_rad = m_view_angles.pitch.as_radians();
            const omath::Vector3<float> fwd = {
                -std::sin(yaw_rad) * std::cos(pitch_rad),
                 std::sin(pitch_rad),
                -std::cos(yaw_rad) * std::cos(pitch_rad)
            };
            const omath::Vector3<float> rgt = {std::cos(yaw_rad), 0.f, -std::sin(yaw_rad)};

            omath::Vector3<float> move{};
            if (input.forward)  move = move + fwd;
            if (input.backward) move = move - fwd;
            if (input.right)    move = move + rgt;
            if (input.left)     move = move - rgt;
            if (input.jump)     move.y += 1.f;

            const float len_sq = move.x*move.x + move.y*move.y + move.z*move.z;
            if (len_sq > 1e-6f)
            {
                const float inv = 1.f / std::sqrt(len_sq);
                move.x *= inv; move.y *= inv; move.z *= inv;
            }

            auto pos = m_collider.get_origin();
            pos.x += move.x * k_max_speed * dt;
            pos.y += move.y * k_max_speed * dt;
            pos.z += move.z * k_max_speed * dt;
            m_collider.set_origin(pos);
            return;
        }

        // --- Build wish direction from input ---
        const float yaw_rad = m_view_angles.yaw.as_radians();
        const omath::Vector3<float> forward = {-std::sin(yaw_rad), 0.f, -std::cos(yaw_rad)};
        const omath::Vector3<float> right   = {std::cos(yaw_rad),  0.f, -std::sin(yaw_rad)};

        omath::Vector3<float> wish_dir{};
        if (input.forward)  wish_dir = wish_dir + forward;
        if (input.backward) wish_dir = wish_dir - forward;
        if (input.right)    wish_dir = wish_dir + right;
        if (input.left)     wish_dir = wish_dir - right;

        const float len_sq = wish_dir.x * wish_dir.x + wish_dir.z * wish_dir.z;
        float wish_speed = 0.f;
        if (len_sq > 1e-6f)
        {
            const float inv = 1.f / std::sqrt(len_sq);
            wish_dir.x *= inv;
            wish_dir.z *= inv;
            wish_speed = k_max_speed;
        }

        // --- Queue jump for bhop ---
        if (input.jump)
            m_jump_queued = true;
        else
            m_jump_queued = false;

        // --- Ground / air movement ---
        if (m_is_grounded)
        {
            if (m_jump_queued)
            {
                // Bhop: jump immediately, skip friction to preserve speed
                m_velocity.y  = k_jump_speed;
                m_is_grounded = false;
                m_jump_queued = false;
            }
            else
            {
                apply_friction(dt);
            }
            accelerate(wish_dir, wish_speed, k_ground_accel, dt);
        }
        else
        {
            accelerate(wish_dir, wish_speed, k_air_accel, dt);
        }

        // --- Gravity ---
        if (!m_is_grounded)
            m_velocity.y += k_gravity * dt;

        // --- Integrate position ---
        auto position = m_collider.get_origin();
        position.x += m_velocity.x * dt;
        position.y += m_velocity.y * dt;
        position.z += m_velocity.z * dt;

        // --- Collision resolution (multiple passes to handle simultaneous contacts) ---
        m_is_grounded = false;
        m_collider.set_origin(position);

        for (int i = 0; i < 5; i++)
            resolve_collisions(world);
    }

    void Player::accelerate(
            const omath::Vector3<float>& wish_dir,
            float wish_speed,
            float accel,
            float dt)
    {
        const float current_speed = m_velocity.x * wish_dir.x + m_velocity.z * wish_dir.z;
        const float add_speed = wish_speed - current_speed;
        if (add_speed <= 0.f)
            return;

        float accel_speed = accel * wish_speed * dt;
        if (accel_speed > add_speed)
            accel_speed = add_speed;

        m_velocity.x += accel_speed * wish_dir.x;
        m_velocity.z += accel_speed * wish_dir.z;
    }

    void Player::apply_friction(float dt)
    {
        const float speed = std::sqrt(m_velocity.x * m_velocity.x + m_velocity.z * m_velocity.z);
        if (speed < 0.1f)
        {
            m_velocity.x = 0.f;
            m_velocity.z = 0.f;
            return;
        }

        const float control = speed < k_stop_speed ? k_stop_speed : speed;
        const float drop = control * k_friction * dt;
        float new_speed = speed - drop;
        if (new_speed < 0.f)
            new_speed = 0.f;
        new_speed /= speed;

        m_velocity.x *= new_speed;
        m_velocity.z *= new_speed;
    }

    void Player::resolve_collisions(const CollisionWorld& world)
    {
        const auto pos = m_collider.get_origin();

        const Aabb player_aabb{
            {pos.x - k_half_width, pos.y - k_half_height, pos.z - k_half_depth},
            {pos.x + k_half_width, pos.y + k_half_height, pos.z + k_half_depth}
        };

        m_query_buf.clear();
        world.query(player_aabb, m_query_buf);

        for (const int idx : m_query_buf)
        {
            if (!player_aabb.overlaps(world.aabbs[idx]))
                continue;

            const auto hit = Gjk::is_collide_with_simplex_info(world.colliders[idx], m_collider);
            if (!hit.hit)
                continue;

            const auto result = Epa::solve(world.colliders[idx], m_collider, hit.simplex);
            if (!result)
                continue;

            const auto& pv = result->penetration_vector;
            m_collider.set_origin(m_collider.get_origin() + (pv * 1.005f));
            const float up_dot = pv.y / pv.length();

            if (up_dot > k_floor_dot)
            {
                m_is_grounded = true;
                if (m_velocity.y < 0.f)
                    m_velocity.y = 0.f;
            }
            else if (up_dot < -k_floor_dot)
            {
                if (m_velocity.y > 0.f)
                    m_velocity.y = 0.f;
            }
        }
    }

    omath::Vector3<float> Player::get_eye_position() const
    {
        const auto position = m_collider.get_origin();
        return {position.x, position.y + k_eye_height, position.z};
    }

    const omath::opengl_engine::ViewAngles& Player::get_view_angles() const
    {
        return m_view_angles;
    }
} // namespace rose::core
