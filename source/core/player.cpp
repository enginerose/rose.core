//
// Created by orange on 26.02.2026.
//
#include "rose/core/player.hpp"
#include <algorithm>
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
            m_jump_buffer_timer = 0.f;
            m_ground_grace_timer = 0.f;
            m_jump_ground_lockout_timer = 0.f;
            m_wall_run_cooldown_timer = 0.f;
            m_has_wall_contact = false;
            m_is_wall_running = false;
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
            m_jump_was_pressed = input.jump;
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

        const bool jump_pressed = input.jump;
        const bool jump_started = jump_pressed && !m_jump_was_pressed;
        const bool jump_released = !jump_pressed && m_jump_was_pressed;
        const bool was_wall_running = m_is_wall_running;
        m_jump_was_pressed = jump_pressed;

        if (m_jump_ground_lockout_timer > 0.f)
            m_jump_ground_lockout_timer = (m_jump_ground_lockout_timer > dt)
                ? (m_jump_ground_lockout_timer - dt)
                : 0.f;
        if (m_wall_run_cooldown_timer > 0.f)
            m_wall_run_cooldown_timer = (m_wall_run_cooldown_timer > dt)
                ? (m_wall_run_cooldown_timer - dt)
                : 0.f;

        if (m_is_grounded)
        {
            m_ground_grace_timer = k_ground_grace_time;
            m_wall_run_cooldown_timer = 0.f;
        }
        else if (m_ground_grace_timer > 0.f)
        {
            m_ground_grace_timer = (m_ground_grace_timer > dt) ? (m_ground_grace_timer - dt) : 0.f;
        }

        if (jump_started || (input.auto_bhop && jump_pressed))
            m_jump_buffer_timer = k_jump_buffer_time;
        else if (m_jump_buffer_timer > 0.f)
            m_jump_buffer_timer = (m_jump_buffer_timer > dt) ? (m_jump_buffer_timer - dt) : 0.f;

        const bool can_jump = m_is_grounded || m_ground_grace_timer > 0.f;
        const bool should_jump = m_jump_buffer_timer > 0.f && can_jump;

        // --- Source-style ground / air movement ---
        if (m_is_grounded)
            m_velocity.y = 0.f;

        if (should_jump)
        {
            // Source skips ground friction on the jump frame.
            m_velocity.y  = k_jump_speed;
            m_is_grounded = false;
            m_jump_buffer_timer = 0.f;
            m_ground_grace_timer = 0.f;
            m_jump_ground_lockout_timer = k_jump_ground_lockout_time;
        }
        else if (m_is_grounded)
        {
            apply_friction(dt);
        }

        if (m_is_grounded)
        {
            accelerate(wish_dir, wish_speed, k_ground_accel, dt);
            m_is_wall_running = false;
        }
        else
        {
            air_accelerate(wish_dir, wish_speed, k_air_accel, dt);

            bool wall_running = false;
            const bool can_wall_jump =
                    input.wallrun
                    && was_wall_running
                    && m_has_wall_contact
                    && m_wall_run_cooldown_timer <= 0.f
                    && std::abs(m_wall_contact_normal.y) <= k_wall_run_max_up_dot;
            const bool can_snap_to_wall = input.wallrun && jump_pressed && can_wall_run(wish_dir, wish_speed);
            if (can_wall_jump || can_snap_to_wall)
            {
                const auto tangent = wall_run_tangent(wish_dir);
                const float tangent_speed =
                        m_velocity.x * tangent.x +
                        m_velocity.z * tangent.z;
                const float run_speed = std::max(tangent_speed, k_wall_run_speed);

                if (jump_released && can_wall_jump)
                {
                    m_velocity.x = tangent.x * run_speed + m_wall_contact_normal.x * k_wall_jump_push_speed;
                    m_velocity.y = k_wall_jump_up_speed;
                    m_velocity.z = tangent.z * run_speed + m_wall_contact_normal.z * k_wall_jump_push_speed;
                    m_jump_buffer_timer = 0.f;
                    m_wall_run_cooldown_timer = k_wall_run_regrab_cooldown;
                }
                else if (jump_pressed)
                {
                    m_velocity.x = tangent.x * run_speed - m_wall_contact_normal.x * k_wall_run_stick_speed;
                    m_velocity.z = tangent.z * run_speed - m_wall_contact_normal.z * k_wall_run_stick_speed;
                    if (m_velocity.y < k_wall_run_max_fall_speed)
                        m_velocity.y = k_wall_run_max_fall_speed;
                    wall_running = true;
                }
            }

            const float gravity_scale = wall_running ? k_wall_run_gravity_scale : 1.f;
            m_velocity.y += k_gravity * gravity_scale * 0.5f * dt;
        }

        // --- Integrate position ---
        auto position = m_collider.get_origin();
        position.x += m_velocity.x * dt;
        position.y += m_velocity.y * dt;
        position.z += m_velocity.z * dt;

        // --- Collision resolution (multiple passes to handle simultaneous contacts) ---
        m_is_grounded = false;
        m_has_wall_contact = false;
        m_collider.set_origin(position);

        for (int i = 0; i < 5; i++)
            resolve_collisions(world);

        const bool post_wall_running = input.wallrun && jump_pressed && can_wall_run(wish_dir, wish_speed);
        m_is_wall_running = post_wall_running;
        if (!m_is_grounded)
            m_velocity.y += k_gravity * (post_wall_running ? k_wall_run_gravity_scale : 1.f) * 0.5f * dt;
        else if (m_velocity.y < 0.f)
            m_velocity.y = 0.f;
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

    void Player::air_accelerate(
            const omath::Vector3<float>& wish_dir,
            float wish_speed,
            float accel,
            float dt)
    {
        float wish_spd = wish_speed;
        if (wish_spd > k_air_speed_cap)
            wish_spd = k_air_speed_cap;

        const float current_speed = m_velocity.x * wish_dir.x + m_velocity.z * wish_dir.z;
        const float add_speed = wish_spd - current_speed;
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

    void Player::clip_velocity(const omath::Vector3<float>& normal)
    {
        const float backoff =
                m_velocity.x * normal.x +
                m_velocity.y * normal.y +
                m_velocity.z * normal.z;
        if (backoff >= 0.f)
            return;

        m_velocity.x -= normal.x * backoff;
        m_velocity.y -= normal.y * backoff;
        m_velocity.z -= normal.z * backoff;
    }

    bool Player::can_wall_run(const omath::Vector3<float>& wish_dir, float wish_speed) const
    {
        if (!m_has_wall_contact
            || wish_speed <= 0.f
            || m_wall_run_cooldown_timer > 0.f
            || std::abs(m_wall_contact_normal.y) > k_wall_run_max_up_dot)
        {
            return false;
        }

        const float away_input =
                wish_dir.x * m_wall_contact_normal.x +
                wish_dir.z * m_wall_contact_normal.z;
        return away_input < k_wall_run_max_away_input_dot;
    }

    omath::Vector3<float> Player::wall_run_tangent(const omath::Vector3<float>& wish_dir) const
    {
        omath::Vector3<float> tangent{
            m_wall_contact_normal.z,
            0.f,
            -m_wall_contact_normal.x
        };

        const float len_sq = tangent.x * tangent.x + tangent.z * tangent.z;
        if (len_sq > 1e-6f)
        {
            const float inv_len = 1.f / std::sqrt(len_sq);
            tangent.x *= inv_len;
            tangent.z *= inv_len;
        }

        const float wish_dot = tangent.x * wish_dir.x + tangent.z * wish_dir.z;
        const float velocity_dot = tangent.x * m_velocity.x + tangent.z * m_velocity.z;
        if ((std::abs(wish_dot) > 0.01f ? wish_dot : velocity_dot) < 0.f)
        {
            tangent.x = -tangent.x;
            tangent.z = -tangent.z;
        }

        return tangent;
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
            const float pv_len = pv.length();
            if (pv_len < 1e-6f)
                continue;

            const float inv_len = 1.f / pv_len;
            const omath::Vector3<float> normal = {
                pv.x * inv_len,
                pv.y * inv_len,
                pv.z * inv_len
            };

            m_collider.set_origin(m_collider.get_origin() + (pv * 1.005f));
            clip_velocity(normal);

            const float up_dot = normal.y;

            if (up_dot > m_floor_dot)
            {
                if (m_jump_ground_lockout_timer <= 0.f)
                {
                    m_is_grounded = true;
                    if (m_velocity.y < 0.f)
                        m_velocity.y = 0.f;
                }
            }
            else if (up_dot < -m_floor_dot)
            {
                if (m_velocity.y > 0.f)
                    m_velocity.y = 0.f;
            }
            else if (std::abs(up_dot) <= k_wall_run_max_up_dot)
            {
                m_has_wall_contact = true;
                m_wall_contact_normal = normal;
            }
        }
    }

    omath::Vector3<float> Player::get_eye_position() const
    {
        const auto position = m_collider.get_origin();
        return {position.x, position.y + k_eye_height, position.z};
    }

    void Player::set_floor_dot(float floor_dot)
    {
        if (floor_dot < 0.f)
            floor_dot = 0.f;
        else if (floor_dot > 1.f)
            floor_dot = 1.f;

        m_floor_dot = floor_dot;
    }

    const omath::opengl_engine::ViewAngles& Player::get_view_angles() const
    {
        return m_view_angles;
    }
} // namespace rose::core
