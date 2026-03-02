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
    Player::Player(const omath::Vector3<float>& position): m_collider(make_box_collider())
    {
        m_collider.set_origin(position);
    }

    void Player::update(
            float dt,
            const CollisionWorld& world,
            const PlayerInput& input
    )
    {
        // --- Mouse look ---
        auto angles = m_view_angles;
        angles.yaw -= decltype(angles.yaw)::from_degrees(input.mouse_dx * k_mouse_sensitivity);
        angles.pitch -= decltype(angles.pitch)::from_degrees(input.mouse_dy * k_mouse_sensitivity);
        m_view_angles = angles;

        // --- Build wish direction from input (ignores pitch, FPS style) ---
        const float yaw_rad = m_view_angles.yaw.as_radians();
        const omath::Vector3<float> forward = {-std::sin(yaw_rad), 0.f, -std::cos(yaw_rad)};
        const omath::Vector3<float> right   = {std::cos(yaw_rad),  0.f, -std::sin(yaw_rad)};

        omath::Vector3<float> wish_dir = {0.f, 0.f, 0.f};
        if (input.forward)  wish_dir = wish_dir + forward;
        if (input.backward) wish_dir = wish_dir - forward;
        if (input.right)    wish_dir = wish_dir + right;
        if (input.left)     wish_dir = wish_dir - right;

        // Normalise wish direction (horizontal only)
        const float wish_len_sq = wish_dir.x * wish_dir.x + wish_dir.z * wish_dir.z;
        if (wish_len_sq > 1e-6f)
        {
            const float inv = 1.f / std::sqrt(wish_len_sq);
            wish_dir.x *= inv;
            wish_dir.z *= inv;
        }

        // --- Quake/Source velocity-based horizontal movement ---
        // On ground: friction first, then high-accel ground movement (capped at k_move_speed).
        // In air:    low accel with no per-frame speed cap — strafing while airborne
        //            accumulates speed (the bhop mechanic).
        if (m_is_grounded)
        {
            apply_friction(dt);
            accelerate(wish_dir, k_move_speed, k_ground_accel, dt);
        }
        else
        {
            accelerate(wish_dir, k_move_speed, k_air_accel, dt);
        }

        auto position = m_collider.get_origin();

        position.x += m_velocity.x * dt;
        position.z += m_velocity.z * dt;

        // --- Jump & gravity ---
        if (input.jump && m_is_grounded)
        {
            m_velocity.y = k_jump_speed;
            m_is_grounded = false;
        }
        if (!m_is_grounded)
            m_velocity.y += k_gravity * dt;

        position.y += m_velocity.y * dt;

        // --- Collision resolution (multiple passes to handle simultaneous contacts) ---
        m_is_grounded = false;
        m_collider.set_origin(position);

        for (int i = 0; i < 5; i++)
            resolve_collisions(world);
    }

    // ---------------------------------------------------------------------------
    // resolve_collisions — two-layer broadphase before GJK + EPA:
    //   1. Chunk grid  — query only colliders whose chunks overlap the player AABB.
    //   2. AABB check  — fast axis-aligned overlap test against each candidate.
    //   3. GJK         — exact convex-shape intersection test.
    //   4. EPA         — penetration vector for depenetration.
    // ---------------------------------------------------------------------------
    void Player::resolve_collisions(const CollisionWorld& world)
    {
        const auto pos = m_collider.get_origin();

        // Player world-space AABB (exact, since the player collider is an AABB).
        const Aabb player_aabb{
            {pos.x - k_half_width, pos.y - k_half_height, pos.z - k_half_depth},
            {pos.x + k_half_width, pos.y + k_half_height, pos.z + k_half_depth}
        };

        // --- Layer 1: chunk query (reuses m_query_buf to avoid per-frame alloc) ---
        m_query_buf.clear();
        world.query(player_aabb, m_query_buf);

        for (const int idx : m_query_buf)
        {
            // --- Layer 2: AABB vs AABB ---
            if (!player_aabb.overlaps(world.aabbs[idx]))
                continue;

            const auto& map_col = world.colliders[idx];

            // --- Layer 3: GJK broad check ---
            const auto hit = Gjk::is_collide_with_simplex_info(map_col, m_collider);
            if (!hit.hit)
                continue;

            // --- Layer 4: EPA for penetration vector ---
            const auto result = Epa::solve(map_col, m_collider, hit.simplex);
            if (!result)
                continue;

            m_collider.set_origin(m_collider.get_origin() + (result->penetration_vector * 1.005));
            const auto up_dot = result->penetration_vector.y / result->penetration_vector.length();

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

    // ---------------------------------------------------------------------------
    // Quake-style friction: decelerate horizontal velocity when on the ground.
    // Uses a "control" speed floor (k_stop_speed) so very slow movement still
    // bleeds off cleanly instead of hanging at near-zero speed forever.
    // ---------------------------------------------------------------------------
    void Player::apply_friction(const float dt)
    {
        const float speed = std::sqrt(m_velocity.x * m_velocity.x + m_velocity.z * m_velocity.z);
        if (speed < 1e-6f)
            return;

        const float control   = (speed < k_stop_speed) ? k_stop_speed : speed;
        const float drop      = control * k_friction * dt;
        const float new_speed = std::max(speed - drop, 0.f);
        const float scale     = new_speed / speed;
        m_velocity.x *= scale;
        m_velocity.z *= scale;
    }

    // ---------------------------------------------------------------------------
    // Quake/Source accelerate: add velocity toward wish_dir without exceeding
    // wish_speed in that direction.  Using a low accel value in air (k_air_accel)
    // lets the player steer their momentum while airborne — strafing left/right
    // while looking slightly off the velocity vector adds a tiny push each frame,
    // accumulating speed across hops (bunny-hopping).
    // ---------------------------------------------------------------------------
    void Player::accelerate(
        const omath::Vector3<float>& wish_dir,
        const float                  wish_speed,
        const float                  accel,
        const float                  dt)
    {
        // How fast we're already moving in the desired direction
        const float current_speed = m_velocity.x * wish_dir.x + m_velocity.z * wish_dir.z;
        const float add_speed     = wish_speed - current_speed;
        if (add_speed <= 0.f)
            return;

        float accel_speed = accel * wish_speed * dt;
        if (accel_speed > add_speed)
            accel_speed = add_speed;

        m_velocity.x += accel_speed * wish_dir.x;
        m_velocity.z += accel_speed * wish_dir.z;
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
