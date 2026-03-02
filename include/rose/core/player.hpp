//
// Created by orange on 26.02.2026.
//
#pragma once
#include "rose/core/collision_world.hpp"
#include <omath/engines/opengl_engine/constants.hpp>
#include <omath/linear_algebra/vector3.hpp>
#include <vector>

namespace rose::core
{
    struct PlayerInput final
    {
        bool  forward  = false;
        bool  backward = false;
        bool  left     = false;
        bool  right    = false;
        bool  jump     = false;
        float mouse_dx = 0.f;
        float mouse_dy = 0.f;
    };

    class Player final
    {
    public:
        // Box half-extents (centred on m_position) — 1 unit = 1 metre
        // Player capsule approximation: 0.5 m wide, 1.8 m tall
        static constexpr float k_half_width  = 1.25f;
        static constexpr float k_half_height = 1.9f;
        static constexpr float k_half_depth  = 1.25f;

        // Eye height above m_position centre (~1.65 m above feet)
        static constexpr float k_eye_height = 1.25f;

        static constexpr float k_move_speed       = 10.f;
        static constexpr float k_jump_speed        = 10.f;
        static constexpr float k_gravity           = -20.f;
        static constexpr float k_mouse_sensitivity = 0.1f;

        // dot(resolve_vec_normalised, up) threshold to count a surface as floor
        static constexpr float k_floor_dot = 0.65f;

        // Quake/Source bhop physics
        static constexpr float k_ground_accel = 250.f;  // snappy ground acceleration
        static constexpr float k_air_accel    = 4.f;   // air-strafe acceleration (enables bhop speed gain)
        static constexpr float k_friction     = 10.f;    // ground friction coefficient
        static constexpr float k_stop_speed   = 2.f;    // speed threshold for full-friction clamp

        explicit Player(const omath::Vector3<float>& position);

        void update(
            float dt,
            const CollisionWorld& world,
            const PlayerInput& input);

        [[nodiscard]] omath::Vector3<float>                   get_eye_position() const;
        [[nodiscard]] const omath::opengl_engine::ViewAngles& get_view_angles()  const;
        [[nodiscard]] bool                                    is_grounded()      const { return m_is_grounded; }

    private:
        omath::Vector3<float>           m_velocity{};
        bool                            m_is_grounded = false;
        omath::opengl_engine::ViewAngles m_view_angles{};

        // Convex box collider — vertices stored in local space, origin = m_position
        omath::collision::MeshCollider<omath::opengl_engine::Mesh> m_collider;

        void resolve_collisions(const CollisionWorld& world);

        void apply_friction(float dt);
        void accelerate(const omath::Vector3<float>& wish_dir, float wish_speed, float accel, float dt);

        // Reused scratch buffer for chunk query results — avoids per-call heap alloc.
        std::vector<int> m_query_buf;
    };
} // namespace rose::core
