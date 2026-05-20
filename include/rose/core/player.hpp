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
        bool  forward   = false;
        bool  backward  = false;
        bool  left      = false;
        bool  right     = false;
        bool  jump      = false;
        bool  auto_bhop = false;
        bool  wallrun   = false;
        bool  noclip    = false;
        float mouse_dx  = 0.f;
        float mouse_dy  = 0.f;
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

        // Source movement cvars converted from Source inches to engine metres.
        static constexpr float k_source_unit_to_meter = 0.0254f;
        static constexpr float k_max_speed            = 320.f * k_source_unit_to_meter;
        static constexpr float k_ground_accel         = 10.f;
        static constexpr float k_air_accel            = 10.f;
        static constexpr float k_air_speed_cap        = 30.f * k_source_unit_to_meter;
        static constexpr float k_friction             = 4.f;
        static constexpr float k_stop_speed           = 100.f * k_source_unit_to_meter;
        static constexpr float k_jump_speed           = 268.3281573f * k_source_unit_to_meter;
        static constexpr float k_gravity              = -800.f * k_source_unit_to_meter;
        static constexpr float k_jump_buffer_time     = 0.1f;
        static constexpr float k_ground_grace_time    = 0.05f;
        static constexpr float k_jump_ground_lockout_time = 0.05f;
        static constexpr float k_wall_run_speed       = k_max_speed * 1.15f;
        static constexpr float k_wall_run_stick_speed = 2.0f;
        static constexpr float k_wall_run_gravity_scale = 0.25f;
        static constexpr float k_wall_run_max_fall_speed = -2.0f;
        static constexpr float k_wall_run_max_up_dot = 0.25f;
        static constexpr float k_wall_run_max_away_input_dot = 0.65f;
        static constexpr float k_wall_jump_push_speed = k_max_speed * 0.75f;
        static constexpr float k_wall_jump_up_speed = k_jump_speed * 0.95f;
        static constexpr float k_wall_run_regrab_cooldown = 0.2f;
        static constexpr float k_mouse_sensitivity = 0.1f;

        // dot(resolve_vec_normalised, up) threshold to count a surface as floor
        static constexpr float k_floor_dot = 0.65f;

        explicit Player(const omath::Vector3<float>& position);

        void update(
            float dt,
            const CollisionWorld& world,
            const PlayerInput& input);

        [[nodiscard]] omath::Vector3<float>                   get_eye_position() const;
        [[nodiscard]] const omath::opengl_engine::ViewAngles& get_view_angles()  const;
        [[nodiscard]] bool                                    is_grounded()      const { return m_is_grounded; }
        void set_floor_dot(float floor_dot);
        [[nodiscard]] float floor_dot() const { return m_floor_dot; }

    private:
        omath::Vector3<float>           m_velocity{};
        bool                            m_is_grounded  = false;
        bool                            m_jump_was_pressed = false;
        bool                            m_noclip       = false;
        bool                            m_noclip_was_pressed = false;
        float                           m_jump_buffer_timer = 0.f;
        float                           m_ground_grace_timer = 0.f;
        float                           m_jump_ground_lockout_timer = 0.f;
        float                           m_wall_run_cooldown_timer = 0.f;
        float                           m_floor_dot = k_floor_dot;
        bool                            m_has_wall_contact = false;
        bool                            m_is_wall_running = false;
        omath::Vector3<float>           m_wall_contact_normal{};
        omath::opengl_engine::ViewAngles m_view_angles{};
        float m_smooth_dx = 0.f;
        float m_smooth_dy = 0.f;

        // Convex box collider — vertices stored in local space, origin = m_position
        omath::collision::MeshCollider<omath::opengl_engine::Mesh> m_collider;

        void resolve_collisions(const CollisionWorld& world);
        void accelerate(const omath::Vector3<float>& wish_dir, float wish_speed, float accel, float dt);
        void air_accelerate(const omath::Vector3<float>& wish_dir, float wish_speed, float accel, float dt);
        void apply_friction(float dt);
        void clip_velocity(const omath::Vector3<float>& normal);
        [[nodiscard]] bool can_wall_run(const omath::Vector3<float>& wish_dir, float wish_speed) const;
        [[nodiscard]] omath::Vector3<float> wall_run_tangent(const omath::Vector3<float>& wish_dir) const;

        // Reused scratch buffer — grown to capacity once, cleared each pass.
        std::vector<int> m_query_buf;
    };
} // namespace rose::core
