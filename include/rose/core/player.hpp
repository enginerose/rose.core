//
// Created by orange on 26.02.2026.
//
#pragma once
#include <omath/collision/mesh_collider.hpp>
#include <omath/engines/opengl_engine/constants.hpp>
#include <omath/engines/opengl_engine/mesh.hpp>
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
        static constexpr float k_half_width  = 0.25f;
        static constexpr float k_half_height = 0.9f;
        static constexpr float k_half_depth  = 0.25f;

        // Eye height above m_position centre (~1.65 m above feet)
        static constexpr float k_eye_height = 0.75f;

        static constexpr float k_move_speed       = 5.f;
        static constexpr float k_jump_speed        = 5.f;
        static constexpr float k_gravity           = -20.f;
        static constexpr float k_mouse_sensitivity = 0.1f;

        // dot(resolve_vec_normalised, up) threshold to count a surface as floor
        static constexpr float k_floor_dot = 0.65f;

        explicit Player(const omath::Vector3<float>& position);

        void update(
            float dt,
            const std::vector<omath::collision::MeshCollider<omath::opengl_engine::Mesh>>& map_colliders,
            const PlayerInput& input);

        [[nodiscard]] omath::Vector3<float>                   get_eye_position() const;
        [[nodiscard]] const omath::opengl_engine::ViewAngles& get_view_angles()  const;
        [[nodiscard]] bool                                    is_grounded()      const { return m_is_grounded; }

    private:
        omath::Vector3<float>           m_position;
        omath::Vector3<float>           m_velocity{};
        bool                            m_is_grounded = false;
        omath::opengl_engine::ViewAngles m_view_angles{};

        // Convex box collider — vertices stored in local space, origin = m_position
        omath::collision::MeshCollider<omath::opengl_engine::Mesh> m_collider;

        void resolve_collisions(
            const std::vector<omath::collision::MeshCollider<omath::opengl_engine::Mesh>>& map_colliders);
    };
} // namespace rose::core
