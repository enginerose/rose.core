//
// Created by orange on 16.02.2026.
//
//
// Created by orange on 04.03.2026.
//

#pragma once
#include <boost/config.hpp>
#include <string>
#include <vector>
#include <optional>

struct MousePose final
{
    std::uint16_t x;
    std::uint16_t y;
};


enum class MouseButtonState : std::uint8_t
{
    None = 0,
    LeftButton = 1,
    RightButton = 2,
    Both = 3
};

struct MouseInputCommand final
{
    std::uint32_t tick;
    MousePose mouse_position;
    MouseButtonState mouse_buttons;
    std::int16_t wheel_scroll;
    std::uint64_t timestamp;
};

class BOOST_SYMBOL_VISIBLE StreamPluginApi
{
public:
    [[nodiscard]]
    virtual bool is_ready_to_stream() const = 0;

    virtual void run() = 0;

    [[nodiscard]]
    virtual int get_frame_queue_size() const = 0;

    virtual void push_frame(const std::vector<std::byte>& frame) = 0;

    [[nodiscard]]
    virtual std::optional<MouseInputCommand> maybe_get_mouse_input() = 0;
    virtual ~StreamPluginApi() = default;
};