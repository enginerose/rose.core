//
// Created by orange on 29.01.2026.
//

#include "rose/core/window_manager.hpp"
#include <boost/dll.hpp>
#include <filesystem>
#include <thread>
int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    rose::core::WindowManager window_manager;
    window_manager.run();
}
