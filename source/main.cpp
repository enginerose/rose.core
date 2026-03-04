//
// Created by orange on 29.01.2026.
//
#include "/home/orange/CLionProjects/rose.stream_plugin/include/cherry_streamer/stream_plugin_api.hpp"
#include "rose/core/window_manager.hpp"
#include <boost/dll.hpp>
#include <filesystem>
#include <thread>
int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    const boost::dll::fs::path lib_path(
            "/home/orange/CLionProjects/rose.core/plugins/rose.stream.so"
    ); // argv[1] contains path to directory with our plugin library
    using pluginapi_create_t = std::shared_ptr<StreamPluginApi>();
    auto creator = boost::dll::import_alias<pluginapi_create_t>(        // type of imported symbol must be explicitly specified
        lib_path,                                            // path to library
        "create_plugin",                                                // symbol to import
        boost::dll::load_mode::default_mode                              // do append extensions and prefixes
    );
    std::shared_ptr<StreamPluginApi> plugin = creator();
    plugin->run();
    rose::core::WindowManager window_manager;
    window_manager.run();
}
