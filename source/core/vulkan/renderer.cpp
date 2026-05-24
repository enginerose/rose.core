//
// Created by orange on 15.05.2026.
//
#include "rose/core/vulkan/renderer.hpp"

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>
#ifdef ROSE_ENABLE_NGX_DLSS
#include <nvsdk_ngx_helpers.h>
#include <nvsdk_ngx_helpers_vk.h>
#endif
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>
#include <spdlog/spdlog.h>

#ifdef _WIN32
#include <process.h>
#endif

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unordered_map>
#include <vector>

namespace rose::core::vulkan
{
    namespace
    {
        constexpr uint32_t k_api_version          = VK_API_VERSION_1_2;
        constexpr int      k_max_frames_in_flight = 2;

        class VulkanError final : public std::runtime_error
        {
        public:
            explicit VulkanError(const std::string& message)
                : std::runtime_error(message)
            {}
        };

        void check_vk(VkResult result, const char* message)
        {
            if (result != VK_SUCCESS)
                throw VulkanError(std::string(message) + " (VkResult " + std::to_string(static_cast<int>(result)) + ")");
        }

        void check_imgui_vk_result(VkResult result)
        {
            if (result != VK_SUCCESS)
            {
                spdlog::critical("ImGui Vulkan backend failed with VkResult {}", static_cast<int>(result));
                std::terminate();
            }
        }

        [[nodiscard]] std::optional<std::string> environment_value(const char* name)
        {
#ifdef _MSC_VER
            char* value = nullptr;
            std::size_t value_size = 0;
            if (_dupenv_s(&value, &value_size, name) != 0 || value == nullptr)
                return std::nullopt;
            std::string result(value);
            std::free(value);
            if (result.empty())
                return std::nullopt;
            return result;
#else
            const char* value = std::getenv(name);
            if (value == nullptr || value[0] == '\0')
                return std::nullopt;
            return std::string(value);
#endif
        }

        [[nodiscard]] std::string quote_command_arg(const std::filesystem::path& path)
        {
            std::string result = "\"";
            for (const char c : path.string())
            {
                if (c == '"')
                    result += "\\\"";
                else
                    result += c;
            }
            result += "\"";
            return result;
        }

        [[nodiscard]] int run_glslc(const std::filesystem::path& compiler_path,
                                    const std::filesystem::path& source_path,
                                    const std::filesystem::path& output_path)
        {
#ifdef _WIN32
            const std::wstring compiler = compiler_path.wstring();
            const std::wstring target_env = L"--target-env=vulkan1.2";
            const std::wstring source = source_path.wstring();
            const std::wstring output_flag = L"-o";
            const std::wstring output = output_path.wstring();
            const wchar_t* args[] = {
                compiler.c_str(),
                target_env.c_str(),
                source.c_str(),
                output_flag.c_str(),
                output.c_str(),
                nullptr
            };
            return static_cast<int>(_wspawnvp(_P_WAIT, compiler.c_str(), args));
#else
            const std::string command = quote_command_arg(compiler_path)
                                      + " --target-env=vulkan1.2 "
                                      + quote_command_arg(source_path)
                                      + " -o "
                                      + quote_command_arg(output_path);
            return std::system(command.c_str());
#endif
        }

        [[nodiscard]] std::filesystem::path glslc_path()
        {
#ifdef _WIN32
            constexpr const char* glslc_filename = "glslc.exe";
#else
            constexpr const char* glslc_filename = "glslc";
#endif
            std::error_code ec;
            const std::filesystem::path local_glslc = std::filesystem::current_path(ec) / glslc_filename;
            if (!ec && std::filesystem::exists(local_glslc))
                return local_glslc;

            if (const auto vulkan_sdk = environment_value("VULKAN_SDK"))
            {
#ifdef _WIN32
                const std::filesystem::path sdk_glslc = std::filesystem::path(*vulkan_sdk) / "Bin" / glslc_filename;
#else
                const std::filesystem::path sdk_glslc = std::filesystem::path(*vulkan_sdk) / "bin" / glslc_filename;
#endif
                if (std::filesystem::exists(sdk_glslc))
                    return sdk_glslc;
            }

            return glslc_filename;
        }

        [[nodiscard]] std::vector<char> read_binary_file(const std::filesystem::path& path)
        {
            spdlog::info("Vulkan: loading shader '{}'", path.string());
            std::ifstream file(path, std::ios::ate | std::ios::binary);
            if (!file)
            {
                spdlog::critical("Vulkan: failed to open shader '{}'", path.string());
                throw VulkanError("Failed to open shader file: " + path.string());
            }

            const auto end = file.tellg();
            if (end <= 0)
            {
                spdlog::critical("Vulkan: shader file is empty '{}'", path.string());
                throw VulkanError("Shader file is empty: " + path.string());
            }

            std::vector<char> buffer(static_cast<std::size_t>(end));
            file.seekg(0, std::ios::beg);
            file.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
            return buffer;
        }

        [[nodiscard]] std::filesystem::path shader_path(const char* filename)
        {
            const std::filesystem::path relative_path = std::filesystem::path("shaders") / filename;
            std::error_code ec;
            const std::filesystem::path cwd_path = std::filesystem::current_path(ec) / relative_path;
            if (!ec && std::filesystem::exists(cwd_path))
                return cwd_path;
            if (std::filesystem::exists(relative_path))
                return relative_path;

            std::optional<std::filesystem::path> build_path;
#ifdef ROSE_SHADER_DIR
            build_path = std::filesystem::path(ROSE_SHADER_DIR) / filename;
#endif

            std::filesystem::path source_filename = filename;
            if (source_filename.extension() == ".spv")
                source_filename.replace_extension();

            std::vector<std::filesystem::path> source_candidates;
            if (!ec)
                source_candidates.push_back(std::filesystem::current_path(ec) / "shaders" / source_filename);
            source_candidates.push_back(std::filesystem::path("shaders") / source_filename);
#ifdef ROSE_SHADER_SOURCE_DIR
            source_candidates.push_back(std::filesystem::path(ROSE_SHADER_SOURCE_DIR) / source_filename);
#endif

            std::optional<std::filesystem::path> source_path;
            for (const std::filesystem::path& candidate : source_candidates)
            {
                if (std::filesystem::exists(candidate))
                {
                    source_path = candidate;
                    break;
                }
            }

            const std::filesystem::path output_path = !ec ? cwd_path : relative_path;
            if (!source_path)
            {
                if (build_path && std::filesystem::exists(*build_path))
                    return *build_path;

                spdlog::critical("Vulkan: shader binary '{}' is missing and GLSL source '{}' was not found",
                                 output_path.string(),
                                 source_filename.string());
                return output_path;
            }

            if (const std::filesystem::path parent_path = output_path.parent_path(); !parent_path.empty())
                std::filesystem::create_directories(parent_path, ec);

            const std::filesystem::path compiler_path = glslc_path();
            spdlog::warn("Vulkan: shader binary '{}' is missing, compiling '{}' with '{}'",
                         output_path.string(),
                         source_path->string(),
                         compiler_path.string());
            const int result = run_glslc(compiler_path, *source_path, output_path);
            if (result != 0 || !std::filesystem::exists(output_path))
            {
                spdlog::critical("Vulkan: failed to compile shader '{}' to '{}' using '{}'",
                                 source_path->string(),
                                 output_path.string(),
                                 compiler_path.string());
                if (build_path && std::filesystem::exists(*build_path))
                    return *build_path;
            }

            return output_path;
        }

        struct QueueFamilies final
        {
            std::optional<uint32_t> graphics;
            std::optional<uint32_t> present;

            [[nodiscard]] bool complete() const noexcept
            {
                return graphics.has_value() && present.has_value();
            }
        };

        struct SwapchainSupport final
        {
            VkSurfaceCapabilitiesKHR capabilities{};
            std::vector<VkSurfaceFormatKHR> formats;
            std::vector<VkPresentModeKHR> present_modes;
        };

        struct BufferResource final
        {
            VkBuffer buffer = VK_NULL_HANDLE;
            VkDeviceMemory memory = VK_NULL_HANDLE;
            VkDeviceSize size = 0;
        };

        struct ImageResource final
        {
            VkImage image = VK_NULL_HANDLE;
            VkDeviceMemory memory = VK_NULL_HANDLE;
            VkImageView view = VK_NULL_HANDLE;
            VkFormat format = VK_FORMAT_UNDEFINED;
            VkExtent2D extent{};
            VkImageLayout layout = VK_IMAGE_LAYOUT_UNDEFINED;
        };

        struct GpuTexture final
        {
            ImageResource image;
            VkSampler sampler = VK_NULL_HANDLE;
            VkDescriptorSet descriptor = VK_NULL_HANDLE;
        };

        struct GpuMesh final
        {
            BufferResource vertex_buffer;
            BufferResource index_buffer;
            uint32_t index_count = 0;
            VkDescriptorSet descriptor = VK_NULL_HANDLE;
            omath::Vector3<float> local_center{};
        };

        struct FrameSync final
        {
            VkSemaphore image_available = VK_NULL_HANDLE;
            VkSemaphore render_finished = VK_NULL_HANDLE;
            VkFence in_flight = VK_NULL_HANDLE;
        };

        struct ReadbackSlot final
        {
            BufferResource buffer;
            VkExtent2D extent{};
            VkFormat format = VK_FORMAT_UNDEFINED;
            bool pending = false;
        };

        struct PushConstants final
        {
            float view_projection[16]{};
            float model[16]{};
            float previous_view_projection[16]{};
            float outline_center[3]{};
            float outline_width = 0.0f;
            float outline_color[3]{};
            float outline_alpha = 0.0f;
            int32_t outline_enabled = 0;
            float outline_padding[3]{};
        };

        struct BloomPushConstants final
        {
            float texel_size[2]{};
            float threshold = 1.0f;
            float intensity = 0.0f;
            float radius = 1.0f;
            int32_t quality = 1;
        };

        [[nodiscard]] const char* dlss_quality_label(DlssQuality quality) noexcept
        {
            switch (quality)
            {
            case DlssQuality::Quality:
                return "Quality";
            case DlssQuality::Balanced:
                return "Balanced";
            case DlssQuality::Performance:
                return "Performance";
            case DlssQuality::UltraPerformance:
                return "Ultra Performance";
            case DlssQuality::UltraQuality:
                return "Ultra Quality";
            case DlssQuality::Dlaa:
                return "DLAA";
            }
            return "Unknown";
        }

#ifdef ROSE_ENABLE_NGX_DLSS
        [[nodiscard]] NVSDK_NGX_PerfQuality_Value to_ngx_quality(DlssQuality quality) noexcept
        {
            switch (quality)
            {
            case DlssQuality::Quality:
                return NVSDK_NGX_PerfQuality_Value_MaxQuality;
            case DlssQuality::Balanced:
                return NVSDK_NGX_PerfQuality_Value_Balanced;
            case DlssQuality::Performance:
                return NVSDK_NGX_PerfQuality_Value_MaxPerf;
            case DlssQuality::UltraPerformance:
                return NVSDK_NGX_PerfQuality_Value_UltraPerformance;
            case DlssQuality::UltraQuality:
                return NVSDK_NGX_PerfQuality_Value_UltraQuality;
            case DlssQuality::Dlaa:
                return NVSDK_NGX_PerfQuality_Value_DLAA;
            }
            return NVSDK_NGX_PerfQuality_Value_MaxQuality;
        }

        [[nodiscard]] const char* ngx_result_name(NVSDK_NGX_Result result) noexcept
        {
            switch (result)
            {
            case NVSDK_NGX_Result_Success:
                return "success";
            case NVSDK_NGX_Result_FAIL_FeatureNotSupported:
                return "feature not supported";
            case NVSDK_NGX_Result_FAIL_PlatformError:
                return "platform error";
            case NVSDK_NGX_Result_FAIL_InvalidParameter:
                return "invalid parameter";
            case NVSDK_NGX_Result_FAIL_OutOfDate:
                return "out of date";
            case NVSDK_NGX_Result_FAIL_UnableToInitializeFeature:
                return "unable to initialize feature";
            case NVSDK_NGX_Result_FAIL_UnsupportedInputFormat:
                return "unsupported input format";
            case NVSDK_NGX_Result_FAIL_UnsupportedFormat:
                return "unsupported format";
            default:
                return "failed";
            }
        }
#endif

        [[nodiscard]] bool is_bgra_format(VkFormat format) noexcept
        {
            return format == VK_FORMAT_B8G8R8A8_SRGB || format == VK_FORMAT_B8G8R8A8_UNORM;
        }

        [[nodiscard]] bool is_rgba_format(VkFormat format) noexcept
        {
            return format == VK_FORMAT_R8G8B8A8_SRGB || format == VK_FORMAT_R8G8B8A8_UNORM;
        }

        [[nodiscard]] const char* vk_format_name(VkFormat format) noexcept
        {
            switch (format)
            {
            case VK_FORMAT_B8G8R8A8_SRGB:
                return "VK_FORMAT_B8G8R8A8_SRGB";
            case VK_FORMAT_B8G8R8A8_UNORM:
                return "VK_FORMAT_B8G8R8A8_UNORM";
            case VK_FORMAT_R8G8B8A8_SRGB:
                return "VK_FORMAT_R8G8B8A8_SRGB";
            case VK_FORMAT_R8G8B8A8_UNORM:
                return "VK_FORMAT_R8G8B8A8_UNORM";
            case VK_FORMAT_R16G16_SFLOAT:
                return "VK_FORMAT_R16G16_SFLOAT";
            case VK_FORMAT_R32G32_SFLOAT:
                return "VK_FORMAT_R32G32_SFLOAT";
            case VK_FORMAT_D32_SFLOAT:
                return "VK_FORMAT_D32_SFLOAT";
            case VK_FORMAT_D32_SFLOAT_S8_UINT:
                return "VK_FORMAT_D32_SFLOAT_S8_UINT";
            case VK_FORMAT_D24_UNORM_S8_UINT:
                return "VK_FORMAT_D24_UNORM_S8_UINT";
            default:
                return "VK_FORMAT_UNKNOWN";
            }
        }

        [[nodiscard]] const char* present_mode_name(VkPresentModeKHR mode) noexcept
        {
            switch (mode)
            {
            case VK_PRESENT_MODE_IMMEDIATE_KHR:
                return "VK_PRESENT_MODE_IMMEDIATE_KHR";
            case VK_PRESENT_MODE_MAILBOX_KHR:
                return "VK_PRESENT_MODE_MAILBOX_KHR";
            case VK_PRESENT_MODE_FIFO_KHR:
                return "VK_PRESENT_MODE_FIFO_KHR";
            case VK_PRESENT_MODE_FIFO_RELAXED_KHR:
                return "VK_PRESENT_MODE_FIFO_RELAXED_KHR";
            default:
                return "VK_PRESENT_MODE_UNKNOWN";
            }
        }

        [[nodiscard]] const char* physical_device_type_name(VkPhysicalDeviceType type) noexcept
        {
            switch (type)
            {
            case VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU:
                return "integrated GPU";
            case VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU:
                return "discrete GPU";
            case VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU:
                return "virtual GPU";
            case VK_PHYSICAL_DEVICE_TYPE_CPU:
                return "CPU";
            default:
                return "other";
            }
        }

        [[nodiscard]] std::string vulkan_version_string(uint32_t version)
        {
            return std::to_string(VK_VERSION_MAJOR(version)) + "."
                 + std::to_string(VK_VERSION_MINOR(version)) + "."
                 + std::to_string(VK_VERSION_PATCH(version));
        }

        [[nodiscard]] std::string join_names(const std::vector<std::string>& names)
        {
            std::string result;
            for (const std::string& name : names)
            {
                if (!result.empty())
                    result += ", ";
                result += name;
            }
            return result;
        }

        [[nodiscard]] bool environment_flag_enabled(const char* name) noexcept
        {
#ifdef _MSC_VER
            char* value = nullptr;
            std::size_t value_size = 0;
            if (_dupenv_s(&value, &value_size, name) != 0 || value == nullptr)
                return false;
            const bool enabled = value[0] != '\0'
                && std::strcmp(value, "0") != 0
                && std::strcmp(value, "false") != 0
                && std::strcmp(value, "FALSE") != 0
                && std::strcmp(value, "off") != 0
                && std::strcmp(value, "OFF") != 0;
            std::free(value);
            return enabled;
#else
            const char* value = std::getenv(name);
            if (value == nullptr || value[0] == '\0')
                return false;
            const bool enabled = std::strcmp(value, "0") != 0
                && std::strcmp(value, "false") != 0
                && std::strcmp(value, "FALSE") != 0
                && std::strcmp(value, "off") != 0
                && std::strcmp(value, "OFF") != 0;
            return enabled;
#endif
        }
    } // namespace

    struct Renderer::Impl final
    {
        GLFWwindow* m_window = nullptr;

        VkInstance m_instance = VK_NULL_HANDLE;
        VkSurfaceKHR m_surface = VK_NULL_HANDLE;
        VkPhysicalDevice m_physical_device = VK_NULL_HANDLE;
        VkDevice m_device = VK_NULL_HANDLE;
        VkQueue m_graphics_queue = VK_NULL_HANDLE;
        VkQueue m_present_queue = VK_NULL_HANDLE;
        uint32_t m_graphics_queue_family = 0;
        uint32_t m_present_queue_family = 0;

        VkSwapchainKHR m_swapchain = VK_NULL_HANDLE;
        std::vector<VkImage> m_swapchain_images;
        std::vector<VkImageView> m_swapchain_image_views;
        VkFormat m_swapchain_image_format = VK_FORMAT_UNDEFINED;
        VkExtent2D m_swapchain_extent{};
        uint32_t m_min_image_count = 2;
        bool m_swapchain_supports_transfer_src = false;
        bool m_swapchain_supports_transfer_dst = false;

        VkRenderPass m_render_pass = VK_NULL_HANDLE;
        VkRenderPass m_present_render_pass = VK_NULL_HANDLE;
        VkDescriptorSetLayout m_descriptor_set_layout = VK_NULL_HANDLE;
        VkDescriptorSetLayout m_post_descriptor_set_layout = VK_NULL_HANDLE;
        VkPipelineLayout m_pipeline_layout = VK_NULL_HANDLE;
        VkPipelineLayout m_bloom_pipeline_layout = VK_NULL_HANDLE;
        VkPipeline m_graphics_pipeline = VK_NULL_HANDLE;
        VkPipeline m_outline_pipeline = VK_NULL_HANDLE;
        VkPipeline m_bloom_pipeline = VK_NULL_HANDLE;
        VkDescriptorPool m_descriptor_pool = VK_NULL_HANDLE;
        VkDescriptorSet m_bloom_descriptor_set = VK_NULL_HANDLE;
        VkImageView m_bloom_descriptor_image_view = VK_NULL_HANDLE;
        VkSampler m_bloom_sampler = VK_NULL_HANDLE;

        VkCommandPool m_command_pool = VK_NULL_HANDLE;
        std::vector<VkCommandBuffer> m_command_buffers;
        std::vector<VkFramebuffer> m_framebuffers;
        VkFramebuffer m_scene_framebuffer = VK_NULL_HANDLE;
        ImageResource m_scene_color_image;
        ImageResource m_motion_vector_image;
        ImageResource m_dlss_output_image;
        ImageResource m_depth_image;
        VkFormat m_depth_format = VK_FORMAT_UNDEFINED;
        VkFormat m_scene_color_format = VK_FORMAT_R8G8B8A8_UNORM;
        VkFormat m_motion_vector_format = VK_FORMAT_R16G16_SFLOAT;
        VkExtent2D m_scene_extent{};

        std::array<FrameSync, k_max_frames_in_flight> m_frames{};
        std::array<ReadbackSlot, k_max_frames_in_flight> m_readback_slots{};
        std::vector<VkFence> m_images_in_flight;
        std::size_t m_current_frame = 0;
        uint32_t m_active_image_index = 0;
        VkCommandBuffer m_active_command_buffer = VK_NULL_HANDLE;
        bool m_frame_started = false;
        bool m_scene_render_pass_active = false;
        bool m_present_render_pass_active = false;

        std::unordered_map<const Texture*, GpuTexture> m_texture_resources;
        std::unordered_map<const Mesh*, GpuMesh> m_mesh_resources;
        GpuTexture m_default_texture;
        bool m_default_texture_created = false;

        std::array<float, 16> m_previous_view_projection{};
        std::array<float, 16> m_frame_view_projection{};
        bool m_previous_view_projection_valid = false;
        bool m_frame_view_projection_set = false;
        std::optional<CapturedFrame> m_completed_stream_frame;

        bool m_dlss_requested = false;
        DlssQuality m_dlss_quality = DlssQuality::Quality;
        std::string m_dlss_status = "DLSS is disabled";
        float m_dlss_sharpness = 0.0f;
        bool m_dlss_reset_next_frame = true;
        SelectionOutlineSettings m_selection_outline_settings{};
        BloomSettings m_bloom_settings{};

        std::vector<std::string> m_ngx_instance_extensions;
        std::vector<std::string> m_ngx_device_extensions;
        bool m_ngx_extensions_available = true;

#ifdef ROSE_ENABLE_NGX_DLSS
        bool m_ngx_init_attempted = false;
        bool m_ngx_disabled_by_env = false;
        bool m_ngx_initialized = false;
        bool m_ngx_available = false;
        NVSDK_NGX_Parameter* m_ngx_parameters = nullptr;
        NVSDK_NGX_Handle* m_ngx_dlss_handle = nullptr;
#endif

        Impl(GLFWwindow* window, const omath::Vector2<int>&)
            : m_window(window)
        {
            spdlog::flush_on(spdlog::level::info);
            spdlog::info("Vulkan: renderer initialization started");
            create_instance();
            create_surface();
            pick_physical_device();
            create_logical_device();
            create_command_pool();
            create_descriptor_pool();
#ifdef ROSE_ENABLE_NGX_DLSS
            m_ngx_disabled_by_env = environment_flag_enabled("ROSE_DISABLE_NGX_DLSS");
            if (m_ngx_disabled_by_env)
            {
                m_dlss_status = "DLSS disabled by ROSE_DISABLE_NGX_DLSS";
                spdlog::info("Vulkan: {}", m_dlss_status);
            }
            else if (m_ngx_extensions_available)
            {
                m_dlss_status = "DLSS is off (enable to initialize)";
                spdlog::info("Vulkan: {}", m_dlss_status);
            }
#endif
            create_swapchain();
            create_image_views();
            create_render_pass();
            create_descriptor_set_layout();
            create_graphics_pipeline();
            create_render_targets();
            create_framebuffers();
            create_command_buffers();
            create_sync_objects();
            init_imgui();
            spdlog::info("Vulkan: renderer initialization completed");
        }

        ~Impl()
        {
            if (m_device != VK_NULL_HANDLE)
                vkDeviceWaitIdle(m_device);

            ImGui_ImplVulkan_Shutdown();
            ImGui_ImplGlfw_Shutdown();

            destroy_gpu_resources();
            cleanup_swapchain();
            shutdown_dlss_sdk();

            if (m_graphics_pipeline != VK_NULL_HANDLE)
                vkDestroyPipeline(m_device, m_graphics_pipeline, nullptr);
            if (m_outline_pipeline != VK_NULL_HANDLE)
                vkDestroyPipeline(m_device, m_outline_pipeline, nullptr);
            if (m_bloom_pipeline != VK_NULL_HANDLE)
                vkDestroyPipeline(m_device, m_bloom_pipeline, nullptr);
            if (m_pipeline_layout != VK_NULL_HANDLE)
                vkDestroyPipelineLayout(m_device, m_pipeline_layout, nullptr);
            if (m_bloom_pipeline_layout != VK_NULL_HANDLE)
                vkDestroyPipelineLayout(m_device, m_bloom_pipeline_layout, nullptr);
            if (m_descriptor_set_layout != VK_NULL_HANDLE)
                vkDestroyDescriptorSetLayout(m_device, m_descriptor_set_layout, nullptr);
            if (m_post_descriptor_set_layout != VK_NULL_HANDLE)
                vkDestroyDescriptorSetLayout(m_device, m_post_descriptor_set_layout, nullptr);
            if (m_bloom_sampler != VK_NULL_HANDLE)
                vkDestroySampler(m_device, m_bloom_sampler, nullptr);
            if (m_render_pass != VK_NULL_HANDLE)
                vkDestroyRenderPass(m_device, m_render_pass, nullptr);
            if (m_present_render_pass != VK_NULL_HANDLE)
                vkDestroyRenderPass(m_device, m_present_render_pass, nullptr);

            for (FrameSync& frame : m_frames)
            {
                if (frame.image_available != VK_NULL_HANDLE)
                    vkDestroySemaphore(m_device, frame.image_available, nullptr);
                if (frame.render_finished != VK_NULL_HANDLE)
                    vkDestroySemaphore(m_device, frame.render_finished, nullptr);
                if (frame.in_flight != VK_NULL_HANDLE)
                    vkDestroyFence(m_device, frame.in_flight, nullptr);
            }

            if (m_command_pool != VK_NULL_HANDLE)
                vkDestroyCommandPool(m_device, m_command_pool, nullptr);
            if (m_descriptor_pool != VK_NULL_HANDLE)
                vkDestroyDescriptorPool(m_device, m_descriptor_pool, nullptr);
            if (m_device != VK_NULL_HANDLE)
                vkDestroyDevice(m_device, nullptr);
            if (m_surface != VK_NULL_HANDLE)
                vkDestroySurfaceKHR(m_instance, m_surface, nullptr);
            if (m_instance != VK_NULL_HANDLE)
                vkDestroyInstance(m_instance, nullptr);
        }

        [[nodiscard]] bool begin_frame()
        {
            FrameSync& frame = m_frames[m_current_frame];
            check_vk(vkWaitForFences(m_device, 1, &frame.in_flight, VK_TRUE, UINT64_MAX), "Failed to wait for frame fence");
            collect_completed_readback(m_current_frame);

            VkResult result = vkAcquireNextImageKHR(
                m_device, m_swapchain, UINT64_MAX, frame.image_available, VK_NULL_HANDLE, &m_active_image_index);
            if (result == VK_ERROR_OUT_OF_DATE_KHR)
            {
                recreate_swapchain();
                return false;
            }
            if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR)
                check_vk(result, "Failed to acquire swapchain image");

            if (m_images_in_flight[m_active_image_index] != VK_NULL_HANDLE)
                check_vk(vkWaitForFences(m_device, 1, &m_images_in_flight[m_active_image_index], VK_TRUE, UINT64_MAX),
                         "Failed to wait for swapchain image fence");
            m_images_in_flight[m_active_image_index] = frame.in_flight;

            check_vk(vkResetFences(m_device, 1, &frame.in_flight), "Failed to reset frame fence");

            m_active_command_buffer = m_command_buffers[m_active_image_index];
            check_vk(vkResetCommandBuffer(m_active_command_buffer, 0), "Failed to reset command buffer");

            VkCommandBufferBeginInfo begin_info{};
            begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
            check_vk(vkBeginCommandBuffer(m_active_command_buffer, &begin_info), "Failed to begin command buffer");

            const std::array<VkClearValue, 3> clear_values{
                VkClearValue{.color = {{0.3f, 0.3f, 0.3f, 1.0f}}},
                VkClearValue{.color = {{0.0f, 0.0f, 0.0f, 0.0f}}},
                VkClearValue{.depthStencil = {1.0f, 0}},
            };

            VkRenderPassBeginInfo render_pass_info{};
            render_pass_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
            render_pass_info.renderPass = m_render_pass;
            render_pass_info.framebuffer = m_scene_framebuffer;
            render_pass_info.renderArea.offset = {0, 0};
            render_pass_info.renderArea.extent = m_scene_extent;
            render_pass_info.clearValueCount = static_cast<uint32_t>(clear_values.size());
            render_pass_info.pClearValues = clear_values.data();

            vkCmdBeginRenderPass(m_active_command_buffer, &render_pass_info, VK_SUBPASS_CONTENTS_INLINE);
            vkCmdBindPipeline(m_active_command_buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_graphics_pipeline);

            VkViewport viewport{};
            viewport.x = 0.0f;
            viewport.y = 0.0f;
            viewport.width = static_cast<float>(m_scene_extent.width);
            viewport.height = static_cast<float>(m_scene_extent.height);
            viewport.minDepth = 0.0f;
            viewport.maxDepth = 1.0f;
            vkCmdSetViewport(m_active_command_buffer, 0, 1, &viewport);

            const VkRect2D scissor{{0, 0}, m_scene_extent};
            vkCmdSetScissor(m_active_command_buffer, 0, 1, &scissor);

            m_frame_started = true;
            m_scene_render_pass_active = true;
            m_present_render_pass_active = false;
            m_frame_view_projection_set = false;
            return true;
        }

        void draw_mesh(const Mesh& mesh, const omath::opengl_engine::Camera& camera)
        {
            draw_mesh_with_pipeline(mesh, camera, m_graphics_pipeline, {}, 0.0f, 0.0f, false);
        }

        void draw_mesh_outline(const Mesh& mesh, const omath::opengl_engine::Camera& camera)
        {
            const int pass_count = std::clamp(m_selection_outline_settings.smoothing_quality, 1, 64);
            const float width = std::clamp(m_selection_outline_settings.width, 0.0f, 2.0f);

            if (pass_count == 1)
            {
                draw_mesh_with_pipeline(mesh,
                                        camera,
                                        m_outline_pipeline,
                                        m_selection_outline_settings.color,
                                        width,
                                        0.85f,
                                        true);
                return;
            }

            for (int pass = pass_count - 1; pass >= 0; --pass)
            {
                const float inner_t = static_cast<float>(pass_count - 1 - pass)
                                    / static_cast<float>(pass_count - 1);
                const float width_t = static_cast<float>(pass + 1) / static_cast<float>(pass_count);
                const float alpha = 0.12f + (0.85f - 0.12f) * inner_t * inner_t;
                draw_mesh_with_pipeline(mesh,
                                        camera,
                                        m_outline_pipeline,
                                        m_selection_outline_settings.color,
                                        width * width_t,
                                        alpha,
                                        true);
            }
        }

        void draw_mesh_with_pipeline(const Mesh& mesh,
                                     const omath::opengl_engine::Camera& camera,
                                     VkPipeline pipeline,
                                     const std::array<float, 3>& outline_color,
                                     float outline_width,
                                     float outline_alpha,
                                     bool outline_enabled)
        {
            if (!m_frame_started)
                throw VulkanError("draw_mesh() called outside a frame");
            if (!m_scene_render_pass_active)
                throw VulkanError("draw_mesh() called after scene rendering finished");

            GpuMesh& gpu_mesh = ensure_mesh_resource(mesh);
            if (gpu_mesh.index_count == 0)
                return;

            vkCmdBindPipeline(m_active_command_buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);

            const VkBuffer vertex_buffers[] = {gpu_mesh.vertex_buffer.buffer};
            const VkDeviceSize offsets[] = {0};
            vkCmdBindVertexBuffers(m_active_command_buffer, 0, 1, vertex_buffers, offsets);
            vkCmdBindIndexBuffer(m_active_command_buffer, gpu_mesh.index_buffer.buffer, 0, VK_INDEX_TYPE_UINT32);
            vkCmdBindDescriptorSets(m_active_command_buffer,
                                    VK_PIPELINE_BIND_POINT_GRAPHICS,
                                    m_pipeline_layout,
                                    0,
                                    1,
                                    &gpu_mesh.descriptor,
                                    0,
                                    nullptr);

            PushConstants push{};
            const auto vp = camera.get_view_projection_matrix().raw_array();
            const auto model = mesh.cpu_mesh().get_to_world_matrix().raw_array();
            if (!m_frame_view_projection_set)
            {
                std::copy(vp.begin(), vp.end(), m_frame_view_projection.begin());
                m_frame_view_projection_set = true;
            }

            std::memcpy(push.view_projection, vp.data(), sizeof(push.view_projection));
            std::memcpy(push.model, model.data(), sizeof(push.model));
            const std::array<float, 16>& previous_vp =
                m_previous_view_projection_valid ? m_previous_view_projection : m_frame_view_projection;
            std::memcpy(push.previous_view_projection, previous_vp.data(), sizeof(push.previous_view_projection));
            push.outline_center[0] = gpu_mesh.local_center.x;
            push.outline_center[1] = gpu_mesh.local_center.y;
            push.outline_center[2] = gpu_mesh.local_center.z;
            push.outline_width = outline_width;
            push.outline_color[0] = outline_color[0];
            push.outline_color[1] = outline_color[1];
            push.outline_color[2] = outline_color[2];
            push.outline_alpha = outline_alpha;
            push.outline_enabled = outline_enabled ? 1 : 0;
            vkCmdPushConstants(m_active_command_buffer,
                               m_pipeline_layout,
                               VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                               0,
                               sizeof(PushConstants),
                               &push);

            vkCmdDrawIndexed(m_active_command_buffer, gpu_mesh.index_count, 1, 0, 0, 0);
        }

        void render_imgui(ImDrawData* draw_data)
        {
            if (!m_frame_started)
                throw VulkanError("render_imgui() called outside a frame");
            finish_scene_rendering();
            ImGui_ImplVulkan_RenderDrawData(draw_data, m_active_command_buffer);
        }

        [[nodiscard]] std::optional<CapturedFrame> end_frame(bool capture_screenshot)
        {
            if (!m_frame_started)
                return std::nullopt;

            std::optional<CapturedFrame> screenshot = std::move(m_completed_stream_frame);
            m_completed_stream_frame.reset();

            ReadbackSlot* readback_slot = nullptr;
            const bool can_capture = capture_screenshot && m_swapchain_supports_transfer_src;
            if (can_capture)
            {
                const VkDeviceSize image_size = static_cast<VkDeviceSize>(m_swapchain_extent.width)
                                              * static_cast<VkDeviceSize>(m_swapchain_extent.height)
                                              * 4u;
                readback_slot = &m_readback_slots[m_current_frame];
                ensure_readback_slot(*readback_slot, image_size);
                readback_slot->extent = m_swapchain_extent;
                readback_slot->format = m_swapchain_image_format;
            }

            finish_scene_rendering();
            if (m_present_render_pass_active)
            {
                vkCmdEndRenderPass(m_active_command_buffer);
                m_present_render_pass_active = false;
            }

            if (readback_slot != nullptr)
                record_screenshot_copy(readback_slot->buffer);

            check_vk(vkEndCommandBuffer(m_active_command_buffer), "Failed to end command buffer");

            FrameSync& frame = m_frames[m_current_frame];
            const VkPipelineStageFlags wait_stage = VK_PIPELINE_STAGE_TRANSFER_BIT
                                                  | VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;

            VkSubmitInfo submit_info{};
            submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
            submit_info.waitSemaphoreCount = 1;
            submit_info.pWaitSemaphores = &frame.image_available;
            submit_info.pWaitDstStageMask = &wait_stage;
            submit_info.commandBufferCount = 1;
            submit_info.pCommandBuffers = &m_active_command_buffer;
            submit_info.signalSemaphoreCount = 1;
            submit_info.pSignalSemaphores = &frame.render_finished;
            check_vk(vkQueueSubmit(m_graphics_queue, 1, &submit_info, frame.in_flight), "Failed to submit draw command buffer");

            if (readback_slot != nullptr)
                readback_slot->pending = true;

            VkPresentInfoKHR present_info{};
            present_info.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
            present_info.waitSemaphoreCount = 1;
            present_info.pWaitSemaphores = &frame.render_finished;
            present_info.swapchainCount = 1;
            present_info.pSwapchains = &m_swapchain;
            present_info.pImageIndices = &m_active_image_index;

            const VkResult result = vkQueuePresentKHR(m_present_queue, &present_info);
            if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR)
                recreate_swapchain();
            else if (result != VK_SUCCESS)
                check_vk(result, "Failed to present swapchain image");

            m_current_frame = (m_current_frame + 1u) % static_cast<std::size_t>(k_max_frames_in_flight);
            m_active_command_buffer = VK_NULL_HANDLE;
            m_frame_started = false;
            if (m_frame_view_projection_set)
            {
                m_previous_view_projection = m_frame_view_projection;
                m_previous_view_projection_valid = true;
                m_frame_view_projection_set = false;
            }
            return screenshot;
        }

        void wait_idle() const
        {
            check_vk(vkDeviceWaitIdle(m_device), "Failed to wait for Vulkan device");
        }

        [[nodiscard]] omath::Vector2<int> framebuffer_size() const
        {
            return {
                static_cast<int>(m_swapchain_extent.width),
                static_cast<int>(m_swapchain_extent.height)
            };
        }

        [[nodiscard]] bool dlss_available() const
        {
#ifdef ROSE_ENABLE_NGX_DLSS
            return m_ngx_available
                || (!m_ngx_init_attempted && !m_ngx_disabled_by_env && m_ngx_extensions_available);
#else
            return false;
#endif
        }

        [[nodiscard]] bool dlss_active() const
        {
#ifdef ROSE_ENABLE_NGX_DLSS
            return m_dlss_requested
                && m_ngx_available
                && m_ngx_dlss_handle != nullptr;
#else
            return false;
#endif
        }

        [[nodiscard]] VkExtent2D choose_scene_extent()
        {
            VkExtent2D extent = m_swapchain_extent;
#ifdef ROSE_ENABLE_NGX_DLSS
            release_dlss_feature();
            if (!m_dlss_requested || !m_ngx_available || m_ngx_parameters == nullptr)
                return extent;

            unsigned int optimal_width = m_swapchain_extent.width;
            unsigned int optimal_height = m_swapchain_extent.height;
            unsigned int max_width = optimal_width;
            unsigned int max_height = optimal_height;
            unsigned int min_width = optimal_width;
            unsigned int min_height = optimal_height;
            float sharpness = 0.0f;
            const NVSDK_NGX_Result settings_result = NGX_DLSS_GET_OPTIMAL_SETTINGS(m_ngx_parameters,
                                                                                   m_swapchain_extent.width,
                                                                                   m_swapchain_extent.height,
                                                                                   to_ngx_quality(m_dlss_quality),
                                                                                   &optimal_width,
                                                                                   &optimal_height,
                                                                                   &max_width,
                                                                                   &max_height,
                                                                                   &min_width,
                                                                                   &min_height,
                                                                                   &sharpness);
            if (NVSDK_NGX_FAILED(settings_result) || optimal_width == 0 || optimal_height == 0)
            {
                m_dlss_status = std::string("DLSS settings query failed: ") + ngx_result_name(settings_result);
                return extent;
            }

            extent.width = optimal_width;
            extent.height = optimal_height;
            m_dlss_sharpness = sharpness;
            if (!create_dlss_feature(extent))
            {
                m_dlss_requested = false;
                extent = m_swapchain_extent;
            }
#endif
            return extent;
        }

        [[nodiscard]] bool ensure_dlss_sdk_initialized()
        {
#ifdef ROSE_ENABLE_NGX_DLSS
            if (m_ngx_available)
                return true;
            if (m_ngx_disabled_by_env)
            {
                m_dlss_status = "DLSS disabled by ROSE_DISABLE_NGX_DLSS";
                return false;
            }
            if (m_ngx_init_attempted)
                return false;

            m_ngx_init_attempted = true;
            init_dlss_sdk();
            return m_ngx_available;
#else
            m_dlss_status = "DLSS support was not compiled";
            return false;
#endif
        }

        void init_dlss_sdk()
        {
#ifdef ROSE_ENABLE_NGX_DLSS
            if (!m_ngx_extensions_available)
            {
                spdlog::warn("Vulkan: skipping NGX init because required extensions are unavailable");
                return;
            }

            std::error_code error;
            const std::filesystem::path log_path = std::filesystem::current_path(error) / "ngx";
            if (!error)
                std::filesystem::create_directories(log_path, error);
            const std::wstring log_path_w = log_path.wstring();
            spdlog::info("Vulkan: initializing NVIDIA NGX/DLSS");

            NVSDK_NGX_FeatureCommonInfo feature_info{};
            feature_info.LoggingInfo.MinimumLoggingLevel = NVSDK_NGX_LOGGING_LEVEL_ON;
            const NVSDK_NGX_Result init_result =
                NVSDK_NGX_VULKAN_Init_with_ProjectID("7b8f624d-44ea-4bc6-b656-4e72206d8d54",
                                                     NVSDK_NGX_ENGINE_TYPE_CUSTOM,
                                                     "1.0.0",
                                                     log_path_w.c_str(),
                                                     m_instance,
                                                     m_physical_device,
                                                     m_device,
                                                     vkGetInstanceProcAddr,
                                                     vkGetDeviceProcAddr,
                                                     &feature_info);
            if (NVSDK_NGX_FAILED(init_result))
            {
                m_dlss_status = std::string("NGX init failed: ") + ngx_result_name(init_result);
                spdlog::warn("Vulkan: {}", m_dlss_status);
                return;
            }
            m_ngx_initialized = true;

            const NVSDK_NGX_Result params_result = NVSDK_NGX_VULKAN_GetCapabilityParameters(&m_ngx_parameters);
            if (NVSDK_NGX_FAILED(params_result) || m_ngx_parameters == nullptr)
            {
                m_dlss_status = std::string("NGX capability query failed: ") + ngx_result_name(params_result);
                spdlog::warn("Vulkan: {}", m_dlss_status);
                return;
            }

            int available = 0;
            const NVSDK_NGX_Result available_result =
                m_ngx_parameters->Get(NVSDK_NGX_Parameter_SuperSampling_Available, &available);
            if (NVSDK_NGX_FAILED(available_result) || available == 0)
            {
                m_dlss_status = "DLSS is not available on this system";
                spdlog::warn("Vulkan: {}", m_dlss_status);
                return;
            }

            m_ngx_available = true;
            m_dlss_status = "DLSS is available";
            spdlog::info("Vulkan: {}", m_dlss_status);
#else
            m_dlss_status = "DLSS support was not compiled";
#endif
        }

        void release_dlss_feature()
        {
#ifdef ROSE_ENABLE_NGX_DLSS
            if (m_ngx_dlss_handle != nullptr)
            {
                const NVSDK_NGX_Result result = NVSDK_NGX_VULKAN_ReleaseFeature(m_ngx_dlss_handle);
                if (NVSDK_NGX_FAILED(result))
                    spdlog::warn("Failed to release DLSS feature: {}", ngx_result_name(result));
                m_ngx_dlss_handle = nullptr;
            }
#endif
        }

        [[nodiscard]] bool create_dlss_feature(VkExtent2D render_extent)
        {
#ifdef ROSE_ENABLE_NGX_DLSS
            NVSDK_NGX_DLSS_Create_Params create_params{};
            create_params.Feature.InWidth = render_extent.width;
            create_params.Feature.InHeight = render_extent.height;
            create_params.Feature.InTargetWidth = m_swapchain_extent.width;
            create_params.Feature.InTargetHeight = m_swapchain_extent.height;
            create_params.Feature.InPerfQualityValue = to_ngx_quality(m_dlss_quality);
            create_params.InFeatureCreateFlags = NVSDK_NGX_DLSS_Feature_Flags_MVLowRes
                                               | NVSDK_NGX_DLSS_Feature_Flags_AutoExposure;
            create_params.InEnableOutputSubrects = false;

            VkCommandBuffer command_buffer = begin_single_time_commands();
            const NVSDK_NGX_Result result = NGX_VULKAN_CREATE_DLSS_EXT1(m_device,
                                                                        command_buffer,
                                                                        1,
                                                                        1,
                                                                        &m_ngx_dlss_handle,
                                                                        m_ngx_parameters,
                                                                        &create_params);
            end_single_time_commands(command_buffer);

            if (NVSDK_NGX_FAILED(result) || m_ngx_dlss_handle == nullptr)
            {
                m_ngx_dlss_handle = nullptr;
                m_dlss_status = std::string("DLSS feature creation failed: ") + ngx_result_name(result);
                spdlog::warn("Vulkan: {}", m_dlss_status);
                return false;
            }

            m_dlss_reset_next_frame = true;
            m_dlss_status = std::string("DLSS ") + dlss_quality_label(m_dlss_quality)
                          + " (" + std::to_string(render_extent.width) + "x" + std::to_string(render_extent.height)
                          + " -> " + std::to_string(m_swapchain_extent.width) + "x" + std::to_string(m_swapchain_extent.height)
                          + ")";
            spdlog::info("Vulkan: {}", m_dlss_status);
            return true;
#else
            static_cast<void>(render_extent);
            return false;
#endif
        }

        void shutdown_dlss_sdk()
        {
#ifdef ROSE_ENABLE_NGX_DLSS
            release_dlss_feature();
            if (m_ngx_parameters != nullptr)
            {
                const NVSDK_NGX_Result result = NVSDK_NGX_VULKAN_DestroyParameters(m_ngx_parameters);
                if (NVSDK_NGX_FAILED(result))
                    spdlog::warn("Failed to destroy NGX parameters: {}", ngx_result_name(result));
                m_ngx_parameters = nullptr;
            }
            if (m_ngx_initialized)
            {
                const NVSDK_NGX_Result result = NVSDK_NGX_VULKAN_Shutdown1(m_device);
                if (NVSDK_NGX_FAILED(result))
                    spdlog::warn("Failed to shut down NGX: {}", ngx_result_name(result));
                m_ngx_initialized = false;
            }
            m_ngx_available = false;
#endif
        }

        void destroy_frame_targets() noexcept
        {
            for (VkFramebuffer framebuffer : m_framebuffers)
                vkDestroyFramebuffer(m_device, framebuffer, nullptr);
            m_framebuffers.clear();

            if (m_scene_framebuffer != VK_NULL_HANDLE)
            {
                vkDestroyFramebuffer(m_device, m_scene_framebuffer, nullptr);
                m_scene_framebuffer = VK_NULL_HANDLE;
            }

            release_dlss_feature();
            m_bloom_descriptor_image_view = VK_NULL_HANDLE;
            destroy_image(m_dlss_output_image);
            destroy_image(m_motion_vector_image);
            destroy_image(m_scene_color_image);
            destroy_image(m_depth_image);
        }

        void recreate_frame_targets()
        {
            if (m_frame_started)
                return;

            spdlog::info("Vulkan: recreating frame targets");
            check_vk(vkDeviceWaitIdle(m_device), "Failed to wait for device before recreating render targets");
            destroy_frame_targets();
            create_render_targets();
            create_framebuffers();
        }

        void set_dlss_enabled(bool enabled)
        {
            spdlog::info("Vulkan: DLSS {}", enabled ? "enable requested" : "disable requested");
            if (enabled && !ensure_dlss_sdk_initialized())
            {
                m_dlss_requested = false;
                m_dlss_reset_next_frame = true;
                spdlog::warn("Vulkan: DLSS enable failed: {}", m_dlss_status);
                return;
            }

            if (m_dlss_requested == enabled)
                return;

            m_dlss_requested = enabled;
            m_dlss_reset_next_frame = true;
            if (!enabled)
                m_dlss_status = "DLSS is disabled";
            recreate_frame_targets();
        }

        void set_dlss_quality(DlssQuality quality)
        {
            if (m_dlss_quality == quality)
                return;

            m_dlss_quality = quality;
            m_dlss_reset_next_frame = true;
            if (m_dlss_requested)
                recreate_frame_targets();
        }

        void set_selection_outline_settings(const SelectionOutlineSettings& settings)
        {
            m_selection_outline_settings = settings;
            for (float& channel : m_selection_outline_settings.color)
                channel = std::clamp(channel, 0.0f, 1.0f);
            m_selection_outline_settings.width = std::clamp(m_selection_outline_settings.width, 0.0f, 2.0f);
            m_selection_outline_settings.smoothing_quality =
                std::clamp(m_selection_outline_settings.smoothing_quality, 1, 64);
        }

        void set_bloom_settings(const BloomSettings& settings)
        {
            m_bloom_settings = settings;
            m_bloom_settings.threshold = std::clamp(m_bloom_settings.threshold, 0.0f, 8.0f);
            m_bloom_settings.intensity = std::clamp(m_bloom_settings.intensity, 0.0f, 5.0f);
            m_bloom_settings.radius = std::clamp(m_bloom_settings.radius, 0.0f, 32.0f);
            m_bloom_settings.quality = std::clamp(m_bloom_settings.quality, 1, 64);
        }

        void create_instance()
        {
            VkApplicationInfo app_info{};
            app_info.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
            app_info.pApplicationName = "ROSE";
            app_info.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
            app_info.pEngineName = "rose.core";
            app_info.engineVersion = VK_MAKE_VERSION(1, 0, 0);
            app_info.apiVersion = k_api_version;

            uint32_t glfw_extension_count = 0;
            const char** glfw_extensions = glfwGetRequiredInstanceExtensions(&glfw_extension_count);
            if (glfw_extensions == nullptr || glfw_extension_count == 0)
                throw VulkanError("GLFW did not report required Vulkan instance extensions");

            load_ngx_required_extensions();
            std::vector<std::string> extension_names;
            extension_names.reserve(glfw_extension_count + m_ngx_instance_extensions.size());
            for (uint32_t i = 0; i < glfw_extension_count; ++i)
                extension_names.emplace_back(glfw_extensions[i]);
            append_available_instance_extensions(extension_names);
            spdlog::info("Vulkan: creating instance api={} extensions=[{}]",
                         vulkan_version_string(k_api_version),
                         join_names(extension_names));

            std::vector<const char*> extension_ptrs;
            extension_ptrs.reserve(extension_names.size());
            for (const std::string& extension : extension_names)
                extension_ptrs.push_back(extension.c_str());

            VkInstanceCreateInfo create_info{};
            create_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
            create_info.pApplicationInfo = &app_info;
            create_info.enabledExtensionCount = static_cast<uint32_t>(extension_ptrs.size());
            create_info.ppEnabledExtensionNames = extension_ptrs.data();
            create_info.enabledLayerCount = 0;

            check_vk(vkCreateInstance(&create_info, nullptr, &m_instance), "Failed to create Vulkan instance");
            spdlog::info("Vulkan: instance created");
        }

        void create_surface()
        {
            check_vk(glfwCreateWindowSurface(m_instance, m_window, nullptr, &m_surface), "Failed to create Vulkan surface");
            spdlog::info("Vulkan: GLFW surface created");
        }

        void load_ngx_required_extensions()
        {
#ifdef ROSE_ENABLE_NGX_DLSS
            unsigned int instance_count = 0;
            unsigned int device_count = 0;
            const char** instance_extensions = nullptr;
            const char** device_extensions = nullptr;
            const NVSDK_NGX_Result result = NVSDK_NGX_VULKAN_RequiredExtensions(
                &instance_count, &instance_extensions, &device_count, &device_extensions);
            if (NVSDK_NGX_FAILED(result))
            {
                m_ngx_extensions_available = false;
                m_dlss_status = std::string("NGX extension query failed: ") + ngx_result_name(result);
                spdlog::warn("Vulkan: {}", m_dlss_status);
                return;
            }

            m_ngx_instance_extensions.clear();
            m_ngx_device_extensions.clear();
            for (unsigned int i = 0; i < instance_count; ++i)
                m_ngx_instance_extensions.emplace_back(instance_extensions[i]);
            for (unsigned int i = 0; i < device_count; ++i)
                m_ngx_device_extensions.emplace_back(device_extensions[i]);
            spdlog::info("Vulkan: NGX requested {} instance extension(s), {} device extension(s)",
                         instance_count,
                         device_count);
#endif
        }

        [[nodiscard]] bool instance_extension_available(const char* name) const
        {
            uint32_t extension_count = 0;
            check_vk(vkEnumerateInstanceExtensionProperties(nullptr, &extension_count, nullptr),
                     "Failed to enumerate Vulkan instance extension count");
            std::vector<VkExtensionProperties> extensions(extension_count);
            check_vk(vkEnumerateInstanceExtensionProperties(nullptr, &extension_count, extensions.data()),
                     "Failed to enumerate Vulkan instance extensions");

            return std::any_of(extensions.begin(),
                               extensions.end(),
                               [name](const VkExtensionProperties& extension)
                               {
                                   return std::strcmp(extension.extensionName, name) == 0;
                               });
        }

        void append_available_instance_extensions(std::vector<std::string>& extensions)
        {
            for (const std::string& extension : m_ngx_instance_extensions)
            {
                if (instance_extension_available(extension.c_str()))
                    extensions.push_back(extension);
                else
                {
                    m_ngx_extensions_available = false;
                    m_dlss_status = "Missing Vulkan instance extension for DLSS: " + extension;
                    spdlog::warn("Vulkan: {}", m_dlss_status);
                }
            }
        }

        [[nodiscard]] bool device_extension_available(const char* name) const
        {
            uint32_t extension_count = 0;
            check_vk(vkEnumerateDeviceExtensionProperties(m_physical_device, nullptr, &extension_count, nullptr),
                     "Failed to enumerate Vulkan device extension count");
            std::vector<VkExtensionProperties> extensions(extension_count);
            check_vk(vkEnumerateDeviceExtensionProperties(m_physical_device, nullptr, &extension_count, extensions.data()),
                     "Failed to enumerate Vulkan device extensions");

            return std::any_of(extensions.begin(),
                               extensions.end(),
                               [name](const VkExtensionProperties& extension)
                               {
                                   return std::strcmp(extension.extensionName, name) == 0;
                               });
        }

        void append_available_device_extensions(std::vector<std::string>& extensions)
        {
            for (const std::string& extension : m_ngx_device_extensions)
            {
                if (device_extension_available(extension.c_str()))
                    extensions.push_back(extension);
                else
                {
                    m_ngx_extensions_available = false;
                    m_dlss_status = "Missing Vulkan device extension for DLSS: " + extension;
                    spdlog::warn("Vulkan: {}", m_dlss_status);
                }
            }
        }

        [[nodiscard]] QueueFamilies find_queue_families(VkPhysicalDevice device) const
        {
            QueueFamilies indices;

            uint32_t queue_family_count = 0;
            vkGetPhysicalDeviceQueueFamilyProperties(device, &queue_family_count, nullptr);
            std::vector<VkQueueFamilyProperties> queue_families(queue_family_count);
            vkGetPhysicalDeviceQueueFamilyProperties(device, &queue_family_count, queue_families.data());

            for (uint32_t i = 0; i < queue_family_count; ++i)
            {
                if ((queue_families[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) != 0)
                    indices.graphics = i;

                VkBool32 present_support = VK_FALSE;
                check_vk(vkGetPhysicalDeviceSurfaceSupportKHR(device, i, m_surface, &present_support),
                         "Failed to query Vulkan present support");
                if (present_support == VK_TRUE)
                    indices.present = i;

                if (indices.complete())
                    break;
            }

            return indices;
        }

        [[nodiscard]] bool check_device_extension_support(VkPhysicalDevice device) const
        {
            uint32_t extension_count = 0;
            vkEnumerateDeviceExtensionProperties(device, nullptr, &extension_count, nullptr);
            std::vector<VkExtensionProperties> available_extensions(extension_count);
            vkEnumerateDeviceExtensionProperties(device, nullptr, &extension_count, available_extensions.data());

            std::set<std::string> required_extensions{VK_KHR_SWAPCHAIN_EXTENSION_NAME};
            for (const VkExtensionProperties& extension : available_extensions)
                required_extensions.erase(extension.extensionName);
            return required_extensions.empty();
        }

        [[nodiscard]] SwapchainSupport query_swapchain_support(VkPhysicalDevice device) const
        {
            SwapchainSupport details;
            check_vk(vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device, m_surface, &details.capabilities),
                     "Failed to query surface capabilities");

            uint32_t format_count = 0;
            check_vk(vkGetPhysicalDeviceSurfaceFormatsKHR(device, m_surface, &format_count, nullptr),
                     "Failed to query surface format count");
            if (format_count != 0)
            {
                details.formats.resize(format_count);
                check_vk(vkGetPhysicalDeviceSurfaceFormatsKHR(device, m_surface, &format_count, details.formats.data()),
                         "Failed to query surface formats");
            }

            uint32_t present_mode_count = 0;
            check_vk(vkGetPhysicalDeviceSurfacePresentModesKHR(device, m_surface, &present_mode_count, nullptr),
                     "Failed to query present mode count");
            if (present_mode_count != 0)
            {
                details.present_modes.resize(present_mode_count);
                check_vk(vkGetPhysicalDeviceSurfacePresentModesKHR(device,
                                                                   m_surface,
                                                                   &present_mode_count,
                                                                   details.present_modes.data()),
                         "Failed to query present modes");
            }

            return details;
        }

        [[nodiscard]] bool is_device_suitable(VkPhysicalDevice device) const
        {
            const QueueFamilies indices = find_queue_families(device);
            if (!indices.complete() || !check_device_extension_support(device))
                return false;

            const SwapchainSupport support = query_swapchain_support(device);
            return !support.formats.empty() && !support.present_modes.empty();
        }

        void pick_physical_device()
        {
            uint32_t device_count = 0;
            check_vk(vkEnumeratePhysicalDevices(m_instance, &device_count, nullptr), "Failed to enumerate Vulkan devices");
            if (device_count == 0)
                throw VulkanError("No Vulkan-capable GPU found");
            spdlog::info("Vulkan: found {} physical device(s)", device_count);

            std::vector<VkPhysicalDevice> devices(device_count);
            check_vk(vkEnumeratePhysicalDevices(m_instance, &device_count, devices.data()), "Failed to enumerate Vulkan devices");

            std::optional<VkPhysicalDevice> fallback_device;
            for (VkPhysicalDevice device : devices)
            {
                VkPhysicalDeviceProperties properties{};
                vkGetPhysicalDeviceProperties(device, &properties);
                const bool suitable = is_device_suitable(device);
                spdlog::info("Vulkan: GPU candidate '{}' type={} api={} driver={} vendor=0x{:04x} device=0x{:04x} suitable={}",
                             properties.deviceName,
                             physical_device_type_name(properties.deviceType),
                             vulkan_version_string(properties.apiVersion),
                             properties.driverVersion,
                             properties.vendorID,
                             properties.deviceID,
                             suitable);
                if (!suitable)
                    continue;

                if (properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU)
                {
                    m_physical_device = device;
                    spdlog::info("Vulkan: selected GPU '{}'", properties.deviceName);
                    return;
                }

                if (!fallback_device)
                    fallback_device = device;
            }

            if (fallback_device)
            {
                VkPhysicalDeviceProperties properties{};
                vkGetPhysicalDeviceProperties(*fallback_device, &properties);
                m_physical_device = *fallback_device;
                spdlog::info("Vulkan: selected GPU '{}'", properties.deviceName);
                return;
            }

            throw VulkanError("No suitable Vulkan device found");
        }

        void create_logical_device()
        {
            const QueueFamilies indices = find_queue_families(m_physical_device);
            m_graphics_queue_family = indices.graphics.value();
            m_present_queue_family = indices.present.value();

            const std::set<uint32_t> unique_queue_families{m_graphics_queue_family, m_present_queue_family};
            const float queue_priority = 1.0f;
            std::vector<VkDeviceQueueCreateInfo> queue_create_infos;
            queue_create_infos.reserve(unique_queue_families.size());
            for (uint32_t queue_family : unique_queue_families)
            {
                VkDeviceQueueCreateInfo queue_create_info{};
                queue_create_info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
                queue_create_info.queueFamilyIndex = queue_family;
                queue_create_info.queueCount = 1;
                queue_create_info.pQueuePriorities = &queue_priority;
                queue_create_infos.push_back(queue_create_info);
            }

            std::vector<std::string> extension_names{VK_KHR_SWAPCHAIN_EXTENSION_NAME};
            append_available_device_extensions(extension_names);
            std::vector<const char*> extension_ptrs;
            extension_ptrs.reserve(extension_names.size());
            for (const std::string& extension : extension_names)
                extension_ptrs.push_back(extension.c_str());
            spdlog::info("Vulkan: creating logical device graphics_queue={} present_queue={} extensions=[{}]",
                         m_graphics_queue_family,
                         m_present_queue_family,
                         join_names(extension_names));

            VkPhysicalDeviceFeatures device_features{};
            VkDeviceCreateInfo create_info{};
            create_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
            create_info.queueCreateInfoCount = static_cast<uint32_t>(queue_create_infos.size());
            create_info.pQueueCreateInfos = queue_create_infos.data();
            create_info.pEnabledFeatures = &device_features;
            create_info.enabledExtensionCount = static_cast<uint32_t>(extension_ptrs.size());
            create_info.ppEnabledExtensionNames = extension_ptrs.data();

            check_vk(vkCreateDevice(m_physical_device, &create_info, nullptr, &m_device), "Failed to create Vulkan device");
            vkGetDeviceQueue(m_device, m_graphics_queue_family, 0, &m_graphics_queue);
            vkGetDeviceQueue(m_device, m_present_queue_family, 0, &m_present_queue);
            spdlog::info("Vulkan: logical device created");
        }

        [[nodiscard]] VkSurfaceFormatKHR choose_surface_format(const std::vector<VkSurfaceFormatKHR>& formats) const
        {
            for (const VkSurfaceFormatKHR& format : formats)
            {
                if (format.format == VK_FORMAT_B8G8R8A8_SRGB && format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR)
                    return format;
            }
            for (const VkSurfaceFormatKHR& format : formats)
            {
                if (format.format == VK_FORMAT_R8G8B8A8_SRGB && format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR)
                    return format;
            }
            for (const VkSurfaceFormatKHR& format : formats)
            {
                if (is_bgra_format(format.format) || is_rgba_format(format.format))
                    return format;
            }
            throw VulkanError("No supported 8-bit RGBA/BGRA swapchain format found");
        }

        [[nodiscard]] VkPresentModeKHR choose_present_mode(const std::vector<VkPresentModeKHR>& present_modes) const
        {
            for (VkPresentModeKHR mode : present_modes)
            {
                if (mode == VK_PRESENT_MODE_MAILBOX_KHR)
                    return mode;
            }
            return VK_PRESENT_MODE_FIFO_KHR;
        }

        [[nodiscard]] VkExtent2D choose_extent(const VkSurfaceCapabilitiesKHR& capabilities) const
        {
            if (capabilities.currentExtent.width != std::numeric_limits<uint32_t>::max())
                return capabilities.currentExtent;

            int width = 0;
            int height = 0;
            glfwGetFramebufferSize(m_window, &width, &height);

            VkExtent2D actual_extent{
                static_cast<uint32_t>(std::max(width, 1)),
                static_cast<uint32_t>(std::max(height, 1)),
            };
            actual_extent.width = std::clamp(actual_extent.width,
                                             capabilities.minImageExtent.width,
                                             capabilities.maxImageExtent.width);
            actual_extent.height = std::clamp(actual_extent.height,
                                              capabilities.minImageExtent.height,
                                              capabilities.maxImageExtent.height);
            return actual_extent;
        }

        void create_swapchain()
        {
            const SwapchainSupport support = query_swapchain_support(m_physical_device);
            const VkSurfaceFormatKHR surface_format = choose_surface_format(support.formats);
            const VkPresentModeKHR present_mode = choose_present_mode(support.present_modes);
            const VkExtent2D extent = choose_extent(support.capabilities);

            uint32_t image_count = support.capabilities.minImageCount + 1u;
            if (support.capabilities.maxImageCount > 0 && image_count > support.capabilities.maxImageCount)
                image_count = support.capabilities.maxImageCount;
            m_min_image_count = support.capabilities.minImageCount;

            VkImageUsageFlags image_usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
            m_swapchain_supports_transfer_src =
                (support.capabilities.supportedUsageFlags & VK_IMAGE_USAGE_TRANSFER_SRC_BIT) != 0;
            m_swapchain_supports_transfer_dst =
                (support.capabilities.supportedUsageFlags & VK_IMAGE_USAGE_TRANSFER_DST_BIT) != 0;
            if (m_swapchain_supports_transfer_src)
                image_usage |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
            if (!m_swapchain_supports_transfer_dst)
                throw VulkanError("Swapchain does not support transfer destination images required by the Vulkan renderer");
            image_usage |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;

            VkSwapchainCreateInfoKHR create_info{};
            create_info.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
            create_info.surface = m_surface;
            create_info.minImageCount = image_count;
            create_info.imageFormat = surface_format.format;
            create_info.imageColorSpace = surface_format.colorSpace;
            create_info.imageExtent = extent;
            create_info.imageArrayLayers = 1;
            create_info.imageUsage = image_usage;

            const uint32_t queue_family_indices[] = {m_graphics_queue_family, m_present_queue_family};
            if (m_graphics_queue_family != m_present_queue_family)
            {
                create_info.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
                create_info.queueFamilyIndexCount = 2;
                create_info.pQueueFamilyIndices = queue_family_indices;
            }
            else
            {
                create_info.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
            }

            create_info.preTransform = support.capabilities.currentTransform;
            create_info.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
            create_info.presentMode = present_mode;
            create_info.clipped = VK_TRUE;
            create_info.oldSwapchain = VK_NULL_HANDLE;

            check_vk(vkCreateSwapchainKHR(m_device, &create_info, nullptr, &m_swapchain), "Failed to create swapchain");

            check_vk(vkGetSwapchainImagesKHR(m_device, m_swapchain, &image_count, nullptr), "Failed to get swapchain image count");
            m_swapchain_images.resize(image_count);
            check_vk(vkGetSwapchainImagesKHR(m_device, m_swapchain, &image_count, m_swapchain_images.data()),
                     "Failed to get swapchain images");

            m_swapchain_image_format = surface_format.format;
            m_swapchain_extent = extent;
            m_images_in_flight.assign(m_swapchain_images.size(), VK_NULL_HANDLE);
            spdlog::info("Vulkan: swapchain created {}x{} images={} format={} color_space={} present_mode={} transfer_src={} transfer_dst={}",
                         m_swapchain_extent.width,
                         m_swapchain_extent.height,
                         m_swapchain_images.size(),
                         vk_format_name(m_swapchain_image_format),
                         static_cast<int>(surface_format.colorSpace),
                         present_mode_name(present_mode),
                         m_swapchain_supports_transfer_src,
                         m_swapchain_supports_transfer_dst);
        }

        [[nodiscard]] VkImageView create_image_view(VkImage image, VkFormat format, VkImageAspectFlags aspect_flags) const
        {
            VkImageViewCreateInfo view_info{};
            view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
            view_info.image = image;
            view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
            view_info.format = format;
            view_info.subresourceRange.aspectMask = aspect_flags;
            view_info.subresourceRange.baseMipLevel = 0;
            view_info.subresourceRange.levelCount = 1;
            view_info.subresourceRange.baseArrayLayer = 0;
            view_info.subresourceRange.layerCount = 1;

            VkImageView image_view = VK_NULL_HANDLE;
            check_vk(vkCreateImageView(m_device, &view_info, nullptr, &image_view), "Failed to create image view");
            return image_view;
        }

        void create_image_views()
        {
            m_swapchain_image_views.resize(m_swapchain_images.size());
            for (std::size_t i = 0; i < m_swapchain_images.size(); ++i)
                m_swapchain_image_views[i] = create_image_view(m_swapchain_images[i],
                                                               m_swapchain_image_format,
                                                               VK_IMAGE_ASPECT_COLOR_BIT);
        }

        void create_render_pass()
        {
            const std::vector<VkFormat> color_format_candidates =
                is_bgra_format(m_swapchain_image_format)
                    ? std::vector<VkFormat>{VK_FORMAT_B8G8R8A8_UNORM, VK_FORMAT_R8G8B8A8_UNORM}
                    : std::vector<VkFormat>{VK_FORMAT_R8G8B8A8_UNORM, VK_FORMAT_B8G8R8A8_UNORM};
            m_scene_color_format = find_supported_format(color_format_candidates,
                                                         VK_IMAGE_TILING_OPTIMAL,
                                                         VK_FORMAT_FEATURE_COLOR_ATTACHMENT_BIT
                                                             | VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT
                                                             | VK_FORMAT_FEATURE_STORAGE_IMAGE_BIT);
            m_motion_vector_format = find_supported_format({VK_FORMAT_R16G16_SFLOAT, VK_FORMAT_R32G32_SFLOAT},
                                                           VK_IMAGE_TILING_OPTIMAL,
                                                           VK_FORMAT_FEATURE_COLOR_ATTACHMENT_BIT
                                                               | VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT);

            VkAttachmentDescription color_attachment{};
            color_attachment.format = m_scene_color_format;
            color_attachment.samples = VK_SAMPLE_COUNT_1_BIT;
            color_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
            color_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
            color_attachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
            color_attachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
            color_attachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            color_attachment.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

            VkAttachmentDescription motion_vector_attachment{};
            motion_vector_attachment.format = m_motion_vector_format;
            motion_vector_attachment.samples = VK_SAMPLE_COUNT_1_BIT;
            motion_vector_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
            motion_vector_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
            motion_vector_attachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
            motion_vector_attachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
            motion_vector_attachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            motion_vector_attachment.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

            m_depth_format = find_depth_format();
            VkAttachmentDescription depth_attachment{};
            depth_attachment.format = m_depth_format;
            depth_attachment.samples = VK_SAMPLE_COUNT_1_BIT;
            depth_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
            depth_attachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
            depth_attachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
            depth_attachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
            depth_attachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            depth_attachment.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

            const std::array<VkAttachmentReference, 2> color_attachment_refs{{
                {0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL},
                {1, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL},
            }};
            const VkAttachmentReference depth_attachment_ref{2, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};

            VkSubpassDescription subpass{};
            subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
            subpass.colorAttachmentCount = static_cast<uint32_t>(color_attachment_refs.size());
            subpass.pColorAttachments = color_attachment_refs.data();
            subpass.pDepthStencilAttachment = &depth_attachment_ref;

            VkSubpassDependency dependency{};
            dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
            dependency.dstSubpass = 0;
            dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT
                                    | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
            dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT
                                    | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
            dependency.srcAccessMask = 0;
            dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT
                                     | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

            const std::array<VkAttachmentDescription, 3> attachments{
                color_attachment,
                motion_vector_attachment,
                depth_attachment
            };
            VkRenderPassCreateInfo render_pass_info{};
            render_pass_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
            render_pass_info.attachmentCount = static_cast<uint32_t>(attachments.size());
            render_pass_info.pAttachments = attachments.data();
            render_pass_info.subpassCount = 1;
            render_pass_info.pSubpasses = &subpass;
            render_pass_info.dependencyCount = 1;
            render_pass_info.pDependencies = &dependency;

            check_vk(vkCreateRenderPass(m_device, &render_pass_info, nullptr, &m_render_pass), "Failed to create render pass");

            VkAttachmentDescription present_attachment{};
            present_attachment.format = m_swapchain_image_format;
            present_attachment.samples = VK_SAMPLE_COUNT_1_BIT;
            present_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
            present_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
            present_attachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
            present_attachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
            present_attachment.initialLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
            present_attachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

            const VkAttachmentReference present_attachment_ref{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};

            VkSubpassDescription present_subpass{};
            present_subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
            present_subpass.colorAttachmentCount = 1;
            present_subpass.pColorAttachments = &present_attachment_ref;

            VkSubpassDependency present_dependency{};
            present_dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
            present_dependency.dstSubpass = 0;
            present_dependency.srcStageMask = VK_PIPELINE_STAGE_TRANSFER_BIT;
            present_dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
            present_dependency.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            present_dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

            VkRenderPassCreateInfo present_render_pass_info{};
            present_render_pass_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
            present_render_pass_info.attachmentCount = 1;
            present_render_pass_info.pAttachments = &present_attachment;
            present_render_pass_info.subpassCount = 1;
            present_render_pass_info.pSubpasses = &present_subpass;
            present_render_pass_info.dependencyCount = 1;
            present_render_pass_info.pDependencies = &present_dependency;

            check_vk(vkCreateRenderPass(m_device, &present_render_pass_info, nullptr, &m_present_render_pass),
                     "Failed to create present render pass");
            spdlog::info("Vulkan: render passes created scene_color={} motion={} depth={} present={}",
                         vk_format_name(m_scene_color_format),
                         vk_format_name(m_motion_vector_format),
                         vk_format_name(m_depth_format),
                         vk_format_name(m_swapchain_image_format));
        }

        [[nodiscard]] VkShaderModule create_shader_module(const std::vector<char>& code) const
        {
            VkShaderModuleCreateInfo create_info{};
            create_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
            create_info.codeSize = code.size();
            create_info.pCode = reinterpret_cast<const uint32_t*>(code.data());

            VkShaderModule shader_module = VK_NULL_HANDLE;
            check_vk(vkCreateShaderModule(m_device, &create_info, nullptr, &shader_module), "Failed to create shader module");
            return shader_module;
        }

        void create_descriptor_set_layout()
        {
            VkDescriptorSetLayoutBinding sampler_layout_binding{};
            sampler_layout_binding.binding = 0;
            sampler_layout_binding.descriptorCount = 1;
            sampler_layout_binding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            sampler_layout_binding.pImmutableSamplers = nullptr;
            sampler_layout_binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

            VkDescriptorSetLayoutCreateInfo layout_info{};
            layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
            layout_info.bindingCount = 1;
            layout_info.pBindings = &sampler_layout_binding;
            check_vk(vkCreateDescriptorSetLayout(m_device, &layout_info, nullptr, &m_descriptor_set_layout),
                     "Failed to create descriptor set layout");

            VkDescriptorSetLayoutBinding post_sampler_layout_binding{};
            post_sampler_layout_binding.binding = 0;
            post_sampler_layout_binding.descriptorCount = 1;
            post_sampler_layout_binding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            post_sampler_layout_binding.pImmutableSamplers = nullptr;
            post_sampler_layout_binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

            VkDescriptorSetLayoutCreateInfo post_layout_info{};
            post_layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
            post_layout_info.bindingCount = 1;
            post_layout_info.pBindings = &post_sampler_layout_binding;
            check_vk(vkCreateDescriptorSetLayout(m_device, &post_layout_info, nullptr, &m_post_descriptor_set_layout),
                     "Failed to create post-process descriptor set layout");

            VkSamplerCreateInfo sampler_info{};
            sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
            sampler_info.magFilter = VK_FILTER_LINEAR;
            sampler_info.minFilter = VK_FILTER_LINEAR;
            sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
            sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
            sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
            sampler_info.anisotropyEnable = VK_FALSE;
            sampler_info.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_BLACK;
            sampler_info.unnormalizedCoordinates = VK_FALSE;
            sampler_info.compareEnable = VK_FALSE;
            sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
            check_vk(vkCreateSampler(m_device, &sampler_info, nullptr, &m_bloom_sampler),
                     "Failed to create bloom sampler");

            VkDescriptorSetAllocateInfo alloc_info{};
            alloc_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
            alloc_info.descriptorPool = m_descriptor_pool;
            alloc_info.descriptorSetCount = 1;
            alloc_info.pSetLayouts = &m_post_descriptor_set_layout;
            check_vk(vkAllocateDescriptorSets(m_device, &alloc_info, &m_bloom_descriptor_set),
                     "Failed to allocate bloom descriptor set");
            spdlog::info("Vulkan: descriptor set layouts created");
        }

        void create_graphics_pipeline()
        {
            const std::filesystem::path vert_shader_path = shader_path("shader.vert.spv");
            const std::filesystem::path frag_shader_path = shader_path("shader.frag.spv");
            spdlog::info("Vulkan: creating graphics pipelines using shaders '{}' and '{}'",
                         vert_shader_path.string(),
                         frag_shader_path.string());
            const std::vector<char> vert_shader_code = read_binary_file(vert_shader_path);
            const std::vector<char> frag_shader_code = read_binary_file(frag_shader_path);
            const VkShaderModule vert_shader_module = create_shader_module(vert_shader_code);
            const VkShaderModule frag_shader_module = create_shader_module(frag_shader_code);

            VkPipelineShaderStageCreateInfo vert_shader_stage_info{};
            vert_shader_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
            vert_shader_stage_info.stage = VK_SHADER_STAGE_VERTEX_BIT;
            vert_shader_stage_info.module = vert_shader_module;
            vert_shader_stage_info.pName = "main";

            VkPipelineShaderStageCreateInfo frag_shader_stage_info{};
            frag_shader_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
            frag_shader_stage_info.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
            frag_shader_stage_info.module = frag_shader_module;
            frag_shader_stage_info.pName = "main";

            const VkPipelineShaderStageCreateInfo shader_stages[] = {vert_shader_stage_info, frag_shader_stage_info};

            using VertexType = omath::opengl_engine::Mesh::VertexType;
            constexpr auto vec3_size = static_cast<uint32_t>(sizeof(omath::Vector3<float>));
            const VkVertexInputBindingDescription binding_description{0, sizeof(VertexType), VK_VERTEX_INPUT_RATE_VERTEX};
            const std::array<VkVertexInputAttributeDescription, 3> attribute_descriptions{{
                {0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},
                {1, 0, VK_FORMAT_R32G32B32_SFLOAT, vec3_size},
                {2, 0, VK_FORMAT_R32G32_SFLOAT, 2u * vec3_size},
            }};

            VkPipelineVertexInputStateCreateInfo vertex_input_info{};
            vertex_input_info.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
            vertex_input_info.vertexBindingDescriptionCount = 1;
            vertex_input_info.pVertexBindingDescriptions = &binding_description;
            vertex_input_info.vertexAttributeDescriptionCount = static_cast<uint32_t>(attribute_descriptions.size());
            vertex_input_info.pVertexAttributeDescriptions = attribute_descriptions.data();

            VkPipelineInputAssemblyStateCreateInfo input_assembly{};
            input_assembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
            input_assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
            input_assembly.primitiveRestartEnable = VK_FALSE;

            VkPipelineViewportStateCreateInfo viewport_state{};
            viewport_state.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
            viewport_state.viewportCount = 1;
            viewport_state.scissorCount = 1;

            VkPipelineRasterizationStateCreateInfo rasterizer{};
            rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
            rasterizer.depthClampEnable = VK_FALSE;
            rasterizer.rasterizerDiscardEnable = VK_FALSE;
            rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
            rasterizer.lineWidth = 1.0f;
            rasterizer.cullMode = VK_CULL_MODE_BACK_BIT;
            rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
            rasterizer.depthBiasEnable = VK_FALSE;

            VkPipelineMultisampleStateCreateInfo multisampling{};
            multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
            multisampling.sampleShadingEnable = VK_FALSE;
            multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

            VkPipelineDepthStencilStateCreateInfo depth_stencil{};
            depth_stencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
            depth_stencil.depthTestEnable = VK_TRUE;
            depth_stencil.depthWriteEnable = VK_TRUE;
            depth_stencil.depthCompareOp = VK_COMPARE_OP_LESS;
            depth_stencil.depthBoundsTestEnable = VK_FALSE;
            depth_stencil.stencilTestEnable = VK_FALSE;

            VkPipelineColorBlendAttachmentState color_blend_attachment{};
            color_blend_attachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT
                                                  | VK_COLOR_COMPONENT_G_BIT
                                                  | VK_COLOR_COMPONENT_B_BIT
                                                  | VK_COLOR_COMPONENT_A_BIT;
            color_blend_attachment.blendEnable = VK_FALSE;
            VkPipelineColorBlendAttachmentState motion_blend_attachment{};
            motion_blend_attachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT;
            motion_blend_attachment.blendEnable = VK_FALSE;
            const std::array<VkPipelineColorBlendAttachmentState, 2> color_blend_attachments{
                color_blend_attachment,
                motion_blend_attachment
            };

            VkPipelineColorBlendStateCreateInfo color_blending{};
            color_blending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
            color_blending.logicOpEnable = VK_FALSE;
            color_blending.attachmentCount = static_cast<uint32_t>(color_blend_attachments.size());
            color_blending.pAttachments = color_blend_attachments.data();

            const VkDynamicState dynamic_states[] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
            VkPipelineDynamicStateCreateInfo dynamic_state{};
            dynamic_state.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
            dynamic_state.dynamicStateCount = 2;
            dynamic_state.pDynamicStates = dynamic_states;

            VkPushConstantRange push_constant_range{};
            push_constant_range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
            push_constant_range.offset = 0;
            push_constant_range.size = sizeof(PushConstants);

            VkPhysicalDeviceProperties device_properties{};
            vkGetPhysicalDeviceProperties(m_physical_device, &device_properties);
            if (device_properties.limits.maxPushConstantsSize < static_cast<uint32_t>(sizeof(PushConstants)))
                throw VulkanError("Vulkan device does not support the push constant size required for motion vectors");

            VkPipelineLayoutCreateInfo pipeline_layout_info{};
            pipeline_layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
            pipeline_layout_info.setLayoutCount = 1;
            pipeline_layout_info.pSetLayouts = &m_descriptor_set_layout;
            pipeline_layout_info.pushConstantRangeCount = 1;
            pipeline_layout_info.pPushConstantRanges = &push_constant_range;

            check_vk(vkCreatePipelineLayout(m_device, &pipeline_layout_info, nullptr, &m_pipeline_layout),
                     "Failed to create pipeline layout");

            VkGraphicsPipelineCreateInfo pipeline_info{};
            pipeline_info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
            pipeline_info.stageCount = 2;
            pipeline_info.pStages = shader_stages;
            pipeline_info.pVertexInputState = &vertex_input_info;
            pipeline_info.pInputAssemblyState = &input_assembly;
            pipeline_info.pViewportState = &viewport_state;
            pipeline_info.pRasterizationState = &rasterizer;
            pipeline_info.pMultisampleState = &multisampling;
            pipeline_info.pDepthStencilState = &depth_stencil;
            pipeline_info.pColorBlendState = &color_blending;
            pipeline_info.pDynamicState = &dynamic_state;
            pipeline_info.layout = m_pipeline_layout;
            pipeline_info.renderPass = m_render_pass;
            pipeline_info.subpass = 0;

            check_vk(vkCreateGraphicsPipelines(m_device, VK_NULL_HANDLE, 1, &pipeline_info, nullptr, &m_graphics_pipeline),
                     "Failed to create graphics pipeline");

            VkPipelineRasterizationStateCreateInfo outline_rasterizer = rasterizer;
            outline_rasterizer.cullMode = VK_CULL_MODE_FRONT_BIT;

            VkPipelineDepthStencilStateCreateInfo outline_depth_stencil = depth_stencil;
            outline_depth_stencil.depthWriteEnable = VK_FALSE;
            outline_depth_stencil.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;

            VkPipelineColorBlendAttachmentState outline_color_blend_attachment = color_blend_attachment;
            outline_color_blend_attachment.blendEnable = VK_TRUE;
            outline_color_blend_attachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
            outline_color_blend_attachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE;
            outline_color_blend_attachment.colorBlendOp = VK_BLEND_OP_ADD;
            outline_color_blend_attachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
            outline_color_blend_attachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
            outline_color_blend_attachment.alphaBlendOp = VK_BLEND_OP_ADD;

            VkPipelineColorBlendAttachmentState outline_motion_blend_attachment = motion_blend_attachment;
            outline_motion_blend_attachment.colorWriteMask = 0;
            const std::array<VkPipelineColorBlendAttachmentState, 2> outline_color_blend_attachments{
                outline_color_blend_attachment,
                outline_motion_blend_attachment
            };

            VkPipelineColorBlendStateCreateInfo outline_color_blending = color_blending;
            outline_color_blending.pAttachments = outline_color_blend_attachments.data();

            pipeline_info.pRasterizationState = &outline_rasterizer;
            pipeline_info.pDepthStencilState = &outline_depth_stencil;
            pipeline_info.pColorBlendState = &outline_color_blending;

            check_vk(vkCreateGraphicsPipelines(m_device, VK_NULL_HANDLE, 1, &pipeline_info, nullptr, &m_outline_pipeline),
                     "Failed to create outline pipeline");

            const std::filesystem::path post_vert_shader_path = shader_path("post.vert.spv");
            const std::filesystem::path bloom_frag_shader_path = shader_path("bloom.frag.spv");
            spdlog::info("Vulkan: creating bloom pipeline using shaders '{}' and '{}'",
                         post_vert_shader_path.string(),
                         bloom_frag_shader_path.string());
            const std::vector<char> post_vert_shader_code = read_binary_file(post_vert_shader_path);
            const std::vector<char> bloom_frag_shader_code = read_binary_file(bloom_frag_shader_path);
            const VkShaderModule post_vert_shader_module = create_shader_module(post_vert_shader_code);
            const VkShaderModule bloom_frag_shader_module = create_shader_module(bloom_frag_shader_code);

            VkPipelineShaderStageCreateInfo post_vert_shader_stage_info{};
            post_vert_shader_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
            post_vert_shader_stage_info.stage = VK_SHADER_STAGE_VERTEX_BIT;
            post_vert_shader_stage_info.module = post_vert_shader_module;
            post_vert_shader_stage_info.pName = "main";

            VkPipelineShaderStageCreateInfo bloom_frag_shader_stage_info{};
            bloom_frag_shader_stage_info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
            bloom_frag_shader_stage_info.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
            bloom_frag_shader_stage_info.module = bloom_frag_shader_module;
            bloom_frag_shader_stage_info.pName = "main";

            const VkPipelineShaderStageCreateInfo bloom_shader_stages[] = {
                post_vert_shader_stage_info,
                bloom_frag_shader_stage_info
            };

            VkPipelineVertexInputStateCreateInfo bloom_vertex_input_info{};
            bloom_vertex_input_info.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;

            VkPipelineRasterizationStateCreateInfo bloom_rasterizer = rasterizer;
            bloom_rasterizer.cullMode = VK_CULL_MODE_NONE;

            VkPipelineDepthStencilStateCreateInfo bloom_depth_stencil{};
            bloom_depth_stencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
            bloom_depth_stencil.depthTestEnable = VK_FALSE;
            bloom_depth_stencil.depthWriteEnable = VK_FALSE;
            bloom_depth_stencil.depthBoundsTestEnable = VK_FALSE;
            bloom_depth_stencil.stencilTestEnable = VK_FALSE;

            VkPipelineColorBlendStateCreateInfo bloom_color_blending{};
            bloom_color_blending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
            bloom_color_blending.logicOpEnable = VK_FALSE;
            bloom_color_blending.attachmentCount = 1;
            bloom_color_blending.pAttachments = &color_blend_attachment;

            VkPushConstantRange bloom_push_constant_range{};
            bloom_push_constant_range.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
            bloom_push_constant_range.offset = 0;
            bloom_push_constant_range.size = sizeof(BloomPushConstants);

            VkPipelineLayoutCreateInfo bloom_pipeline_layout_info{};
            bloom_pipeline_layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
            bloom_pipeline_layout_info.setLayoutCount = 1;
            bloom_pipeline_layout_info.pSetLayouts = &m_post_descriptor_set_layout;
            bloom_pipeline_layout_info.pushConstantRangeCount = 1;
            bloom_pipeline_layout_info.pPushConstantRanges = &bloom_push_constant_range;

            check_vk(vkCreatePipelineLayout(m_device,
                                            &bloom_pipeline_layout_info,
                                            nullptr,
                                            &m_bloom_pipeline_layout),
                     "Failed to create bloom pipeline layout");

            VkGraphicsPipelineCreateInfo bloom_pipeline_info{};
            bloom_pipeline_info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
            bloom_pipeline_info.stageCount = 2;
            bloom_pipeline_info.pStages = bloom_shader_stages;
            bloom_pipeline_info.pVertexInputState = &bloom_vertex_input_info;
            bloom_pipeline_info.pInputAssemblyState = &input_assembly;
            bloom_pipeline_info.pViewportState = &viewport_state;
            bloom_pipeline_info.pRasterizationState = &bloom_rasterizer;
            bloom_pipeline_info.pMultisampleState = &multisampling;
            bloom_pipeline_info.pDepthStencilState = &bloom_depth_stencil;
            bloom_pipeline_info.pColorBlendState = &bloom_color_blending;
            bloom_pipeline_info.pDynamicState = &dynamic_state;
            bloom_pipeline_info.layout = m_bloom_pipeline_layout;
            bloom_pipeline_info.renderPass = m_present_render_pass;
            bloom_pipeline_info.subpass = 0;

            check_vk(vkCreateGraphicsPipelines(m_device,
                                               VK_NULL_HANDLE,
                                               1,
                                               &bloom_pipeline_info,
                                               nullptr,
                                               &m_bloom_pipeline),
                     "Failed to create bloom pipeline");
            spdlog::info("Vulkan: graphics pipelines created");

            vkDestroyShaderModule(m_device, bloom_frag_shader_module, nullptr);
            vkDestroyShaderModule(m_device, post_vert_shader_module, nullptr);
            vkDestroyShaderModule(m_device, frag_shader_module, nullptr);
            vkDestroyShaderModule(m_device, vert_shader_module, nullptr);
        }

        [[nodiscard]] VkFormat find_supported_format(const std::vector<VkFormat>& candidates,
                                                     VkImageTiling tiling,
                                                     VkFormatFeatureFlags features) const
        {
            for (VkFormat format : candidates)
            {
                VkFormatProperties properties{};
                vkGetPhysicalDeviceFormatProperties(m_physical_device, format, &properties);
                if (tiling == VK_IMAGE_TILING_LINEAR && (properties.linearTilingFeatures & features) == features)
                    return format;
                if (tiling == VK_IMAGE_TILING_OPTIMAL && (properties.optimalTilingFeatures & features) == features)
                    return format;
            }
            throw VulkanError("Failed to find a supported Vulkan image format");
        }

        [[nodiscard]] VkFormat find_depth_format() const
        {
            return find_supported_format({VK_FORMAT_D32_SFLOAT,
                                          VK_FORMAT_D32_SFLOAT_S8_UINT,
                                          VK_FORMAT_D24_UNORM_S8_UINT},
                                         VK_IMAGE_TILING_OPTIMAL,
                                         VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT);
        }

        void create_render_targets()
        {
            m_scene_extent = choose_scene_extent();
            spdlog::info("Vulkan: creating render targets scene={}x{} swapchain={}x{} dlss_active={}",
                         m_scene_extent.width,
                         m_scene_extent.height,
                         m_swapchain_extent.width,
                         m_swapchain_extent.height,
                         dlss_active());

            create_image(m_scene_extent.width,
                         m_scene_extent.height,
                         m_scene_color_format,
                         VK_IMAGE_TILING_OPTIMAL,
                         VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT
                             | VK_IMAGE_USAGE_TRANSFER_SRC_BIT
                             | VK_IMAGE_USAGE_SAMPLED_BIT,
                         VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                         m_scene_color_image);
            m_scene_color_image.view = create_image_view(m_scene_color_image.image,
                                                         m_scene_color_format,
                                                         VK_IMAGE_ASPECT_COLOR_BIT);

            create_image(m_scene_extent.width,
                         m_scene_extent.height,
                         m_motion_vector_format,
                         VK_IMAGE_TILING_OPTIMAL,
                         VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT
                             | VK_IMAGE_USAGE_SAMPLED_BIT,
                         VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                         m_motion_vector_image);
            m_motion_vector_image.view = create_image_view(m_motion_vector_image.image,
                                                           m_motion_vector_format,
                                                           VK_IMAGE_ASPECT_COLOR_BIT);

            create_image(m_scene_extent.width,
                         m_scene_extent.height,
                         m_depth_format,
                         VK_IMAGE_TILING_OPTIMAL,
                         VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT
                             | VK_IMAGE_USAGE_SAMPLED_BIT,
                         VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                         m_depth_image);
            m_depth_image.view = create_image_view(m_depth_image.image, m_depth_format, VK_IMAGE_ASPECT_DEPTH_BIT);

            if (dlss_active())
            {
                create_image(m_swapchain_extent.width,
                             m_swapchain_extent.height,
                             m_scene_color_format,
                             VK_IMAGE_TILING_OPTIMAL,
                             VK_IMAGE_USAGE_STORAGE_BIT
                                 | VK_IMAGE_USAGE_TRANSFER_SRC_BIT
                                 | VK_IMAGE_USAGE_TRANSFER_DST_BIT
                                 | VK_IMAGE_USAGE_SAMPLED_BIT,
                             VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                             m_dlss_output_image);
                m_dlss_output_image.view = create_image_view(m_dlss_output_image.image,
                                                             m_scene_color_format,
                                                             VK_IMAGE_ASPECT_COLOR_BIT);
            }
            spdlog::info("Vulkan: render targets created");
        }

        void create_framebuffers()
        {
            const std::array<VkImageView, 3> scene_attachments{
                m_scene_color_image.view,
                m_motion_vector_image.view,
                m_depth_image.view
            };

            VkFramebufferCreateInfo scene_framebuffer_info{};
            scene_framebuffer_info.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
            scene_framebuffer_info.renderPass = m_render_pass;
            scene_framebuffer_info.attachmentCount = static_cast<uint32_t>(scene_attachments.size());
            scene_framebuffer_info.pAttachments = scene_attachments.data();
            scene_framebuffer_info.width = m_scene_extent.width;
            scene_framebuffer_info.height = m_scene_extent.height;
            scene_framebuffer_info.layers = 1;

            check_vk(vkCreateFramebuffer(m_device, &scene_framebuffer_info, nullptr, &m_scene_framebuffer),
                     "Failed to create scene framebuffer");

            m_framebuffers.resize(m_swapchain_image_views.size());
            for (std::size_t i = 0; i < m_swapchain_image_views.size(); ++i)
            {
                const VkImageView attachment = m_swapchain_image_views[i];

                VkFramebufferCreateInfo framebuffer_info{};
                framebuffer_info.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
                framebuffer_info.renderPass = m_present_render_pass;
                framebuffer_info.attachmentCount = 1;
                framebuffer_info.pAttachments = &attachment;
                framebuffer_info.width = m_swapchain_extent.width;
                framebuffer_info.height = m_swapchain_extent.height;
                framebuffer_info.layers = 1;

                check_vk(vkCreateFramebuffer(m_device, &framebuffer_info, nullptr, &m_framebuffers[i]),
                         "Failed to create framebuffer");
            }
        }

        void create_command_pool()
        {
            const QueueFamilies queue_families = find_queue_families(m_physical_device);

            VkCommandPoolCreateInfo pool_info{};
            pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
            pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
            pool_info.queueFamilyIndex = queue_families.graphics.value();

            check_vk(vkCreateCommandPool(m_device, &pool_info, nullptr, &m_command_pool), "Failed to create command pool");
        }

        void create_command_buffers()
        {
            m_command_buffers.resize(m_framebuffers.size());

            VkCommandBufferAllocateInfo alloc_info{};
            alloc_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
            alloc_info.commandPool = m_command_pool;
            alloc_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
            alloc_info.commandBufferCount = static_cast<uint32_t>(m_command_buffers.size());

            check_vk(vkAllocateCommandBuffers(m_device, &alloc_info, m_command_buffers.data()),
                     "Failed to allocate command buffers");
        }

        void create_sync_objects()
        {
            VkSemaphoreCreateInfo semaphore_info{};
            semaphore_info.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

            VkFenceCreateInfo fence_info{};
            fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
            fence_info.flags = VK_FENCE_CREATE_SIGNALED_BIT;

            for (FrameSync& frame : m_frames)
            {
                check_vk(vkCreateSemaphore(m_device, &semaphore_info, nullptr, &frame.image_available),
                         "Failed to create image-available semaphore");
                check_vk(vkCreateSemaphore(m_device, &semaphore_info, nullptr, &frame.render_finished),
                         "Failed to create render-finished semaphore");
                check_vk(vkCreateFence(m_device, &fence_info, nullptr, &frame.in_flight), "Failed to create frame fence");
            }
        }

        void create_descriptor_pool()
        {
            const std::array<VkDescriptorPoolSize, 11> pool_sizes{{
                {VK_DESCRIPTOR_TYPE_SAMPLER, 1024},
                {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 4096},
                {VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1024},
                {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1024},
                {VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1024},
                {VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1024},
                {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1024},
                {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1024},
                {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1024},
                {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1024},
                {VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1024},
            }};

            VkDescriptorPoolCreateInfo pool_info{};
            pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
            pool_info.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
            pool_info.maxSets = 4096;
            pool_info.poolSizeCount = static_cast<uint32_t>(pool_sizes.size());
            pool_info.pPoolSizes = pool_sizes.data();

            check_vk(vkCreateDescriptorPool(m_device, &pool_info, nullptr, &m_descriptor_pool),
                     "Failed to create descriptor pool");
        }

        void init_imgui()
        {
            ImGui_ImplGlfw_InitForVulkan(m_window, true);

            ImGui_ImplVulkan_InitInfo init_info{};
            init_info.ApiVersion = k_api_version;
            init_info.Instance = m_instance;
            init_info.PhysicalDevice = m_physical_device;
            init_info.Device = m_device;
            init_info.QueueFamily = m_graphics_queue_family;
            init_info.Queue = m_graphics_queue;
            init_info.DescriptorPool = m_descriptor_pool;
            init_info.RenderPass = m_present_render_pass;
            init_info.MinImageCount = m_min_image_count;
            init_info.ImageCount = static_cast<uint32_t>(m_swapchain_images.size());
            init_info.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
            init_info.CheckVkResultFn = check_imgui_vk_result;
            ImGui_ImplVulkan_Init(&init_info);
        }

        [[nodiscard]] uint32_t find_memory_type(uint32_t type_filter, VkMemoryPropertyFlags properties) const
        {
            VkPhysicalDeviceMemoryProperties memory_properties{};
            vkGetPhysicalDeviceMemoryProperties(m_physical_device, &memory_properties);

            for (uint32_t i = 0; i < memory_properties.memoryTypeCount; ++i)
            {
                if ((type_filter & (1u << i)) != 0
                    && (memory_properties.memoryTypes[i].propertyFlags & properties) == properties)
                    return i;
            }

            throw VulkanError("Failed to find suitable Vulkan memory type");
        }

        void create_buffer(VkDeviceSize size,
                           VkBufferUsageFlags usage,
                           VkMemoryPropertyFlags properties,
                           BufferResource& buffer) const
        {
            buffer.size = size;

            VkBufferCreateInfo buffer_info{};
            buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
            buffer_info.size = size;
            buffer_info.usage = usage;
            buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

            check_vk(vkCreateBuffer(m_device, &buffer_info, nullptr, &buffer.buffer), "Failed to create buffer");

            VkMemoryRequirements memory_requirements{};
            vkGetBufferMemoryRequirements(m_device, buffer.buffer, &memory_requirements);

            VkMemoryAllocateInfo alloc_info{};
            alloc_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
            alloc_info.allocationSize = memory_requirements.size;
            alloc_info.memoryTypeIndex = find_memory_type(memory_requirements.memoryTypeBits, properties);

            check_vk(vkAllocateMemory(m_device, &alloc_info, nullptr, &buffer.memory), "Failed to allocate buffer memory");
            check_vk(vkBindBufferMemory(m_device, buffer.buffer, buffer.memory, 0), "Failed to bind buffer memory");
        }

        void destroy_buffer(BufferResource& buffer) const noexcept
        {
            if (buffer.buffer != VK_NULL_HANDLE)
                vkDestroyBuffer(m_device, buffer.buffer, nullptr);
            if (buffer.memory != VK_NULL_HANDLE)
                vkFreeMemory(m_device, buffer.memory, nullptr);
            buffer = {};
        }

        [[nodiscard]] VkCommandBuffer begin_single_time_commands() const
        {
            VkCommandBufferAllocateInfo alloc_info{};
            alloc_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
            alloc_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
            alloc_info.commandPool = m_command_pool;
            alloc_info.commandBufferCount = 1;

            VkCommandBuffer command_buffer = VK_NULL_HANDLE;
            check_vk(vkAllocateCommandBuffers(m_device, &alloc_info, &command_buffer),
                     "Failed to allocate upload command buffer");

            VkCommandBufferBeginInfo begin_info{};
            begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
            begin_info.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

            check_vk(vkBeginCommandBuffer(command_buffer, &begin_info), "Failed to begin upload command buffer");
            return command_buffer;
        }

        void end_single_time_commands(VkCommandBuffer command_buffer) const
        {
            check_vk(vkEndCommandBuffer(command_buffer), "Failed to end upload command buffer");

            VkSubmitInfo submit_info{};
            submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
            submit_info.commandBufferCount = 1;
            submit_info.pCommandBuffers = &command_buffer;

            check_vk(vkQueueSubmit(m_graphics_queue, 1, &submit_info, VK_NULL_HANDLE), "Failed to submit upload command buffer");
            check_vk(vkQueueWaitIdle(m_graphics_queue), "Failed to wait for upload queue");
            vkFreeCommandBuffers(m_device, m_command_pool, 1, &command_buffer);
        }

        void copy_buffer(VkBuffer src_buffer, VkBuffer dst_buffer, VkDeviceSize size) const
        {
            VkCommandBuffer command_buffer = begin_single_time_commands();

            VkBufferCopy copy_region{};
            copy_region.size = size;
            vkCmdCopyBuffer(command_buffer, src_buffer, dst_buffer, 1, &copy_region);

            end_single_time_commands(command_buffer);
        }

        void create_device_buffer(const void* data,
                                  VkDeviceSize size,
                                  VkBufferUsageFlags usage,
                                  BufferResource& out_buffer) const
        {
            BufferResource staging_buffer;
            create_buffer(size,
                          VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                          staging_buffer);

            void* mapped = nullptr;
            check_vk(vkMapMemory(m_device, staging_buffer.memory, 0, size, 0, &mapped), "Failed to map staging buffer");
            std::memcpy(mapped, data, static_cast<std::size_t>(size));
            vkUnmapMemory(m_device, staging_buffer.memory);

            create_buffer(size,
                          VK_BUFFER_USAGE_TRANSFER_DST_BIT | usage,
                          VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                          out_buffer);
            copy_buffer(staging_buffer.buffer, out_buffer.buffer, size);
            destroy_buffer(staging_buffer);
        }

        void create_image(uint32_t width,
                          uint32_t height,
                          VkFormat format,
                          VkImageTiling tiling,
                          VkImageUsageFlags usage,
                          VkMemoryPropertyFlags properties,
                          ImageResource& image) const
        {
            image.format = format;
            image.extent = {width, height};
            image.layout = VK_IMAGE_LAYOUT_UNDEFINED;

            VkImageCreateInfo image_info{};
            image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
            image_info.imageType = VK_IMAGE_TYPE_2D;
            image_info.extent.width = width;
            image_info.extent.height = height;
            image_info.extent.depth = 1;
            image_info.mipLevels = 1;
            image_info.arrayLayers = 1;
            image_info.format = format;
            image_info.tiling = tiling;
            image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            image_info.usage = usage;
            image_info.samples = VK_SAMPLE_COUNT_1_BIT;
            image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

            check_vk(vkCreateImage(m_device, &image_info, nullptr, &image.image), "Failed to create image");

            VkMemoryRequirements memory_requirements{};
            vkGetImageMemoryRequirements(m_device, image.image, &memory_requirements);

            VkMemoryAllocateInfo alloc_info{};
            alloc_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
            alloc_info.allocationSize = memory_requirements.size;
            alloc_info.memoryTypeIndex = find_memory_type(memory_requirements.memoryTypeBits, properties);

            check_vk(vkAllocateMemory(m_device, &alloc_info, nullptr, &image.memory), "Failed to allocate image memory");
            check_vk(vkBindImageMemory(m_device, image.image, image.memory, 0), "Failed to bind image memory");
        }

        void destroy_image(ImageResource& image) const noexcept
        {
            if (image.view != VK_NULL_HANDLE)
                vkDestroyImageView(m_device, image.view, nullptr);
            if (image.image != VK_NULL_HANDLE)
                vkDestroyImage(m_device, image.image, nullptr);
            if (image.memory != VK_NULL_HANDLE)
                vkFreeMemory(m_device, image.memory, nullptr);
            image = {};
        }

        void transition_image_layout(VkImage image,
                                     VkFormat,
                                     VkImageLayout old_layout,
                                     VkImageLayout new_layout) const
        {
            VkCommandBuffer command_buffer = begin_single_time_commands();

            VkImageMemoryBarrier barrier{};
            barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
            barrier.oldLayout = old_layout;
            barrier.newLayout = new_layout;
            barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.image = image;
            barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            barrier.subresourceRange.baseMipLevel = 0;
            barrier.subresourceRange.levelCount = 1;
            barrier.subresourceRange.baseArrayLayer = 0;
            barrier.subresourceRange.layerCount = 1;

            VkPipelineStageFlags source_stage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
            VkPipelineStageFlags destination_stage = VK_PIPELINE_STAGE_TRANSFER_BIT;

            if (old_layout == VK_IMAGE_LAYOUT_UNDEFINED && new_layout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL)
            {
                barrier.srcAccessMask = 0;
                barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            }
            else if (old_layout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL
                     && new_layout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)
            {
                barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
                barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
                source_stage = VK_PIPELINE_STAGE_TRANSFER_BIT;
                destination_stage = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
            }
            else
            {
                throw VulkanError("Unsupported image layout transition");
            }

            vkCmdPipelineBarrier(command_buffer,
                                 source_stage,
                                 destination_stage,
                                 0,
                                 0,
                                 nullptr,
                                 0,
                                 nullptr,
                                 1,
                                 &barrier);

            end_single_time_commands(command_buffer);
        }

        void record_image_barrier(ImageResource& image,
                                  VkImageAspectFlags aspect_mask,
                                  VkImageLayout new_layout,
                                  VkAccessFlags src_access,
                                  VkAccessFlags dst_access,
                                  VkPipelineStageFlags src_stage,
                                  VkPipelineStageFlags dst_stage) const
        {
            VkImageMemoryBarrier barrier{};
            barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
            barrier.oldLayout = image.layout;
            barrier.newLayout = new_layout;
            barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.image = image.image;
            barrier.subresourceRange.aspectMask = aspect_mask;
            barrier.subresourceRange.baseMipLevel = 0;
            barrier.subresourceRange.levelCount = 1;
            barrier.subresourceRange.baseArrayLayer = 0;
            barrier.subresourceRange.layerCount = 1;
            barrier.srcAccessMask = src_access;
            barrier.dstAccessMask = dst_access;
            if (image.layout == VK_IMAGE_LAYOUT_UNDEFINED)
            {
                barrier.srcAccessMask = 0;
                src_stage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
            }

            vkCmdPipelineBarrier(m_active_command_buffer,
                                 src_stage,
                                 dst_stage,
                                 0,
                                 0,
                                 nullptr,
                                 0,
                                 nullptr,
                                 1,
                                 &barrier);
            image.layout = new_layout;
        }

        void record_swapchain_barrier(VkImageLayout old_layout,
                                      VkImageLayout new_layout,
                                      VkAccessFlags src_access,
                                      VkAccessFlags dst_access,
                                      VkPipelineStageFlags src_stage,
                                      VkPipelineStageFlags dst_stage) const
        {
            VkImageMemoryBarrier barrier{};
            barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
            barrier.oldLayout = old_layout;
            barrier.newLayout = new_layout;
            barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.image = m_swapchain_images[m_active_image_index];
            barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            barrier.subresourceRange.baseMipLevel = 0;
            barrier.subresourceRange.levelCount = 1;
            barrier.subresourceRange.baseArrayLayer = 0;
            barrier.subresourceRange.layerCount = 1;
            barrier.srcAccessMask = src_access;
            barrier.dstAccessMask = dst_access;

            vkCmdPipelineBarrier(m_active_command_buffer,
                                 src_stage,
                                 dst_stage,
                                 0,
                                 0,
                                 nullptr,
                                 0,
                                 nullptr,
                                 1,
                                 &barrier);
        }

        void blit_to_swapchain(ImageResource& source)
        {
            record_image_barrier(source,
                                 VK_IMAGE_ASPECT_COLOR_BIT,
                                 VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                                 VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
                                 VK_ACCESS_TRANSFER_READ_BIT,
                                 VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                 VK_PIPELINE_STAGE_TRANSFER_BIT);
            record_swapchain_barrier(VK_IMAGE_LAYOUT_UNDEFINED,
                                     VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                     0,
                                     VK_ACCESS_TRANSFER_WRITE_BIT,
                                     VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                                     VK_PIPELINE_STAGE_TRANSFER_BIT);

            VkImageBlit blit{};
            blit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            blit.srcSubresource.mipLevel = 0;
            blit.srcSubresource.baseArrayLayer = 0;
            blit.srcSubresource.layerCount = 1;
            blit.srcOffsets[0] = {0, 0, 0};
            blit.srcOffsets[1] = {
                static_cast<int32_t>(source.extent.width),
                static_cast<int32_t>(source.extent.height),
                1
            };
            blit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            blit.dstSubresource.mipLevel = 0;
            blit.dstSubresource.baseArrayLayer = 0;
            blit.dstSubresource.layerCount = 1;
            blit.dstOffsets[0] = {0, 0, 0};
            blit.dstOffsets[1] = {
                static_cast<int32_t>(m_swapchain_extent.width),
                static_cast<int32_t>(m_swapchain_extent.height),
                1
            };

            vkCmdBlitImage(m_active_command_buffer,
                           source.image,
                           VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                           m_swapchain_images[m_active_image_index],
                           VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                           1,
                           &blit,
                           VK_FILTER_LINEAR);

            record_swapchain_barrier(VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                     VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                                     VK_ACCESS_TRANSFER_WRITE_BIT,
                                     VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
                                     VK_PIPELINE_STAGE_TRANSFER_BIT,
                                     VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT);
        }

        void update_bloom_descriptor(const ImageResource& source)
        {
            if (m_bloom_descriptor_image_view == source.view)
                return;

            VkDescriptorImageInfo image_info{};
            image_info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            image_info.imageView = source.view;
            image_info.sampler = m_bloom_sampler;

            VkWriteDescriptorSet descriptor_write{};
            descriptor_write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            descriptor_write.dstSet = m_bloom_descriptor_set;
            descriptor_write.dstBinding = 0;
            descriptor_write.dstArrayElement = 0;
            descriptor_write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            descriptor_write.descriptorCount = 1;
            descriptor_write.pImageInfo = &image_info;

            vkUpdateDescriptorSets(m_device, 1, &descriptor_write, 0, nullptr);
            m_bloom_descriptor_image_view = source.view;
        }

        void render_bloom_to_swapchain(ImageResource& source)
        {
            record_image_barrier(source,
                                 VK_IMAGE_ASPECT_COLOR_BIT,
                                 VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                 VK_ACCESS_SHADER_WRITE_BIT
                                     | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT
                                     | VK_ACCESS_TRANSFER_WRITE_BIT,
                                 VK_ACCESS_SHADER_READ_BIT,
                                 VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT
                                     | VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT
                                     | VK_PIPELINE_STAGE_TRANSFER_BIT,
                                 VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);

            record_swapchain_barrier(VK_IMAGE_LAYOUT_UNDEFINED,
                                     VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                     0,
                                     VK_ACCESS_TRANSFER_WRITE_BIT,
                                     VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                                     VK_PIPELINE_STAGE_TRANSFER_BIT);
            constexpr VkClearColorValue clear_color{{0.0f, 0.0f, 0.0f, 1.0f}};
            const VkImageSubresourceRange color_range{
                VK_IMAGE_ASPECT_COLOR_BIT,
                0,
                1,
                0,
                1
            };
            vkCmdClearColorImage(m_active_command_buffer,
                                 m_swapchain_images[m_active_image_index],
                                 VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                 &clear_color,
                                 1,
                                 &color_range);
            record_swapchain_barrier(VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                     VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                                     VK_ACCESS_TRANSFER_WRITE_BIT,
                                     VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
                                     VK_PIPELINE_STAGE_TRANSFER_BIT,
                                     VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT);

            update_bloom_descriptor(source);
            begin_present_render_pass();

            VkViewport viewport{};
            viewport.x = 0.0f;
            viewport.y = 0.0f;
            viewport.width = static_cast<float>(m_swapchain_extent.width);
            viewport.height = static_cast<float>(m_swapchain_extent.height);
            viewport.minDepth = 0.0f;
            viewport.maxDepth = 1.0f;
            vkCmdSetViewport(m_active_command_buffer, 0, 1, &viewport);

            const VkRect2D scissor{{0, 0}, m_swapchain_extent};
            vkCmdSetScissor(m_active_command_buffer, 0, 1, &scissor);

            BloomPushConstants push{};
            push.texel_size[0] = 1.0f / static_cast<float>(source.extent.width);
            push.texel_size[1] = 1.0f / static_cast<float>(source.extent.height);
            push.threshold = m_bloom_settings.threshold;
            push.intensity = m_bloom_settings.intensity;
            push.radius = m_bloom_settings.radius;
            push.quality = m_bloom_settings.quality;

            vkCmdBindPipeline(m_active_command_buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_bloom_pipeline);
            vkCmdBindDescriptorSets(m_active_command_buffer,
                                    VK_PIPELINE_BIND_POINT_GRAPHICS,
                                    m_bloom_pipeline_layout,
                                    0,
                                    1,
                                    &m_bloom_descriptor_set,
                                    0,
                                    nullptr);
            vkCmdPushConstants(m_active_command_buffer,
                               m_bloom_pipeline_layout,
                               VK_SHADER_STAGE_FRAGMENT_BIT,
                               0,
                               sizeof(BloomPushConstants),
                               &push);
            vkCmdDraw(m_active_command_buffer, 3, 1, 0, 0);
        }

        [[nodiscard]] bool evaluate_dlss()
        {
#ifdef ROSE_ENABLE_NGX_DLSS
            if (!dlss_active())
                return false;

            record_image_barrier(m_scene_color_image,
                                 VK_IMAGE_ASPECT_COLOR_BIT,
                                 VK_IMAGE_LAYOUT_GENERAL,
                                 VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
                                 VK_ACCESS_SHADER_READ_BIT,
                                 VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                                 VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
            record_image_barrier(m_motion_vector_image,
                                 VK_IMAGE_ASPECT_COLOR_BIT,
                                 VK_IMAGE_LAYOUT_GENERAL,
                                 VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
                                 VK_ACCESS_SHADER_READ_BIT,
                                 VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
                                 VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
            record_image_barrier(m_depth_image,
                                 VK_IMAGE_ASPECT_DEPTH_BIT,
                                 VK_IMAGE_LAYOUT_GENERAL,
                                 VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
                                 VK_ACCESS_SHADER_READ_BIT,
                                 VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT,
                                 VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);
            record_image_barrier(m_dlss_output_image,
                                 VK_IMAGE_ASPECT_COLOR_BIT,
                                 VK_IMAGE_LAYOUT_GENERAL,
                                 VK_ACCESS_TRANSFER_READ_BIT | VK_ACCESS_SHADER_READ_BIT,
                                 VK_ACCESS_SHADER_WRITE_BIT,
                                 VK_PIPELINE_STAGE_TRANSFER_BIT | VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
                                 VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);

            const VkImageSubresourceRange color_range{
                VK_IMAGE_ASPECT_COLOR_BIT,
                0,
                1,
                0,
                1
            };
            const VkImageSubresourceRange depth_range{
                VK_IMAGE_ASPECT_DEPTH_BIT,
                0,
                1,
                0,
                1
            };

            NVSDK_NGX_Resource_VK color_resource = NVSDK_NGX_Create_ImageView_Resource_VK(m_scene_color_image.view,
                                                                                          m_scene_color_image.image,
                                                                                          color_range,
                                                                                          m_scene_color_format,
                                                                                          m_scene_extent.width,
                                                                                          m_scene_extent.height,
                                                                                          false);
            NVSDK_NGX_Resource_VK motion_resource = NVSDK_NGX_Create_ImageView_Resource_VK(m_motion_vector_image.view,
                                                                                           m_motion_vector_image.image,
                                                                                           color_range,
                                                                                           m_motion_vector_format,
                                                                                           m_scene_extent.width,
                                                                                           m_scene_extent.height,
                                                                                           false);
            NVSDK_NGX_Resource_VK depth_resource = NVSDK_NGX_Create_ImageView_Resource_VK(m_depth_image.view,
                                                                                          m_depth_image.image,
                                                                                          depth_range,
                                                                                          m_depth_format,
                                                                                          m_scene_extent.width,
                                                                                          m_scene_extent.height,
                                                                                          false);
            NVSDK_NGX_Resource_VK output_resource = NVSDK_NGX_Create_ImageView_Resource_VK(m_dlss_output_image.view,
                                                                                           m_dlss_output_image.image,
                                                                                           color_range,
                                                                                           m_scene_color_format,
                                                                                           m_swapchain_extent.width,
                                                                                           m_swapchain_extent.height,
                                                                                           true);

            NVSDK_NGX_VK_DLSS_Eval_Params eval_params{};
            eval_params.Feature.pInColor = &color_resource;
            eval_params.Feature.pInOutput = &output_resource;
            eval_params.Feature.InSharpness = m_dlss_sharpness;
            eval_params.pInDepth = &depth_resource;
            eval_params.pInMotionVectors = &motion_resource;
            eval_params.InRenderSubrectDimensions = {m_scene_extent.width, m_scene_extent.height};
            eval_params.InReset = m_dlss_reset_next_frame ? 1 : 0;
            eval_params.InMVScaleX = static_cast<float>(m_scene_extent.width);
            eval_params.InMVScaleY = static_cast<float>(m_scene_extent.height);
            eval_params.InPreExposure = 1.0f;
            eval_params.InExposureScale = 1.0f;

            const NVSDK_NGX_Result result = NGX_VULKAN_EVALUATE_DLSS_EXT(m_active_command_buffer,
                                                                         m_ngx_dlss_handle,
                                                                         m_ngx_parameters,
                                                                         &eval_params);
            if (NVSDK_NGX_FAILED(result))
            {
                m_dlss_status = std::string("DLSS evaluate failed: ") + ngx_result_name(result);
                m_dlss_reset_next_frame = true;
                m_dlss_requested = false;
                return false;
            }

            m_dlss_reset_next_frame = false;
            return true;
#else
            return false;
#endif
        }

        void begin_present_render_pass()
        {
            VkRenderPassBeginInfo render_pass_info{};
            render_pass_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
            render_pass_info.renderPass = m_present_render_pass;
            render_pass_info.framebuffer = m_framebuffers[m_active_image_index];
            render_pass_info.renderArea.offset = {0, 0};
            render_pass_info.renderArea.extent = m_swapchain_extent;
            render_pass_info.clearValueCount = 0;
            render_pass_info.pClearValues = nullptr;

            vkCmdBeginRenderPass(m_active_command_buffer, &render_pass_info, VK_SUBPASS_CONTENTS_INLINE);
            m_present_render_pass_active = true;
        }

        void finish_scene_rendering()
        {
            if (!m_scene_render_pass_active)
                return;

            vkCmdEndRenderPass(m_active_command_buffer);
            m_scene_render_pass_active = false;
            m_scene_color_image.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
            m_motion_vector_image.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
            m_depth_image.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

            ImageResource* resolved_source = &m_scene_color_image;
            if (evaluate_dlss())
                resolved_source = &m_dlss_output_image;
            if (m_bloom_settings.enabled)
                render_bloom_to_swapchain(*resolved_source);
            else
            {
                blit_to_swapchain(*resolved_source);
                begin_present_render_pass();
            }
        }

        void copy_buffer_to_image(VkBuffer buffer, VkImage image, uint32_t width, uint32_t height) const
        {
            VkCommandBuffer command_buffer = begin_single_time_commands();

            VkBufferImageCopy region{};
            region.bufferOffset = 0;
            region.bufferRowLength = 0;
            region.bufferImageHeight = 0;
            region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            region.imageSubresource.mipLevel = 0;
            region.imageSubresource.baseArrayLayer = 0;
            region.imageSubresource.layerCount = 1;
            region.imageOffset = {0, 0, 0};
            region.imageExtent = {width, height, 1};

            vkCmdCopyBufferToImage(command_buffer, buffer, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
            end_single_time_commands(command_buffer);
        }

        [[nodiscard]] std::vector<unsigned char> texture_pixels_rgba(const Texture& texture) const
        {
            std::vector<unsigned char> rgba(static_cast<std::size_t>(texture.width())
                                           * static_cast<std::size_t>(texture.height())
                                           * 4u);
            const std::vector<unsigned char>& src = texture.pixels();
            const int components = texture.components();

            for (int y = 0; y < texture.height(); ++y)
            {
                for (int x = 0; x < texture.width(); ++x)
                {
                    const auto src_index = static_cast<std::size_t>(y * texture.width() + x)
                                         * static_cast<std::size_t>(components);
                    const auto dst_index = static_cast<std::size_t>(y * texture.width() + x) * 4u;

                    const unsigned char r = components >= 1 ? src[src_index] : 255u;
                    const unsigned char g = components >= 2 ? src[src_index + 1u] : r;
                    const unsigned char b = components >= 3 ? src[src_index + 2u] : r;
                    const unsigned char a = components >= 4 ? src[src_index + 3u] : 255u;
                    rgba[dst_index] = r;
                    rgba[dst_index + 1u] = g;
                    rgba[dst_index + 2u] = b;
                    rgba[dst_index + 3u] = a;
                }
            }

            return rgba;
        }

        GpuTexture create_texture_resource(const Texture& texture)
        {
            const std::vector<unsigned char> pixels = texture_pixels_rgba(texture);
            const uint32_t width = static_cast<uint32_t>(texture.width());
            const uint32_t height = static_cast<uint32_t>(texture.height());
            const VkDeviceSize image_size = static_cast<VkDeviceSize>(pixels.size());

            BufferResource staging_buffer;
            create_buffer(image_size,
                          VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                          staging_buffer);

            void* mapped = nullptr;
            check_vk(vkMapMemory(m_device, staging_buffer.memory, 0, image_size, 0, &mapped),
                     "Failed to map texture staging buffer");
            std::memcpy(mapped, pixels.data(), pixels.size());
            vkUnmapMemory(m_device, staging_buffer.memory);

            GpuTexture gpu_texture;
            create_image(width,
                         height,
                         VK_FORMAT_R8G8B8A8_SRGB,
                         VK_IMAGE_TILING_OPTIMAL,
                         VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                         VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                         gpu_texture.image);

            transition_image_layout(gpu_texture.image.image,
                                    VK_FORMAT_R8G8B8A8_SRGB,
                                    VK_IMAGE_LAYOUT_UNDEFINED,
                                    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
            copy_buffer_to_image(staging_buffer.buffer, gpu_texture.image.image, width, height);
            transition_image_layout(gpu_texture.image.image,
                                    VK_FORMAT_R8G8B8A8_SRGB,
                                    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            destroy_buffer(staging_buffer);

            gpu_texture.image.view = create_image_view(gpu_texture.image.image,
                                                       VK_FORMAT_R8G8B8A8_SRGB,
                                                       VK_IMAGE_ASPECT_COLOR_BIT);

            VkSamplerCreateInfo sampler_info{};
            sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
            sampler_info.magFilter = VK_FILTER_LINEAR;
            sampler_info.minFilter = VK_FILTER_LINEAR;
            sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
            sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
            sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
            sampler_info.anisotropyEnable = VK_FALSE;
            sampler_info.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
            sampler_info.unnormalizedCoordinates = VK_FALSE;
            sampler_info.compareEnable = VK_FALSE;
            sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;

            check_vk(vkCreateSampler(m_device, &sampler_info, nullptr, &gpu_texture.sampler), "Failed to create texture sampler");
            gpu_texture.descriptor = allocate_texture_descriptor(gpu_texture);
            return gpu_texture;
        }

        [[nodiscard]] VkDescriptorSet allocate_texture_descriptor(const GpuTexture& texture) const
        {
            VkDescriptorSetAllocateInfo alloc_info{};
            alloc_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
            alloc_info.descriptorPool = m_descriptor_pool;
            alloc_info.descriptorSetCount = 1;
            alloc_info.pSetLayouts = &m_descriptor_set_layout;

            VkDescriptorSet descriptor_set = VK_NULL_HANDLE;
            check_vk(vkAllocateDescriptorSets(m_device, &alloc_info, &descriptor_set), "Failed to allocate texture descriptor set");

            VkDescriptorImageInfo image_info{};
            image_info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
            image_info.imageView = texture.image.view;
            image_info.sampler = texture.sampler;

            VkWriteDescriptorSet descriptor_write{};
            descriptor_write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            descriptor_write.dstSet = descriptor_set;
            descriptor_write.dstBinding = 0;
            descriptor_write.dstArrayElement = 0;
            descriptor_write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            descriptor_write.descriptorCount = 1;
            descriptor_write.pImageInfo = &image_info;

            vkUpdateDescriptorSets(m_device, 1, &descriptor_write, 0, nullptr);
            return descriptor_set;
        }

        [[nodiscard]] VkDescriptorSet default_texture_descriptor()
        {
            if (!m_default_texture_created)
            {
                const unsigned char white[] = {255u, 255u, 255u, 255u};
                const Texture texture{1, 1, 4, white};
                m_default_texture = create_texture_resource(texture);
                m_default_texture_created = true;
            }
            return m_default_texture.descriptor;
        }

        [[nodiscard]] VkDescriptorSet descriptor_for_mesh_texture(const Mesh& mesh)
        {
            for (const MeshTexture& mesh_texture : mesh.textures())
            {
                if (mesh_texture.type != TextureType::BaseColor || !mesh_texture.texture || !mesh_texture.texture->valid())
                    continue;

                const Texture* texture = mesh_texture.texture.get();
                const auto it = m_texture_resources.find(texture);
                if (it != m_texture_resources.end())
                    return it->second.descriptor;

                auto [inserted_it, _] = m_texture_resources.emplace(texture, create_texture_resource(*texture));
                return inserted_it->second.descriptor;
            }
            return default_texture_descriptor();
        }

        [[nodiscard]] GpuMesh& ensure_mesh_resource(const Mesh& mesh)
        {
            const auto existing = m_mesh_resources.find(&mesh);
            if (existing != m_mesh_resources.end())
                return existing->second;

            GpuMesh gpu_mesh;
            const auto& vertices = mesh.cpu_mesh().m_vertex_buffer;
            const auto& triangles = mesh.cpu_mesh().m_element_buffer_object;

            static_assert(sizeof(omath::Vector3<uint32_t>) == 3 * sizeof(uint32_t),
                          "omath::Vector3<uint32_t> must be tightly packed");

            const VkDeviceSize vertex_buffer_size = static_cast<VkDeviceSize>(vertices.size())
                                                  * static_cast<VkDeviceSize>(sizeof(vertices.front()));
            const VkDeviceSize index_buffer_size = static_cast<VkDeviceSize>(triangles.size())
                                                 * static_cast<VkDeviceSize>(sizeof(triangles.front()));

            create_device_buffer(vertices.data(), vertex_buffer_size, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, gpu_mesh.vertex_buffer);
            create_device_buffer(triangles.data(), index_buffer_size, VK_BUFFER_USAGE_INDEX_BUFFER_BIT, gpu_mesh.index_buffer);
            gpu_mesh.index_count = static_cast<uint32_t>(triangles.size() * 3u);
            gpu_mesh.descriptor = descriptor_for_mesh_texture(mesh);

            omath::Vector3<float> local_min = vertices.front().position;
            omath::Vector3<float> local_max = vertices.front().position;
            for (const auto& vertex : vertices)
            {
                local_min.x = std::min(local_min.x, vertex.position.x);
                local_min.y = std::min(local_min.y, vertex.position.y);
                local_min.z = std::min(local_min.z, vertex.position.z);
                local_max.x = std::max(local_max.x, vertex.position.x);
                local_max.y = std::max(local_max.y, vertex.position.y);
                local_max.z = std::max(local_max.z, vertex.position.z);
            }
            gpu_mesh.local_center = (local_min + local_max) / 2.0f;

            auto [inserted_it, _] = m_mesh_resources.emplace(&mesh, gpu_mesh);
            return inserted_it->second;
        }

        void destroy_texture(GpuTexture& texture) const noexcept
        {
            if (texture.sampler != VK_NULL_HANDLE)
                vkDestroySampler(m_device, texture.sampler, nullptr);
            ImageResource image = texture.image;
            destroy_image(image);
            texture = {};
        }

        void destroy_gpu_resources() noexcept
        {
            for (auto& [_, mesh] : m_mesh_resources)
            {
                destroy_buffer(mesh.vertex_buffer);
                destroy_buffer(mesh.index_buffer);
            }
            m_mesh_resources.clear();

            for (auto& [_, texture] : m_texture_resources)
                destroy_texture(texture);
            m_texture_resources.clear();

            if (m_default_texture_created)
            {
                destroy_texture(m_default_texture);
                m_default_texture_created = false;
            }
        }

        void destroy_readback_slots() noexcept
        {
            for (ReadbackSlot& slot : m_readback_slots)
            {
                destroy_buffer(slot.buffer);
                slot = {};
            }
            m_completed_stream_frame.reset();
        }

        void ensure_readback_slot(ReadbackSlot& slot, VkDeviceSize size)
        {
            if (slot.buffer.buffer != VK_NULL_HANDLE && slot.buffer.size == size)
                return;

            destroy_buffer(slot.buffer);
            create_buffer(size,
                          VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                          slot.buffer);
        }

        void collect_completed_readback(std::size_t frame_index)
        {
            ReadbackSlot& slot = m_readback_slots[frame_index];
            if (!slot.pending)
                return;

            m_completed_stream_frame = readback_frame(slot.buffer, slot.extent, slot.format);
            slot.pending = false;
        }

        void record_screenshot_copy(const BufferResource& readback_buffer) const
        {
            VkImageMemoryBarrier to_transfer{};
            to_transfer.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
            to_transfer.oldLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
            to_transfer.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
            to_transfer.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            to_transfer.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            to_transfer.image = m_swapchain_images[m_active_image_index];
            to_transfer.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            to_transfer.subresourceRange.baseMipLevel = 0;
            to_transfer.subresourceRange.levelCount = 1;
            to_transfer.subresourceRange.baseArrayLayer = 0;
            to_transfer.subresourceRange.layerCount = 1;
            to_transfer.srcAccessMask = 0;
            to_transfer.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;

            vkCmdPipelineBarrier(m_active_command_buffer,
                                 VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
                                 VK_PIPELINE_STAGE_TRANSFER_BIT,
                                 0,
                                 0,
                                 nullptr,
                                 0,
                                 nullptr,
                                 1,
                                 &to_transfer);

            VkBufferImageCopy copy_region{};
            copy_region.bufferOffset = 0;
            copy_region.bufferRowLength = 0;
            copy_region.bufferImageHeight = 0;
            copy_region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            copy_region.imageSubresource.mipLevel = 0;
            copy_region.imageSubresource.baseArrayLayer = 0;
            copy_region.imageSubresource.layerCount = 1;
            copy_region.imageOffset = {0, 0, 0};
            copy_region.imageExtent = {m_swapchain_extent.width, m_swapchain_extent.height, 1};

            vkCmdCopyImageToBuffer(m_active_command_buffer,
                                   m_swapchain_images[m_active_image_index],
                                   VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                                   readback_buffer.buffer,
                                   1,
                                   &copy_region);

            VkImageMemoryBarrier to_present = to_transfer;
            to_present.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
            to_present.newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
            to_present.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
            to_present.dstAccessMask = 0;

            vkCmdPipelineBarrier(m_active_command_buffer,
                                 VK_PIPELINE_STAGE_TRANSFER_BIT,
                                 VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
                                 0,
                                 0,
                                 nullptr,
                                 0,
                                 nullptr,
                                 1,
                                 &to_present);
        }

        [[nodiscard]] CapturedFrame readback_frame(const BufferResource& readback_buffer,
                                                   VkExtent2D extent,
                                                   VkFormat format) const
        {
            void* mapped = nullptr;
            check_vk(vkMapMemory(m_device, readback_buffer.memory, 0, readback_buffer.size, 0, &mapped),
                     "Failed to map screenshot buffer");

            CapturedFrame frame;
            frame.width = extent.width;
            frame.height = extent.height;
            frame.format = is_bgra_format(format) ? CapturedFrameFormat::Bgra : CapturedFrameFormat::Rgba;
            frame.pixels.resize(static_cast<std::size_t>(extent.width) * static_cast<std::size_t>(extent.height) * 4u);
            std::memcpy(frame.pixels.data(), mapped, frame.pixels.size());

            vkUnmapMemory(m_device, readback_buffer.memory);
            return frame;
        }

        void cleanup_swapchain() noexcept
        {
            if (!m_command_buffers.empty())
            {
                vkFreeCommandBuffers(m_device,
                                     m_command_pool,
                                     static_cast<uint32_t>(m_command_buffers.size()),
                                     m_command_buffers.data());
                m_command_buffers.clear();
            }

            destroy_frame_targets();
            destroy_readback_slots();

            for (VkImageView image_view : m_swapchain_image_views)
                vkDestroyImageView(m_device, image_view, nullptr);
            m_swapchain_image_views.clear();

            if (m_swapchain != VK_NULL_HANDLE)
            {
                vkDestroySwapchainKHR(m_device, m_swapchain, nullptr);
                m_swapchain = VK_NULL_HANDLE;
            }
        }

        void recreate_swapchain()
        {
            int width = 0;
            int height = 0;
            glfwGetFramebufferSize(m_window, &width, &height);
            while (width == 0 || height == 0)
            {
                glfwWaitEvents();
                glfwGetFramebufferSize(m_window, &width, &height);
            }

            spdlog::info("Vulkan: recreating swapchain");
            check_vk(vkDeviceWaitIdle(m_device), "Failed to wait for device before swapchain recreation");
            const VkFormat old_format = m_swapchain_image_format;
            cleanup_swapchain();
            create_swapchain();
            if (m_render_pass != VK_NULL_HANDLE && old_format != m_swapchain_image_format)
                throw VulkanError("Swapchain image format changed during resize; restart the application");
            create_image_views();
            create_render_targets();
            create_framebuffers();
            create_command_buffers();
            ImGui_ImplVulkan_SetMinImageCount(m_min_image_count);
        }
    };

    Renderer::Renderer(GLFWwindow* window, const omath::Vector2<int>& initial_size)
        : m_impl(std::make_unique<Impl>(window, initial_size))
    {}

    Renderer::~Renderer() = default;

    bool Renderer::begin_frame()
    {
        return m_impl->begin_frame();
    }

    void Renderer::draw_mesh(const Mesh& mesh, const omath::opengl_engine::Camera& camera)
    {
        m_impl->draw_mesh(mesh, camera);
    }

    void Renderer::draw_mesh_outline(const Mesh& mesh, const omath::opengl_engine::Camera& camera)
    {
        m_impl->draw_mesh_outline(mesh, camera);
    }

    void Renderer::render_imgui(ImDrawData* draw_data)
    {
        m_impl->render_imgui(draw_data);
    }

    std::optional<CapturedFrame> Renderer::end_frame(bool capture_screenshot)
    {
        return m_impl->end_frame(capture_screenshot);
    }

    void Renderer::wait_idle() const
    {
        m_impl->wait_idle();
    }

    omath::Vector2<int> Renderer::framebuffer_size() const
    {
        return m_impl->framebuffer_size();
    }

    bool Renderer::dlss_available() const
    {
        return m_impl->dlss_available();
    }

    bool Renderer::dlss_enabled() const
    {
        return m_impl->m_dlss_requested;
    }

    void Renderer::set_dlss_enabled(bool enabled)
    {
        m_impl->set_dlss_enabled(enabled);
    }

    DlssQuality Renderer::dlss_quality() const
    {
        return m_impl->m_dlss_quality;
    }

    void Renderer::set_dlss_quality(DlssQuality quality)
    {
        m_impl->set_dlss_quality(quality);
    }

    std::string Renderer::dlss_status() const
    {
        return m_impl->m_dlss_status;
    }

    SelectionOutlineSettings Renderer::selection_outline_settings() const
    {
        return m_impl->m_selection_outline_settings;
    }

    void Renderer::set_selection_outline_settings(const SelectionOutlineSettings& settings)
    {
        m_impl->set_selection_outline_settings(settings);
    }

    BloomSettings Renderer::bloom_settings() const
    {
        return m_impl->m_bloom_settings;
    }

    void Renderer::set_bloom_settings(const BloomSettings& settings)
    {
        m_impl->set_bloom_settings(settings);
    }
} // namespace rose::core::vulkan
