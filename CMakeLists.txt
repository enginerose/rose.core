cmake_minimum_required(VERSION 3.31)
project(rose.core)


file(GLOB_RECURSE ROSE_HEADERS include/*.hpp)
file(GLOB_RECURSE ROSE_SOURCES source/*.cpp)

add_executable(${PROJECT_NAME} ${ROSE_HEADERS} ${ROSE_SOURCES})

set(OMATH_STATIC_MSVC_RUNTIME_LIBRARY ON)

set_target_properties(${PROJECT_NAME} PROPERTIES
        CXX_STANDARD 23
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out/${CMAKE_BUILD_TYPE}"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out/${CMAKE_BUILD_TYPE}"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out/${CMAKE_BUILD_TYPE}"
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
)

target_include_directories(${PROJECT_NAME} PRIVATE include)

if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(${PROJECT_NAME} PRIVATE /W4 /WX)
elseif (OMATH_THREAT_WARNING_AS_ERROR)
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif ()

if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_definitions(${PROJECT_NAME} PRIVATE NOMINMAX)
endif ()

find_package(glfw3 CONFIG REQUIRED)
find_package(Vulkan REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(Boost REQUIRED COMPONENTS dll)
find_package(imgui CONFIG REQUIRED)
find_path(TINYGLTF_INCLUDE_DIRS "tiny_gltf.h")
find_package(asio CONFIG REQUIRED)

find_program(GLSLC_EXECUTABLE glslc HINTS "$ENV{VULKAN_SDK}/Bin")
if (NOT GLSLC_EXECUTABLE)
    message(FATAL_ERROR "glslc was not found. Install the Vulkan SDK or put glslc on PATH.")
endif ()

set(ROSE_SHADER_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/shaders")
file(MAKE_DIRECTORY "${ROSE_SHADER_OUTPUT_DIR}")

set(ROSE_SHADER_SOURCES
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/shader.vert"
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/shader.frag"
)

set(ROSE_SHADER_OUTPUTS)
foreach (ROSE_SHADER_SOURCE IN LISTS ROSE_SHADER_SOURCES)
    get_filename_component(ROSE_SHADER_NAME "${ROSE_SHADER_SOURCE}" NAME)
    set(ROSE_SHADER_OUTPUT "${ROSE_SHADER_OUTPUT_DIR}/${ROSE_SHADER_NAME}.spv")
    add_custom_command(
            OUTPUT "${ROSE_SHADER_OUTPUT}"
            COMMAND "${GLSLC_EXECUTABLE}" --target-env=vulkan1.2 "${ROSE_SHADER_SOURCE}" -o "${ROSE_SHADER_OUTPUT}"
            DEPENDS "${ROSE_SHADER_SOURCE}"
            VERBATIM
    )
    list(APPEND ROSE_SHADER_OUTPUTS "${ROSE_SHADER_OUTPUT}")
endforeach ()

add_custom_target(rose_shaders DEPENDS ${ROSE_SHADER_OUTPUTS})
add_dependencies(${PROJECT_NAME} rose_shaders)
target_compile_definitions(${PROJECT_NAME} PRIVATE ROSE_SHADER_DIR="${ROSE_SHADER_OUTPUT_DIR}")

add_subdirectory(extlibs/omath)
target_link_libraries(${PROJECT_NAME} PRIVATE asio::asio imgui::imgui omath::omath glfw Vulkan::Vulkan spdlog::spdlog Boost::dll winmm)
target_link_libraries(${PROJECT_NAME} PRIVATE)
target_include_directories(${PROJECT_NAME} PRIVATE ${TINYGLTF_INCLUDE_DIRS})
