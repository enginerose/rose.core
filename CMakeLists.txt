cmake_minimum_required(VERSION 3.31)
project(rose.core)


file(GLOB_RECURSE ROSE_HEADERS include/*.hpp)
file(GLOB_RECURSE ROSE_SOURCES source/*.cpp)

add_executable(${PROJECT_NAME} ${ROSE_HEADERS} ${ROSE_SOURCES})

option(ROSE_DYNAMIC_MSVC_RUNTIME "Use the dynamic MSVC runtime library (/MD, /MDd)" ON)
if (ROSE_DYNAMIC_MSVC_RUNTIME)
    set(ROSE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
    set(OMATH_STATIC_MSVC_RUNTIME_LIBRARY OFF CACHE BOOL "" FORCE)
else ()
    set(ROSE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    set(OMATH_STATIC_MSVC_RUNTIME_LIBRARY ON CACHE BOOL "" FORCE)
endif ()

set_target_properties(${PROJECT_NAME} PROPERTIES
        CXX_STANDARD 23
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out/${CMAKE_BUILD_TYPE}"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out/${CMAKE_BUILD_TYPE}"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out/${CMAKE_BUILD_TYPE}"
        MSVC_RUNTIME_LIBRARY "${ROSE_MSVC_RUNTIME_LIBRARY}"
)

if (MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "${ROSE_MSVC_RUNTIME_LIBRARY}" CACHE STRING "" FORCE)
endif ()

target_include_directories(${PROJECT_NAME} PRIVATE include)

if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(${PROJECT_NAME} PRIVATE /W4 /WX)
    set_source_files_properties("${CMAKE_CURRENT_SOURCE_DIR}/source/core/model.cpp" PROPERTIES COMPILE_OPTIONS /wd4702)
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

add_library(imguizmo STATIC extlibs/ImGuizmo/ImGuizmo.cpp)
target_include_directories(imguizmo PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/extlibs/ImGuizmo")
target_link_libraries(imguizmo PUBLIC imgui::imgui)
set_target_properties(imguizmo PROPERTIES
        CXX_STANDARD 23
        MSVC_RUNTIME_LIBRARY "${ROSE_MSVC_RUNTIME_LIBRARY}"
)
if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(imguizmo PRIVATE /W0)
endif ()

set(ROSE_NGX_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/extlibs/Streamline/external/ngx-sdk")
set(ROSE_NGX_LIB_DIR "${ROSE_NGX_SDK_DIR}/lib/Windows_x86_64")
set(ROSE_NGX_LIB_DEBUG "${ROSE_NGX_LIB_DIR}/nvsdk_ngx_d_dbg.lib")
set(ROSE_NGX_LIB_RELEASE "${ROSE_NGX_LIB_DIR}/nvsdk_ngx_d.lib")
set(ROSE_NGX_DLSS_DLL "" CACHE FILEPATH "Path to nvngx_dlss.dll. Leave empty to use Streamline bin/x64 or a local DLSS Swapper install.")
if (WIN32 AND ROSE_DYNAMIC_MSVC_RUNTIME AND EXISTS "${ROSE_NGX_LIB_DEBUG}" AND EXISTS "${ROSE_NGX_LIB_RELEASE}")
    option(ROSE_ENABLE_NGX_DLSS "Enable NVIDIA DLSS through the NGX SDK bundled with Streamline" ON)
else ()
    option(ROSE_ENABLE_NGX_DLSS "Enable NVIDIA DLSS through the NGX SDK bundled with Streamline" OFF)
endif ()
if (ROSE_ENABLE_NGX_DLSS AND NOT ROSE_DYNAMIC_MSVC_RUNTIME)
    message(FATAL_ERROR "ROSE_ENABLE_NGX_DLSS requires ROSE_DYNAMIC_MSVC_RUNTIME=ON because NVIDIA's NGX libraries use /MD.")
endif ()

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
target_compile_definitions(${PROJECT_NAME} PRIVATE
        ROSE_SHADER_DIR="${ROSE_SHADER_OUTPUT_DIR}"
        ROSE_SHADER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/shaders"
)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${PROJECT_NAME}>/shaders"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${ROSE_SHADER_OUTPUTS}
        ${ROSE_SHADER_SOURCES}
        "$<TARGET_FILE_DIR:${PROJECT_NAME}>/shaders"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${GLSLC_EXECUTABLE}"
        "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
        VERBATIM
)

add_subdirectory(extlibs/omath)
target_link_libraries(${PROJECT_NAME} PRIVATE asio::asio imgui::imgui imguizmo omath::omath glfw Vulkan::Vulkan spdlog::spdlog Boost::dll winmm)
target_link_libraries(${PROJECT_NAME} PRIVATE)
target_include_directories(${PROJECT_NAME} PRIVATE ${TINYGLTF_INCLUDE_DIRS})

if (ROSE_ENABLE_NGX_DLSS)
    target_compile_definitions(${PROJECT_NAME} PRIVATE ROSE_ENABLE_NGX_DLSS=1)
    target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE "${ROSE_NGX_SDK_DIR}/include")
    target_link_libraries(${PROJECT_NAME} PRIVATE
            debug "${ROSE_NGX_LIB_DEBUG}"
            optimized "${ROSE_NGX_LIB_RELEASE}"
    )

    set(ROSE_NGX_DLSS_DLL_TO_COPY "${ROSE_NGX_DLSS_DLL}")
    if (NOT ROSE_NGX_DLSS_DLL_TO_COPY AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/extlibs/Streamline/bin/x64/nvngx_dlss.dll")
        set(ROSE_NGX_DLSS_DLL_TO_COPY "${CMAKE_CURRENT_SOURCE_DIR}/extlibs/Streamline/bin/x64/nvngx_dlss.dll")
    endif ()
    if (NOT ROSE_NGX_DLSS_DLL_TO_COPY AND DEFINED ENV{LOCALAPPDATA})
        file(GLOB_RECURSE ROSE_LOCAL_DLSS_DLLS "$ENV{LOCALAPPDATA}/DLSS Swapper/dlls/dlss/*/nvngx_dlss.dll")
        if (ROSE_LOCAL_DLSS_DLLS)
            list(GET ROSE_LOCAL_DLSS_DLLS 0 ROSE_NGX_DLSS_DLL_TO_COPY)
        endif ()
    endif ()

    if (ROSE_NGX_DLSS_DLL_TO_COPY)
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${ROSE_NGX_DLSS_DLL_TO_COPY}"
                "$<TARGET_FILE_DIR:${PROJECT_NAME}>/nvngx_dlss.dll"
                VERBATIM
        )
        message(STATUS "DLSS runtime DLL: ${ROSE_NGX_DLSS_DLL_TO_COPY}")
    else ()
        message(WARNING "nvngx_dlss.dll was not found. DLSS can initialize NGX core, but DLSS feature creation will fail until the DLL is copied next to rose.core.exe or ROSE_NGX_DLSS_DLL is set.")
    endif ()
endif ()
