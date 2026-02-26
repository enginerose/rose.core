cmake_minimum_required(VERSION 3.31)
project(rose.core)


file(GLOB_RECURSE ROSE_HEADERS include/*.hpp)
file(GLOB_RECURSE ROSE_SOURCES source/*.cpp)

add_executable(${PROJECT_NAME} ${ROSE_HEADERS} ${ROSE_SOURCES})


set_target_properties(${PROJECT_NAME} PROPERTIES
        CXX_STANDARD 26
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out/${CMAKE_BUILD_TYPE}"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out/${CMAKE_BUILD_TYPE}"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out/${CMAKE_BUILD_TYPE}"
)

target_include_directories(${PROJECT_NAME} PRIVATE include)

if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(${PROJECT_NAME} PRIVATE /W4 /WX)
elseif (OMATH_THREAT_WARNING_AS_ERROR)
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif ()

if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_definitions(${PROJECT_NAME} INTERFACE NOMINMAX)
endif ()

find_package(GLEW REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(omath CONFIG REQUIRED)
find_package(OpenGL REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(Boost REQUIRED COMPONENTS dll)

find_path(TINYGLTF_INCLUDE_DIRS "tiny_gltf.h")


target_link_libraries(${PROJECT_NAME} PRIVATE omath::omath GLEW::GLEW glfw OpenGL::GL spdlog::spdlog Boost::dll)
target_link_libraries(${PROJECT_NAME} PRIVATE)
target_include_directories(${PROJECT_NAME} PRIVATE ${TINYGLTF_INCLUDE_DIRS})