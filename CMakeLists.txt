cmake_minimum_required(VERSION 3.31)
project(rose.core)



add_executable(${PROJECT_NAME} source/main.cpp)


set_target_properties(${PROJECT_NAME} PROPERTIES
        CXX_STANDARD 26
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out/${CMAKE_BUILD_TYPE}"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out/${CMAKE_BUILD_TYPE}"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out/${CMAKE_BUILD_TYPE}"
)
find_package(GLEW REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(omath CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE omath::omath)



target_link_libraries(${PROJECT_NAME} PRIVATE omath::omath GLEW::GLEW glfw)

find_path(TINYGLTF_INCLUDE_DIRS "tiny_gltf.h")
target_include_directories(${PROJECT_NAME} PRIVATE ${TINYGLTF_INCLUDE_DIRS})