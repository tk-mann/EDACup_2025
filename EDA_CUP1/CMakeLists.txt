cmake_minimum_required(VERSION 3.25)

project(move_sample)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")


set(CMAKE_PREFIX_PATH "C:/dev/vcpkg/packages/nlohmann-json_x64-windows")
find_package(nlohmann_json CONFIG REQUIRED)


file(GLOB SAMPLE_SOURCE "*.cpp")

add_executable(move_sample ${SAMPLE_SOURCE})

target_include_directories(move_sample PRIVATE
    "C:/dev/vcpkg/installed/x64-windows/include"
)

target_link_libraries(move_sample PRIVATE nlohmann_json::nlohmann_json)
