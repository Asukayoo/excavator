cmake_minimum_required(VERSION 3.20)
project(robot_control LANGUAGES CXX)

# 根据目标平台归一化 amd64 / arm64（交叉编译时以 CMAKE_SYSTEM_PROCESSOR 为准）。
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _robot_proc)
if(_robot_proc MATCHES "^(x86_64|amd64)$")
    set(ROBOT_CONTROL_ARCH "amd64")
elseif(_robot_proc MATCHES "^(aarch64|arm64)$")
    set(ROBOT_CONTROL_ARCH "arm64")
else()
    set(ROBOT_CONTROL_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
    message(WARNING
        "CMAKE_SYSTEM_PROCESSOR=${CMAKE_SYSTEM_PROCESSOR} 非典型 amd64/arm64；若为交叉编译请检查工具链。")
endif()
message(STATUS "robot_control 目标架构: ${ROBOT_CONTROL_ARCH}")

if(APPLE)
    string(LENGTH "${CMAKE_OSX_ARCHITECTURES}" _robot_osx_arch_len)
    if(_robot_osx_arch_len EQUAL 0)
        if(ROBOT_CONTROL_ARCH STREQUAL "arm64")
            set(CMAKE_OSX_ARCHITECTURES arm64 CACHE STRING "macOS 单架构" FORCE)
        elseif(ROBOT_CONTROL_ARCH STREQUAL "amd64")
            set(CMAKE_OSX_ARCHITECTURES x86_64 CACHE STRING "macOS 单架构" FORCE)
        endif()
    endif()
endif()

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 便于 clangd 等工具获取 compile_commands（MSVC 需 CMake 3.20+）
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Eigen3: 先尝试 CONFIG（vcpkg/自装），失败时回退到 Ubuntu 常见模块模式。
find_package(Eigen3 CONFIG QUIET)
if(NOT Eigen3_FOUND)
    find_package(Eigen3 REQUIRED)
endif()

find_package(Threads REQUIRED)

add_library(robot_control INTERFACE)
add_library(robot_control::robot_control ALIAS robot_control)

target_include_directories(robot_control INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(robot_control INTERFACE Eigen3::Eigen)

add_library(robot_control_excavator STATIC
    src/excavator/excavator_client.cpp
    src/excavator/excavator_control.cpp
    src/excavator/excavator_comm.cpp
    src/excavator/excavator_converter.cpp)
target_link_libraries(robot_control_excavator PUBLIC robot_control)
target_link_libraries(robot_control_excavator PRIVATE Threads::Threads)
target_include_directories(robot_control_excavator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_library(robot_control_canlib STATIC
    src/can/excavator_canlib.cpp
    src/can/imu_canlib.cpp)
target_link_libraries(robot_control_canlib PUBLIC robot_control)
target_link_libraries(robot_control_canlib PRIVATE Threads::Threads)
target_include_directories(robot_control_canlib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_library(robot_control_api STATIC
    src/excavator_api/excavator_runtime.cpp
    src/excavator_api/excavator_control.cpp
    src/excavator_api/excavator_receive.cpp)
target_link_libraries(robot_control_api PUBLIC robot_control robot_control_excavator robot_control_canlib)
target_link_libraries(robot_control_api PRIVATE Threads::Threads)
target_include_directories(robot_control_api PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
if(WIN32)
    target_link_libraries(robot_control_excavator PRIVATE winmm)
endif()

add_executable(excavator_api_tcp_client tools/excavator_api_tcp_client.cpp)
target_link_libraries(excavator_api_tcp_client PRIVATE robot_control_api)

add_custom_target(
    sync_compile_commands
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_BINARY_DIR}/compile_commands.json"
            "${CMAKE_SOURCE_DIR}/compile_commands.json"
    COMMENT "将 compile_commands.json 复制到仓库根，供未配置 build 路径的 clangd 使用")
add_dependencies(sync_compile_commands excavator_api_tcp_client)

if(BUILD_TESTING)
    # Visual Studio 等多配置生成器需展开可执行文件路径，否则 ctest 显示 NOT_AVAILABLE
    add_test(NAME excavator_api_tcp_client_help COMMAND $<TARGET_FILE:excavator_api_tcp_client> --help)
endif()
