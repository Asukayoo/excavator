#pragma once

#include <excavator_api/types.hpp>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

namespace tcp_demo_log {

inline std::string makeTimestampDirName() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now{};
#if defined(_WIN32)
    localtime_s(&tm_now, &t);
#else
    localtime_r(&t, &tm_now);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y%m%d_%H%M%S");
    return oss.str();
}

inline void ensureLogDirs(const std::filesystem::path& base_dir) {
    std::filesystem::create_directories(base_dir / "ref");
    std::filesystem::create_directories(base_dir / "resp");
}

inline std::string vec8ToLine(const excavator_api::Vector8d& v) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    for (int i = 0; i < 8; ++i) {
        if (i != 0) oss << " ";
        oss << v(i);
    }
    return oss.str();
}

inline std::string vec12ToLine(const excavator_api::Vector12i& v) {
    std::ostringstream oss;
    for (int i = 0; i < 12; ++i) {
        if (i != 0) oss << " ";
        oss << v(i);
    }
    return oss.str();
}

inline void appendLine(const std::filesystem::path& file, const std::string& line) {
    std::ofstream ofs(file, std::ios::app);
    ofs << line << "\n";
}

inline void appendState(
    const std::filesystem::path& root,
    const std::string& group,
    const excavator_api::RefState& state) {
    const std::filesystem::path dir = root / group;
    appendLine(dir / "position.txt", vec8ToLine(state.position));
    appendLine(dir / "velocity.txt", vec8ToLine(state.velocity));
    appendLine(dir / "acceleration.txt", vec8ToLine(state.acceleration));
    appendLine(dir / "velocity_scalar.txt", vec8ToLine(state.velocity_scalar));
    appendLine(dir / "status.txt", vec12ToLine(state.status));
    appendLine(dir / "motor_rpm.txt", vec8ToLine(state.motor_rpm));
    appendLine(dir / "plan_rpm.txt", vec8ToLine(state.plan_rpm));
}

inline void appendState(
    const std::filesystem::path& root,
    const std::string& group,
    const excavator_api::RespState& state) {
    const std::filesystem::path dir = root / group;
    appendLine(dir / "position.txt", vec8ToLine(state.position));
    appendLine(dir / "velocity.txt", vec8ToLine(state.velocity));
    appendLine(dir / "acceleration.txt", vec8ToLine(state.acceleration));
    appendLine(dir / "velocity_scalar.txt", vec8ToLine(state.velocity_scalar));
    appendLine(dir / "status.txt", vec12ToLine(state.status));
    appendLine(dir / "motor_rpm.txt", vec8ToLine(state.motor_rpm));
    appendLine(dir / "plan_rpm.txt", vec8ToLine(state.plan_rpm));
}

inline void appendTimestamp(const std::filesystem::path& root, std::uint64_t recv_time_ns) {
    appendLine(root / "timestamp.txt", std::to_string(recv_time_ns));
}

}  // namespace tcp_demo_log
