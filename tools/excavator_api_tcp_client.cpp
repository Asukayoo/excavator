#include <excavator_api/excavator_control.hpp>
#include <excavator_api/excavator_receive.hpp>
#include "tcp_demo_log_utils.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cerrno>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

// 仓库约定默认 joint 配置（未指定 --pid-yaml 时使用）
inline constexpr const char kDefaultJointPidYaml[] = "config/joint_pid.yaml";

std::string resolve_default_pid_yaml_path() {
    namespace fs = std::filesystem;
    std::vector<fs::path> candidates;
#if defined(__linux__)
    std::error_code ec;
    const fs::path exe = fs::read_symlink("/proc/self/exe", ec);
    if (!ec) {
        candidates.push_back(exe.parent_path() / ".." / "config" / "joint_pid.yaml");
        candidates.push_back(exe.parent_path() / "config" / "joint_pid.yaml");
    }
#endif
    candidates.emplace_back(kDefaultJointPidYaml);
    candidates.emplace_back("../config/joint_pid.yaml");

    for (const auto& p : candidates) {
        std::error_code ec;
        if (!fs::exists(p, ec)) continue;
        std::error_code ec2;
        fs::path c = fs::weakly_canonical(p, ec2);
        return ec2 ? p.string() : c.string();
    }
    return std::string(kDefaultJointPidYaml);
}

inline constexpr std::uint32_t kServoPacketMagic = 0x56524553u;
inline constexpr std::uint32_t kServoPacketVersion3 = 3u;
inline constexpr std::uint32_t kStatusPacketVersion5 = 5u;

#pragma pack(push, 1)
struct ServoPacketV3 {
    std::uint32_t magic{0};
    std::uint32_t version{0};
    double joint_normalized[8]{};
    double motor_speed_normalized{0.0};
};
struct StatusPacketV5 {
    std::uint32_t magic{0};
    std::uint32_t version{0};
    double motor_speed_normalized{0.0};
    std::uint16_t toggle_mask{0};
    std::uint16_t reserved{0};
};
#pragma pack(pop)

inline constexpr std::size_t kServoPacketV3Bytes = sizeof(ServoPacketV3);
inline constexpr std::size_t kStatusPacketV5Bytes = sizeof(StatusPacketV5);

struct RxView {
    excavator_api::Vector8d speed_scalar = excavator_api::Vector8d::Zero();
    std::uint16_t toggle_mask{0};
};

double clamp_n(double v) { return std::clamp(v, -1.0, 1.0); }

bool parse_vector_values(const std::string& line, std::vector<double>& out_values) {
    const std::size_t l = line.find('[');
    const std::size_t r = line.find(']');
    if (l == std::string::npos || r == std::string::npos || r <= l) return false;
    std::string body = line.substr(l + 1, r - l - 1);
    for (char& c : body) {
        if (c == ',') c = ' ';
    }
    std::istringstream iss(body);
    out_values.clear();
    double v = 0.0;
    while (iss >> v) {
        out_values.push_back(v);
    }
    return out_values.size() == 8;
}

std::string trim_copy(const std::string& s) {
    const std::size_t first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    const std::size_t last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
}

bool parse_yaml_bool_value(const std::string& raw, bool& out_value) {
    const std::string value = trim_copy(raw);
    if (value == "1" || value == "true" || value == "True" || value == "TRUE") {
        out_value = true;
        return true;
    }
    if (value == "0" || value == "false" || value == "False" || value == "FALSE") {
        out_value = false;
        return true;
    }
    return false;
}

bool parse_yaml_string_value(const std::string& raw, std::string& out_value) {
    std::string value = trim_copy(raw);
    if (value.empty()) return false;
    if (value.front() == '"' && value.back() == '"' && value.size() >= 2) {
        value = value.substr(1, value.size() - 2);
    }
    if (value.front() == '\'' && value.back() == '\'' && value.size() >= 2) {
        value = value.substr(1, value.size() - 2);
    }
    out_value = value;
    return true;
}

// 解析 TCP 端口（1~65535）
bool parse_yaml_port_value(const std::string& raw, int& out_port) {
    const std::string value = trim_copy(raw);
    if (value.empty()) return false;
    try {
        std::size_t idx = 0;
        const long n = std::stol(value, &idx, 10);
        if (idx != value.size()) return false;
        if (n < 1 || n > 65535) return false;
        out_port = static_cast<int>(n);
        return true;
    } catch (...) {
        return false;
    }
}

struct RuntimeConfigFromYaml {
    bool has_host{false};
    std::string host;
    bool has_port{false};
    int port{0};
    bool has_can_if{false};
    std::string can_if;
    bool has_imu_if{false};
    std::string imu_if;
    bool has_can_sim{false};
    bool can_sim{false};
    bool has_imu_sim{false};
    bool imu_sim{false};
    bool has_can_bus_enabled{false};
    bool can_bus_enabled{true};
    bool has_save_log{false};
    bool save_log{false};
};

bool load_runtime_config_from_yaml(const std::string& yaml_path, RuntimeConfigFromYaml& out_cfg) {
    std::ifstream fin(yaml_path);
    if (!fin.is_open()) return false;
    out_cfg = RuntimeConfigFromYaml{};
    std::string line;
    while (std::getline(fin, line)) {
        const std::size_t comment_pos = line.find('#');
        if (comment_pos != std::string::npos) line = line.substr(0, comment_pos);
        const std::size_t sep_pos = line.find(':');
        if (sep_pos == std::string::npos) continue;
        const std::string key = trim_copy(line.substr(0, sep_pos));
        const std::string value = line.substr(sep_pos + 1);
        if (key == "can_if" || key == "can_if_name") {
            std::string parsed;
            if (!parse_yaml_string_value(value, parsed)) return false;
            out_cfg.can_if = parsed;
            out_cfg.has_can_if = true;
            continue;
        }
        if (key == "imu_if" || key == "imu_if_name") {
            std::string parsed;
            if (!parse_yaml_string_value(value, parsed)) return false;
            out_cfg.imu_if = parsed;
            out_cfg.has_imu_if = true;
            continue;
        }
        if (key == "can_simulation" || key == "can_sim") {
            bool parsed = false;
            if (!parse_yaml_bool_value(value, parsed)) return false;
            out_cfg.can_sim = parsed;
            out_cfg.has_can_sim = true;
            continue;
        }
        if (key == "imu_simulation" || key == "imu_sim") {
            bool parsed = false;
            if (!parse_yaml_bool_value(value, parsed)) return false;
            out_cfg.imu_sim = parsed;
            out_cfg.has_imu_sim = true;
            continue;
        }
        if (key == "can_bus_enabled" || key == "use_can") {
            bool parsed = false;
            if (!parse_yaml_bool_value(value, parsed)) return false;
            out_cfg.can_bus_enabled = parsed;
            out_cfg.has_can_bus_enabled = true;
            continue;
        }
        if (key == "host" || key == "hostname" || key == "tcp_host") {
            std::string parsed;
            if (!parse_yaml_string_value(value, parsed)) return false;
            out_cfg.host = parsed;
            out_cfg.has_host = true;
            continue;
        }
        if (key == "port" || key == "tcp_port") {
            int parsed = 0;
            if (!parse_yaml_port_value(value, parsed)) return false;
            out_cfg.port = parsed;
            out_cfg.has_port = true;
            continue;
        }
        if (key == "save_log") {
            bool parsed = false;
            if (!parse_yaml_bool_value(value, parsed)) return false;
            out_cfg.save_log = parsed;
            out_cfg.has_save_log = true;
            continue;
        }
    }
    return true;
}

bool load_pid_vectors_from_yaml(const std::string& yaml_path, std::vector<std::vector<double>>& pid_vectors) {
    std::ifstream fin(yaml_path);
    if (!fin.is_open()) return false;
    const std::vector<std::string> keys = {
        "position_kp", "position_ki", "position_kd", "velocity_kp",
        "velocity_ki", "velocity_kd", "velocity_scalar_max", "feedforward_scalar_threshold"};
    pid_vectors.assign(keys.size(), std::vector<double>{});
    std::string line;
    while (std::getline(fin, line)) {
        const std::size_t hash_pos = line.find('#');
        if (hash_pos != std::string::npos) {
            line = line.substr(0, hash_pos);
        }
        for (std::size_t i = 0; i < keys.size(); ++i) {
            const std::string key = keys[i] + ":";
            if (line.find(key) == std::string::npos) continue;
            std::vector<double> vals;
            if (!parse_vector_values(line, vals)) return false;
            pid_vectors[i] = std::move(vals);
        }
    }
    for (const auto& v : pid_vectors) {
        if (v.size() != 8) return false;
    }
    return true;
}

// 空行保持 default；0/1 为否/是
bool parse_bool_01_line(const std::string& line, bool default_val, bool& out) {
    const std::string t = trim_copy(line);
    if (t.empty()) {
        out = default_val;
        return true;
    }
    if (t == "0") {
        out = false;
        return true;
    }
    if (t == "1") {
        out = true;
        return true;
    }
    return false;
}

std::string prompt_string_or_default(const char* label, const std::string& current) {
    std::cout << "请输入 " << label << " (当前默认: " << current << ")，直接回车保持默认: ";
    std::string line;
    if (!std::getline(std::cin, line)) {
        return current;
    }
    const std::string t = trim_copy(line);
    return t.empty() ? current : t;
}

void prompt_port_or_default(int& port) {
    const int def = port;
    while (true) {
        std::cout << "请输入 port (当前默认: " << def << ")，直接回车保持默认: ";
        std::string line;
        if (!std::getline(std::cin, line)) {
            port = def;
            return;
        }
        if (trim_copy(line).empty()) {
            port = def;
            return;
        }
        int parsed = 0;
        if (parse_yaml_port_value(line, parsed)) {
            port = parsed;
            return;
        }
        std::cout << "端口无效，请输入 1-65535 的数字。\n";
    }
}

bool prompt_bool_01_or_default(const char* label, bool current_default) {
    while (true) {
        std::cout << "请输入 " << label << " (当前默认: " << (current_default ? 1 : 0)
                  << ")，0=否 1=是，直接回车保持默认: ";
        std::string line;
        if (!std::getline(std::cin, line)) {
            return current_default;
        }
        bool v = current_default;
        if (parse_bool_01_line(line, current_default, v)) {
            return v;
        }
        std::cout << "输入无效，请输入 0、1 或空行。\n";
    }
}

// 终端覆盖：否=依次询问
void maybe_interactive_override_runtime_config(std::string& host, int& port, std::string& can_if, std::string& imu_if,
                                               bool& can_sim, bool& imu_sim, bool& save_log) {
    std::cout << "是否使用当前默认配置 (YAML 与命令行已合并)? 1=是 0=否: ";
    int choice = 1;
    if (!(std::cin >> choice)) {
        if (!std::cin.eof()) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
        return;
    }
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    if (choice != 0) {
        return;
    }
    host = prompt_string_or_default("hostname (IPv4)", host);
    prompt_port_or_default(port);
    can_if = prompt_string_or_default("can_if", can_if);
    imu_if = prompt_string_or_default("imu_if", imu_if);
    can_sim = prompt_bool_01_or_default("can_simulation", can_sim);
    imu_sim = prompt_bool_01_or_default("imu_simulation", imu_sim);
    save_log = prompt_bool_01_or_default("save_log", save_log);
}

excavator_api::ControlMode choose_control_mode() {
    while (true) {
        std::cout << "请选择控制模式:\n"
                  << "  0: OpenLoopMotorSpeed\n"
                  << "  1: ClosedLoopJointPosition\n"
                  << "  2: ClosedLoopJointVelocity\n"
                  << "  3: ClosedLoopVelocityScalar\n"
                  << "请输入数字并回车: ";
        int selected = -1;
        if (!(std::cin >> selected)) {
            if (std::cin.eof()) return excavator_api::ControlMode::OpenLoopMotorSpeed;
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "输入无效，请输入 0-3。\n";
            continue;
        }
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        switch (selected) {
            case 0:
                return excavator_api::ControlMode::OpenLoopMotorSpeed;
            case 1:
                return excavator_api::ControlMode::ClosedLoopJointPosition;
            case 2:
                return excavator_api::ControlMode::ClosedLoopJointVelocity;
            case 3:
                return excavator_api::ControlMode::ClosedLoopVelocityScalar;
            default:
                std::cout << "输入越界，请输入 0-3。\n";
                break;
        }
    }
}

// err_out：失败原因（connect 在 close 前保存 errno）；connect_errno_out：仅 connect 失败时的 errno
bool connect_tcp(const std::string& host, int port, int& sock_out, std::string* err_out, int* connect_errno_out) {
    if (connect_errno_out) {
        *connect_errno_out = 0;
    }
    sock_out = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_out < 0) {
        if (err_out) *err_out = std::string("socket: ") + std::strerror(errno);
        return false;
    }
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<std::uint16_t>(port));
    if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
        if (err_out) *err_out = "IPv4 地址无效: " + host;
        (void)close(sock_out);
        sock_out = -1;
        return false;
    }
    if (connect(sock_out, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        const int e = errno;
        if (connect_errno_out) {
            *connect_errno_out = e;
        }
        if (err_out) {
            *err_out = std::string("connect: ") + std::strerror(e);
        }
        (void)close(sock_out);
        sock_out = -1;
        return false;
    }
    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 50000;  // 50ms
    (void)setsockopt(sock_out, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    return true;
}

void apply_toggle_mask(excavator_api::ExcavatorControl& control, std::uint16_t mask) {
    (void)control.applyStatusToggleMask(mask);
}

bool process_packet(excavator_api::ExcavatorControl& control,
                    RxView& rx_view,
                    const std::vector<std::uint8_t>& packet) {
    if (packet.size() < 8) return false;
    std::uint32_t magic = 0;
    std::uint32_t version = 0;
    std::memcpy(&magic, packet.data(), sizeof(magic));
    std::memcpy(&version, packet.data() + 4, sizeof(version));
    if (magic != kServoPacketMagic) return false;

    if (version == kServoPacketVersion3 && packet.size() == kServoPacketV3Bytes) {
        ServoPacketV3 sp{};
        std::memcpy(&sp, packet.data(), sizeof(sp));
        excavator_api::SpeedScalarCmd cmd{};
        for (int i = 0; i < 8; ++i) {
            cmd.speed_scalar(i) = clamp_n(sp.joint_normalized[static_cast<std::size_t>(i)]);
            rx_view.speed_scalar(i) = cmd.speed_scalar(i);
        }
        (void)control.sendCommand(cmd);
        return true;
    }
    if (version == kStatusPacketVersion5 && packet.size() == kStatusPacketV5Bytes) {
        StatusPacketV5 sp{};
        std::memcpy(&sp, packet.data(), sizeof(sp));
        rx_view.toggle_mask = sp.toggle_mask;
        apply_toggle_mask(control, sp.toggle_mask);
        return true;
    }
    return false;
}

}  // namespace

int main(int argc, char** argv) {
    std::string host = "127.0.0.1";
    int port = 29753;
    std::string can_if = "can0";
    std::string imu_if = "can1";
    std::string can_shm = "canlib_shm_tcp";
    std::string imu_shm = "imu_canlib_shm";
    bool can_sim = false;
    bool imu_sim = true;
    bool can_bus_enabled = true;
    bool save_log = false;
    std::string pid_yaml_path;
    for (int i = 1; i + 1 < argc; i += 2) {
        if (std::string(argv[i]) == "--pid-yaml") {
            pid_yaml_path = argv[i + 1];
        }
    }
    if (pid_yaml_path.empty()) {
        pid_yaml_path = resolve_default_pid_yaml_path();
    }
    if (!pid_yaml_path.empty()) {
        RuntimeConfigFromYaml yaml_runtime_cfg{};
        if (!load_runtime_config_from_yaml(pid_yaml_path, yaml_runtime_cfg)) {
            std::cerr << "运行参数 YAML 解析失败: " << pid_yaml_path << "\n";
            return 1;
        }
        if (yaml_runtime_cfg.has_host) host = yaml_runtime_cfg.host;
        if (yaml_runtime_cfg.has_port) port = yaml_runtime_cfg.port;
        if (yaml_runtime_cfg.has_can_if) can_if = yaml_runtime_cfg.can_if;
        if (yaml_runtime_cfg.has_imu_if) imu_if = yaml_runtime_cfg.imu_if;
        if (yaml_runtime_cfg.has_can_sim) can_sim = yaml_runtime_cfg.can_sim;
        if (yaml_runtime_cfg.has_imu_sim) imu_sim = yaml_runtime_cfg.imu_sim;
        if (yaml_runtime_cfg.has_can_bus_enabled) can_bus_enabled = yaml_runtime_cfg.can_bus_enabled;
        if (yaml_runtime_cfg.has_save_log) save_log = yaml_runtime_cfg.save_log;
    }
    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string k = argv[i];
        const std::string v = argv[i + 1];
        if (k == "--pid-yaml") {
            continue;
        }
        if (k == "--host") host = v;
        if (k == "--port") port = std::stoi(v);
        if (k == "--can-if") can_if = v;
        if (k == "--imu-if") imu_if = v;
        if (k == "--can-shm") can_shm = v;
        if (k == "--imu-shm") imu_shm = v;
        if (k == "--can-sim") can_sim = (v == "1" || v == "true");
        if (k == "--imu-sim") imu_sim = (v == "1" || v == "true");
        if (k == "--can-bus") can_bus_enabled = (v == "1" || v == "true");
    }

    maybe_interactive_override_runtime_config(host, port, can_if, imu_if, can_sim, imu_sim, save_log);

    excavator_api::SessionConfig cfg{};
    cfg.can_if_name = can_if;
    cfg.imu_if_name = imu_if;
    cfg.can_shm_name = can_shm;
    cfg.imu_shm_name = imu_shm;
    cfg.create_mapping = true;
    cfg.can_simulation = can_sim;
    cfg.imu_simulation = imu_sim;
    cfg.can_bus_enabled = can_bus_enabled;

    excavator_api::ExcavatorControl control;
    excavator_api::ExcavatorReceive receive;
    if (!control.connect(cfg) || !receive.connect(cfg) || !control.start()) {
        std::cerr << "runtime 启动失败: " << control.lastError() << "\n";
        return 1;
    }
    const excavator_api::ControlMode mode = choose_control_mode();
    (void)control.setControlMode(mode);
    if (!pid_yaml_path.empty()) {
        std::vector<std::vector<double>> pid_vectors;
        if (!load_pid_vectors_from_yaml(pid_yaml_path, pid_vectors) || !control.setPidVectors(pid_vectors)) {
            std::cerr << "PID YAML 加载失败: " << pid_yaml_path << "\n";
            (void)control.close();
            (void)receive.close();
            return 1;
        }
    }

    int sock = -1;
    std::string tcp_err;
    int tcp_connect_errno = 0;
    if (!connect_tcp(host, port, sock, &tcp_err, &tcp_connect_errno)) {
        std::cerr << "连接 tcp server 失败: " << host << ":" << port;
        if (!tcp_err.empty()) {
            std::cerr << " (" << tcp_err << ")";
        }
        std::cerr << "\n";
        if (tcp_connect_errno == ECONNREFUSED) {
            std::cerr << "提示: 对端该端口无进程监听，或服务端仅 bind 了 127.0.0.1；"
                         "请在服务端使用 --host 0.0.0.0 并确认已启动。\n";
        } else if (tcp_connect_errno == ETIMEDOUT) {
            std::cerr << "提示: TCP 超时，常见于防火墙丢弃 SYN；请检查主机 " << host << " 上 " << port
                      << "/tcp 是否放行。\n";
        }
        (void)control.close();
        (void)receive.close();
        return 1;
    }
    std::vector<std::uint8_t> buf;
    buf.reserve(2048);
    std::vector<std::uint8_t> chunk(512);
    RxView rx_view{};
    bool running = true;
    std::filesystem::path log_root;
    const bool logging_enabled = save_log;
    if (logging_enabled) {
        log_root = std::filesystem::path("log") / tcp_demo_log::makeTimestampDirName();
        tcp_demo_log::ensureLogDirs(log_root);
    }
    while (running) {
        const ssize_t n = recv(sock, chunk.data(), static_cast<long>(chunk.size()), 0);
        if (n > 0) {
            buf.insert(buf.end(), chunk.begin(), chunk.begin() + n);
            while (buf.size() >= 8U) {
                std::uint32_t magic = 0;
                std::uint32_t ver = 0;
                std::memcpy(&magic, buf.data(), 4);
                std::memcpy(&ver, buf.data() + 4, 4);
                if (magic != kServoPacketMagic) {
                    buf.erase(buf.begin());
                    continue;
                }
                std::size_t need = 0;
                if (ver == kServoPacketVersion3) need = kServoPacketV3Bytes;
                if (ver == kStatusPacketVersion5) need = kStatusPacketV5Bytes;
                if (need == 0) {
                    buf.erase(buf.begin());
                    continue;
                }
                if (buf.size() < need) break;
                std::vector<std::uint8_t> pkt(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(need));
                buf.erase(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(need));
                (void)process_packet(control, rx_view, pkt);
            }
        } else if (n == 0) {
            break;
        } else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            break;
        }

        excavator_api::Snapshot snap{};
        if (receive.get(snap, std::chrono::milliseconds(50))) {
            if (logging_enabled) {
                tcp_demo_log::appendState(log_root, "ref", snap.ref);
                tcp_demo_log::appendState(log_root, "resp", snap.resp);
                tcp_demo_log::appendTimestamp(log_root, snap.meta.recv_time_ns);
            }
            (void)rx_view;
        }
    }

    (void)close(sock);
    (void)control.close();
    (void)receive.close();
    return 0;
}
