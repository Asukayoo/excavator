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

bool load_pid_vectors_from_yaml(const std::string& yaml_path, std::vector<std::vector<double>>& pid_vectors) {
    std::ifstream fin(yaml_path);
    if (!fin.is_open()) return false;
    const std::vector<std::string> keys = {
        "position_kp", "position_ki", "position_kd", "velocity_kp",
        "velocity_ki", "velocity_kd", "velocity_scalar_max"};
    pid_vectors.assign(keys.size(), std::vector<double>{});
    std::string line;
    while (std::getline(fin, line)) {
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

bool connect_tcp(const std::string& host, int port, int& sock_out) {
    sock_out = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_out < 0) return false;
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<std::uint16_t>(port));
    if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
        (void)close(sock_out);
        sock_out = -1;
        return false;
    }
    if (connect(sock_out, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
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
    std::string pid_yaml_path;
    for (int i = 1; i + 1 < argc; ++i) {
        const std::string k = argv[i];
        const std::string v = argv[i + 1];
        if (k == "--host") host = v;
        if (k == "--port") port = std::stoi(v);
        if (k == "--can-if") can_if = v;
        if (k == "--imu-if") imu_if = v;
        if (k == "--can-shm") can_shm = v;
        if (k == "--imu-shm") imu_shm = v;
        if (k == "--can-sim") can_sim = (v == "1" || v == "true");
        if (k == "--imu-sim") imu_sim = (v == "1" || v == "true");
        if (k == "--can-bus") can_bus_enabled = (v == "1" || v == "true");
        if (k == "--pid-yaml") pid_yaml_path = v;
    }

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
    if (!connect_tcp(host, port, sock)) {
        std::cerr << "连接 tcp server 失败: " << host << ":" << port << "\n";
        (void)control.close();
        (void)receive.close();
        return 1;
    }
    std::vector<std::uint8_t> buf;
    buf.reserve(2048);
    std::vector<std::uint8_t> chunk(512);
    RxView rx_view{};
    bool running = true;
    const std::filesystem::path log_root =
        std::filesystem::path("log") / tcp_demo_log::makeTimestampDirName();
    tcp_demo_log::ensureLogDirs(log_root);
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
            // 每周期落盘一次：数据文件仅保存纯数据，时间戳单独保存。
            tcp_demo_log::appendState(log_root, "ref", snap.ref);
            tcp_demo_log::appendState(log_root, "resp", snap.resp);
            tcp_demo_log::appendTimestamp(log_root, snap.meta.recv_time_ns);
            (void)rx_view;
        }
    }

    (void)close(sock);
    (void)control.close();
    (void)receive.close();
    return 0;
}
