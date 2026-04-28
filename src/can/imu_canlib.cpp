#include <can/internal/imu_canlib.hpp>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>

#if defined(__linux__)
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace canlib {
namespace {

inline constexpr std::size_t kCanPayloadSize = kImuCanPayloadBytes;
static_assert(kCanPayloadSize == kImuCanPayloadBytes, "imu payload");
inline constexpr std::uint64_t kImuShmMagic = 0x494D555F43414E31ULL;
inline constexpr auto kLoopPeriod = std::chrono::milliseconds(20);  // 50Hz
inline constexpr auto kOfflineTimeout = std::chrono::milliseconds(100);

std::string normalize_shm_name(const std::string& name) {
    if (name.empty()) return "/imu_canlib_shm";
    if (name.front() == '/') return name;
    return "/" + name;
}

std::uint16_t get_u16_le(const std::array<std::uint8_t, kCanPayloadSize>& in, std::size_t idx) {
    return static_cast<std::uint16_t>(static_cast<std::uint16_t>(in[idx]) |
                                      static_cast<std::uint16_t>(in[idx + 1] << 8));
}

std::int16_t get_i16_le(const std::array<std::uint8_t, kCanPayloadSize>& in, std::size_t idx) {
    return static_cast<std::int16_t>(get_u16_le(in, idx));
}

float get_f32_le(const std::array<std::uint8_t, kCanPayloadSize>& in, std::size_t idx) {
    float out = 0.0F;
    std::memcpy(&out, &in[idx], sizeof(float));
    return out;
}

std::uint32_t get_u32_le(const std::array<std::uint8_t, kCanPayloadSize>& in, std::size_t idx) {
    return static_cast<std::uint32_t>(in[idx]) | (static_cast<std::uint32_t>(in[idx + 1]) << 8) |
           (static_cast<std::uint32_t>(in[idx + 2]) << 16) | (static_cast<std::uint32_t>(in[idx + 3]) << 24);
}

std::uint64_t now_ns() {
    const auto t = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(t).count());
}

// 协议欧拉角刻度为 0.01°；折到 [-180,180] 后转弧度
float euler_deg_to_rad_pm_pi(float deg) {
    while (deg > 180.0F) deg -= 360.0F;
    while (deg < -180.0F) deg += 360.0F;
    constexpr float kDegToRad = 3.14159265f / 180.0f;
    return deg * kDegToRad;
}

void builtin_imu_apply_can_payload_to_partials(std::uint16_t can_id,
                                                 const std::array<std::uint8_t, kCanPayloadSize>& payload,
                                                 std::array<ImuRxAccumulator, kImuDeviceCount>& partials) {
    const std::uint8_t device_addr = static_cast<std::uint8_t>(can_id & 0x07U);
    // 兼容两种设备地址编码：
    // - 0..3（零基）
    // - 1..4（一基）
    static int addr_mode = 0;  // 0:未知 1:零基 2:一基
    if (device_addr == 0U) addr_mode = 1;
    if (device_addr == kImuDeviceCount) addr_mode = 2;
    if (device_addr > kImuDeviceCount) return;

    std::size_t idx = 0;
    if (addr_mode == 1) {
        if (device_addr >= kImuDeviceCount) return;
        idx = static_cast<std::size_t>(device_addr);
    } else if (addr_mode == 2) {
        if (device_addr == 0U) return;
        idx = static_cast<std::size_t>(device_addr - 1U);
    } else {
        // 未探测到模式时保持历史默认（一基）行为。
        if (device_addr == 0U) return;
        idx = static_cast<std::size_t>(device_addr - 1U);
    }
    const std::uint8_t cmd = static_cast<std::uint8_t>((can_id >> 3U) & 0x07U);
    auto& pf = partials[idx];

    pf.last_rx_ns = now_ns();
    switch (cmd) {
        case 0x00:  // 欧拉角（总线 0.01° → 解析后统一弧度）
            pf.roll_rad = euler_deg_to_rad_pm_pi(static_cast<float>(get_i16_le(payload, 0)) * 0.01F);
            pf.pitch_rad = euler_deg_to_rad_pm_pi(static_cast<float>(get_i16_le(payload, 2)) * 0.01F);
            pf.yaw_rad = euler_deg_to_rad_pm_pi(static_cast<float>(get_i16_le(payload, 4)) * 0.01F);
            pf.has_euler = true;
            break;
        case 0x01:  // 角速率
            pf.gyro_x_dps = static_cast<float>(get_i16_le(payload, 0)) * 0.1F;
            pf.gyro_y_dps = static_cast<float>(get_i16_le(payload, 2)) * 0.1F;
            pf.gyro_z_dps = static_cast<float>(get_i16_le(payload, 4)) * 0.1F;
            pf.has_gyro = true;
            break;
        case 0x02:  // 加速度
            pf.accel_x_mps2 = static_cast<float>(get_i16_le(payload, 0)) * 0.1F;
            pf.accel_y_mps2 = static_cast<float>(get_i16_le(payload, 2)) * 0.1F;
            pf.accel_z_mps2 = static_cast<float>(get_i16_le(payload, 4)) * 0.1F;
            pf.has_accel = true;
            break;
        case 0x03:  // 四元数1 q0 q1
            pf.q0 = get_f32_le(payload, 0);
            pf.q1 = get_f32_le(payload, 4);
            pf.has_quat_1 = true;
            break;
        case 0x04:  // 四元数2 q2 q3
            pf.q2 = get_f32_le(payload, 0);
            pf.q3 = get_f32_le(payload, 4);
            pf.has_quat_2 = true;
            break;
        case 0x05:  // 数据状态
            pf.timestamp_ms = get_u32_le(payload, 0);
            pf.valid_flags = payload[4];
            pf.has_status = true;
            break;
        default:
            break;
    }
}

}  // namespace

void ImuDefaultCanFrameParser::parseFrame(std::uint16_t can_id,
                                          const std::array<std::uint8_t, kImuCanPayloadBytes>& payload,
                                          std::array<ImuRxAccumulator, kImuDeviceCount>& partials) {
    builtin_imu_apply_can_payload_to_partials(can_id, payload, partials);
}

struct ImuCanLib::Impl {
    std::string can_if_name;
    std::string shm_name;
    bool create_mapping{false};
    bool opened{false};
    std::atomic<bool> running{false};
    std::thread worker;
    std::string last_error;
    std::atomic<bool> simulation_enabled{true};
    std::atomic<std::uint64_t> loop_tick{0};
    ImuDefaultCanFrameParser default_frame_parser_;
    std::unique_ptr<ImuCanFrameParser> frame_parser_;

    ImuCanFrameParser* effective_frame_parser() {
        return frame_parser_ ? frame_parser_.get() : &default_frame_parser_;
    }

#if defined(__linux__)
    int can_fd{-1};
    int shm_fd{-1};
    ImuSharedMemoryLayout* shm_view{nullptr};
#endif

    std::array<ImuRxAccumulator, kImuDeviceCount> partials{};

    ~Impl() { close_impl(); }

    bool open_impl() {
#if defined(__linux__)
        last_error.clear();
        close_impl();
        if (!open_shm()) return false;
        if (!simulation_enabled.load(std::memory_order_acquire)) {
            if (!open_can()) {
                close_shm();
                return false;
            }
        }
        opened = true;
        return true;
#else
        return false;
#endif
    }

    bool start_impl() {
        if (!opened || running.load(std::memory_order_acquire)) return false;
        loop_tick.store(0, std::memory_order_release);
        running.store(true, std::memory_order_release);
        worker = std::thread([this] { loop(); });
        return true;
    }

    bool stop_impl() {
        running.store(false, std::memory_order_release);
        if (worker.joinable()) worker.join();
        return true;
    }

    void close_impl() {
        stop_impl();
#if defined(__linux__)
        close_can();
        close_shm();
#endif
        opened = false;
    }

    bool is_open_impl() const { return opened; }

#if defined(__linux__)
    bool open_shm() {
        const std::string normalized = normalize_shm_name(shm_name);
        const int flags = create_mapping ? (O_CREAT | O_RDWR) : O_RDWR;
        shm_fd = shm_open(normalized.c_str(), flags, 0660);
        if (shm_fd < 0) {
            last_error = "imu shm_open 失败: " + std::string(std::strerror(errno));
            return false;
        }
        constexpr std::size_t kSize = sizeof(ImuSharedMemoryLayout);
        if (create_mapping && ftruncate(shm_fd, static_cast<off_t>(kSize)) != 0) {
            last_error = "imu ftruncate 失败: " + std::string(std::strerror(errno));
            close_shm();
            return false;
        }
        void* mapped = mmap(nullptr, kSize, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
        if (mapped == MAP_FAILED) {
            last_error = "imu mmap 失败: " + std::string(std::strerror(errno));
            close_shm();
            return false;
        }
        shm_view = static_cast<ImuSharedMemoryLayout*>(mapped);
        if (create_mapping) {
            std::memset(shm_view, 0, sizeof(ImuSharedMemoryLayout));
            shm_view->magic = kImuShmMagic;
            for (std::size_t i = 0; i < kImuDeviceCount; ++i) {
                shm_view->imus[i].device_addr = static_cast<std::uint8_t>(i + 1);
            }
        } else if (shm_view->magic != kImuShmMagic) {
            last_error = "imu 共享内存 magic 不匹配";
            close_shm();
            return false;
        }
        return true;
    }

    bool open_can() {
        can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_fd < 0) {
            last_error = "imu socket(PF_CAN) 失败: " + std::string(std::strerror(errno));
            return false;
        }

        can_filter filter{};
        filter.can_id = kImuBaseIdHighSpeedCh1;
        filter.can_mask = 0x780U;
        if (setsockopt(can_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) != 0) {
            last_error = "imu setsockopt(CAN_RAW_FILTER) 失败: " + std::string(std::strerror(errno));
            close_can();
            return false;
        }

        ifreq ifr{};
        std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", can_if_name.c_str());
        if (ioctl(can_fd, SIOCGIFINDEX, &ifr) < 0) {
            last_error = "imu ioctl(SIOCGIFINDEX) 失败: " + std::string(std::strerror(errno));
            close_can();
            return false;
        }

        sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(can_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            last_error = "imu bind(can) 失败: " + std::string(std::strerror(errno));
            close_can();
            return false;
        }

        const int fd_flags = fcntl(can_fd, F_GETFL, 0);
        if (fd_flags < 0 || fcntl(can_fd, F_SETFL, fd_flags | O_NONBLOCK) < 0) {
            last_error = "imu fcntl(O_NONBLOCK) 失败: " + std::string(std::strerror(errno));
            close_can();
            return false;
        }
        return true;
    }

    void close_shm() {
        if (shm_view) {
            munmap(shm_view, sizeof(ImuSharedMemoryLayout));
            shm_view = nullptr;
        }
        if (shm_fd >= 0) {
            ::close(shm_fd);
            shm_fd = -1;
        }
    }

    void close_can() {
        if (can_fd >= 0) {
            ::close(can_fd);
            can_fd = -1;
        }
    }

    void publish_simulation_defaults() {
        if (!shm_view) return;
        const std::uint64_t tns = now_ns();
        const std::uint32_t tms = static_cast<std::uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
        for (std::size_t i = 0; i < kImuDeviceCount; ++i) {
            auto& d = shm_view->imus[i];
            d.device_addr = static_cast<std::uint8_t>(i + 1U);
            d.online = 1U;
            d.valid_attitude = 1U;
            d.valid_gyro = 1U;
            d.valid_accel = 1U;
            d.packet_loss_count = 0;
            d.imu_timestamp_ms = tms;
            d.host_rx_time_ns = tns;
            d.rpy_rad.setZero();
            d.gyro_dps.setZero();
            d.accel_mps2.setZero();
            d.quaternion = Eigen::Quaternionf(1.0F, 0.0F, 0.0F, 0.0F);
        }
        ++shm_view->sequence;
    }

    void drain_can() {
        if (can_fd < 0) return;
        while (true) {
            can_frame frame{};
            const ssize_t n = recv(can_fd, &frame, sizeof(frame), MSG_DONTWAIT);
            if (n < 0) break;
            if (n != static_cast<ssize_t>(sizeof(frame)) || frame.can_dlc < kCanPayloadSize) continue;
            if ((frame.can_id & CAN_EFF_FLAG) != 0) continue;  // 只处理标准帧
            const std::uint16_t can_id = static_cast<std::uint16_t>(frame.can_id & CAN_SFF_MASK);
            const std::uint16_t func = static_cast<std::uint16_t>((can_id >> 6U) & 0x1FU);
            if (func != 0x08U) continue;
            std::array<std::uint8_t, kCanPayloadSize> payload{};
            std::memcpy(payload.data(), frame.data, kCanPayloadSize);
            effective_frame_parser()->parseFrame(can_id, payload, partials);
        }
    }

    void publish_shm() {
        if (!shm_view) return;
        const std::uint64_t now = now_ns();
        for (std::size_t i = 0; i < kImuDeviceCount; ++i) {
            const auto& pf = partials[i];
            auto& dst = shm_view->imus[i];
            dst.device_addr = static_cast<std::uint8_t>(i + 1U);
            dst.online = (pf.last_rx_ns != 0U && (now - pf.last_rx_ns) <=
                                                 static_cast<std::uint64_t>(
                                                     std::chrono::duration_cast<std::chrono::nanoseconds>(kOfflineTimeout)
                                                         .count()))
                             ? 1U
                             : 0U;
            dst.valid_attitude = ((pf.valid_flags & 0x01U) != 0U) ? 1U : 0U;
            dst.valid_gyro = ((pf.valid_flags & 0x02U) != 0U) ? 1U : 0U;
            dst.valid_accel = ((pf.valid_flags & 0x04U) != 0U) ? 1U : 0U;
            dst.imu_timestamp_ms = pf.timestamp_ms;
            dst.host_rx_time_ns = pf.last_rx_ns;
            if (pf.has_euler) {
                dst.rpy_rad(0) = pf.roll_rad;
                dst.rpy_rad(1) = pf.pitch_rad;
                dst.rpy_rad(2) = pf.yaw_rad;
            }
            if (pf.has_gyro) {
                dst.gyro_dps(0) = pf.gyro_x_dps;
                dst.gyro_dps(1) = pf.gyro_y_dps;
                dst.gyro_dps(2) = pf.gyro_z_dps;
            }
            if (pf.has_accel) {
                dst.accel_mps2(0) = pf.accel_x_mps2;
                dst.accel_mps2(1) = pf.accel_y_mps2;
                dst.accel_mps2(2) = pf.accel_z_mps2;
            }
            if (pf.has_quat_1 && pf.has_quat_2) {
                dst.quaternion = Eigen::Quaternionf(pf.q0, pf.q1, pf.q2, pf.q3);
            }
        }
        ++shm_view->sequence;
    }

    void loop() {
        using clock = std::chrono::steady_clock;
        auto next_tick = clock::now();
        while (running.load(std::memory_order_acquire)) {
            loop_tick.fetch_add(1, std::memory_order_acq_rel);
            next_tick += kLoopPeriod;
            if (simulation_enabled.load(std::memory_order_acquire)) {
                publish_simulation_defaults();
            } else {
                drain_can();
                publish_shm();
            }
            std::this_thread::sleep_until(next_tick);
        }
    }
#endif
};

ImuCanLib::ImuCanLib(std::string can_if_name, std::string shm_name, bool create_mapping)
    : impl_(std::make_unique<Impl>()) {
    impl_->can_if_name = std::move(can_if_name);
    impl_->shm_name = std::move(shm_name);
    impl_->create_mapping = create_mapping;
}

ImuCanLib::~ImuCanLib() = default;

bool ImuCanLib::open() {
    if (!impl_) return false;
    return impl_->open_impl();
}

bool ImuCanLib::start() {
    if (!impl_) return false;
    return impl_->start_impl();
}

bool ImuCanLib::stop() {
    if (!impl_) return false;
    return impl_->stop_impl();
}

bool ImuCanLib::close() {
    if (!impl_) return false;
    impl_->close_impl();
    return true;
}

bool ImuCanLib::isOpen() const {
    if (!impl_) return false;
    return impl_->is_open_impl();
}

std::string ImuCanLib::lastError() const {
    if (!impl_) return "imu canlib impl 为空";
    return impl_->last_error;
}

std::uint64_t ImuCanLib::loopTick() const {
    if (!impl_) return 0;
    return impl_->loop_tick.load(std::memory_order_acquire);
}

void ImuCanLib::setSimulationEnabled(bool enabled) {
    if (!impl_) return;
    impl_->simulation_enabled.store(enabled, std::memory_order_release);
}

bool ImuCanLib::isSimulationEnabled() const {
    if (!impl_) return false;
    return impl_->simulation_enabled.load(std::memory_order_acquire);
}

void ImuCanLib::setFrameParser(std::unique_ptr<ImuCanFrameParser> parser) {
    if (!impl_) return;
    impl_->frame_parser_ = std::move(parser);
}

ImuCanFrameParser* ImuCanLib::frameParser() {
    if (!impl_) return nullptr;
    return impl_->effective_frame_parser();
}

}  // namespace canlib
