#include <excavator/internal/excavator_communication.hpp>

#include <can/internal/excavator_canlib.hpp>
#include <can/internal/imu_canlib.hpp>
#include <excavator/internal/excavator_data_type.hpp>

#include <cstring>
#include <string>

#if defined(__linux__)
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace excavator {
namespace {

inline constexpr std::uint64_t kCanShmMagic = 0x42494C4E41435F32ULL;
inline constexpr std::uint64_t kImuShmMagic = 0x494D555F43414E31ULL;

std::string normalize_can_shm_name(const std::string& name) {
    if (name.empty()) {
        return "/canlib_shm";
    }
    if (name.front() == '/') {
        return name;
    }
    return "/" + name;
}

std::string normalize_imu_shm_name(const std::string& name) {
    if (name.empty()) {
        return "/imu_canlib_shm";
    }
    if (name.front() == '/') {
        return name;
    }
    return "/" + name;
}

void copy_status_from_10hz(const canlib::Slave100msData& src, Vector12i& dst) {
    dst(0) = src.ignition;
    dst(1) = src.flameout;
    dst(2) = src.crush;
    dst(3) = src.chassis_light;
    dst(4) = src.remote_mode;
    dst(5) = src.pilot;
    dst(6) = src.high_speed;
    dst(7) = src.chassis_dozer_mode;
    dst(8) = src.horn;
    dst(9) = src.motor_gear;
    dst(10) = src.estop;
    dst(11) = src.standby;
}

void clear_imu_hw(ExcavatorHardwareState& hw) {
    for (auto& d : hw.imu.devices) {
        d = ExcavatorImuHardwareState::ImuSample{};
    }
}

double clamp_velocity(double v) {
    if (v > kHalfPi) return kHalfPi;
    if (v < -kHalfPi) return -kHalfPi;
    return v;
}

std::uint16_t velocity_to_raw(double v) {
    const double nv = (clamp_velocity(v) + kHalfPi) / (2.0 * kHalfPi);
    const double raw = static_cast<double>(canlib::kAxisMin) +
                       nv * static_cast<double>(canlib::kAxisMax - canlib::kAxisMin);
    return static_cast<std::uint16_t>(raw + 0.5);
}

std::uint16_t command_to_raw(double v) {
    // 兼容两种输入：原始码值(1638~14742) 或 速度值(-pi/2~pi/2)。
    if (v >= static_cast<double>(canlib::kAxisMin) && v <= static_cast<double>(canlib::kAxisMax)) {
        return static_cast<std::uint16_t>(v + 0.5);
    }
    return velocity_to_raw(v);
}

void fill_50hz_from_motor_rpm(const Vector8d& rpm, canlib::Slave50HzDataA& out_a, canlib::Slave50HzDataB& out_b) {
    out_a.swing = command_to_raw(rpm(0));
    out_a.arm = command_to_raw(rpm(1));
    out_a.boom = command_to_raw(rpm(2));
    out_a.bucket = command_to_raw(rpm(3));
    out_b.left_track = command_to_raw(rpm(4));
    out_b.right_track = command_to_raw(rpm(5));
    out_b.boom_offset = command_to_raw(rpm(6));
    out_b.chassis_dozer = command_to_raw(rpm(7));
}

void fill_10hz_from_status(const Vector12i& status, canlib::Slave100msData& out) {
    out.ignition = static_cast<std::uint8_t>(status(0) != 0 ? 1 : 0);
    out.flameout = static_cast<std::uint8_t>(status(1) != 0 ? 1 : 0);
    out.crush = static_cast<std::uint8_t>(status(2) != 0 ? 1 : 0);
    out.chassis_light = static_cast<std::uint8_t>(status(3) != 0 ? 1 : 0);
    out.remote_mode = static_cast<std::uint8_t>(status(4) != 0 ? 1 : 0);
    out.pilot = static_cast<std::uint8_t>(status(5) != 0 ? 1 : 0);
    out.high_speed = static_cast<std::uint8_t>(status(6) != 0 ? 1 : 0);
    out.chassis_dozer_mode = static_cast<std::uint8_t>(status(7) != 0 ? 1 : 0);
    out.horn = static_cast<std::uint8_t>(status(8) != 0 ? 1 : 0);
    out.motor_gear = static_cast<std::uint8_t>(status(9) & 0x3);
    out.estop = static_cast<std::uint8_t>(status(10) != 0 ? 1 : 0);
    out.standby = static_cast<std::uint8_t>(status(11) != 0 ? 1 : 0);
}

}  // namespace

struct ExcavatorCommunication::Impl {
    std::string can_shm_name;
    std::string imu_shm_name;
    bool create_mapping{false};
    bool opened{false};
#if defined(__linux__)
    int can_shm_fd{-1};
    canlib::CanSharedMemoryLayout* can_view{nullptr};
    int imu_shm_fd{-1};
    const canlib::ImuSharedMemoryLayout* imu_view{nullptr};
#endif

    ~Impl() { close_impl(); }

    void close_impl() {
#if defined(__linux__)
        if (can_view) {
            munmap(can_view, sizeof(canlib::CanSharedMemoryLayout));
            can_view = nullptr;
        }
        if (can_shm_fd >= 0) {
            ::close(can_shm_fd);
            can_shm_fd = -1;
        }
        if (imu_view) {
            munmap(const_cast<canlib::ImuSharedMemoryLayout*>(imu_view), sizeof(canlib::ImuSharedMemoryLayout));
            imu_view = nullptr;
        }
        if (imu_shm_fd >= 0) {
            ::close(imu_shm_fd);
            imu_shm_fd = -1;
        }
#endif
        opened = false;
    }

    bool open_can_shm() {
#if defined(__linux__)
        const std::string n = normalize_can_shm_name(can_shm_name);
        const int flags = create_mapping ? (O_CREAT | O_RDWR) : O_RDWR;
        can_shm_fd = shm_open(n.c_str(), flags, 0660);
        if (can_shm_fd < 0) {
            return false;
        }
        constexpr std::size_t kSize = sizeof(canlib::CanSharedMemoryLayout);
        if (create_mapping && ftruncate(can_shm_fd, static_cast<off_t>(kSize)) != 0) {
            if (can_shm_fd >= 0) {
                ::close(can_shm_fd);
                can_shm_fd = -1;
            }
            return false;
        }
        void* mapped = mmap(nullptr, kSize, PROT_READ | PROT_WRITE, MAP_SHARED, can_shm_fd, 0);
        if (mapped == MAP_FAILED) {
            if (can_shm_fd >= 0) {
                ::close(can_shm_fd);
                can_shm_fd = -1;
            }
            return false;
        }
        can_view = static_cast<canlib::CanSharedMemoryLayout*>(mapped);
        if (create_mapping) {
            std::memset(can_view, 0, sizeof(canlib::CanSharedMemoryLayout));
            can_view->magic = kCanShmMagic;
        } else if (can_view->magic != kCanShmMagic) {
            munmap(can_view, sizeof(canlib::CanSharedMemoryLayout));
            can_view = nullptr;
            ::close(can_shm_fd);
            can_shm_fd = -1;
            return false;
        }
        return true;
#else
        return false;
#endif
    }

    bool try_open_imu_shm() {
#if defined(__linux__)
        const std::string n = normalize_imu_shm_name(imu_shm_name);
        imu_shm_fd = shm_open(n.c_str(), O_RDONLY, 0660);
        if (imu_shm_fd < 0) {
            return false;
        }
        constexpr std::size_t kSize = sizeof(canlib::ImuSharedMemoryLayout);
        void* mapped = mmap(nullptr, kSize, PROT_READ, MAP_SHARED, imu_shm_fd, 0);
        if (mapped == MAP_FAILED) {
            ::close(imu_shm_fd);
            imu_shm_fd = -1;
            return false;
        }
        auto* p = static_cast<const canlib::ImuSharedMemoryLayout*>(mapped);
        if (p->magic != kImuShmMagic) {
            munmap(const_cast<canlib::ImuSharedMemoryLayout*>(p), sizeof(canlib::ImuSharedMemoryLayout));
            imu_shm_fd = -1;
            return false;
        }
        imu_view = p;
        return true;
#else
        return false;
#endif
    }

    bool open_impl() {
#if defined(__linux__)
        close_impl();
        if (!open_can_shm()) {
            return false;
        }
        (void)try_open_imu_shm();
        opened = true;
        return true;
#else
        return false;
#endif
    }

    canlib::CanSharedMemoryLayout* can_layout() {
#if defined(__linux__)
        return opened ? can_view : nullptr;
#else
        return nullptr;
#endif
    }
    const canlib::CanSharedMemoryLayout* can_layout() const {
#if defined(__linux__)
        return opened ? can_view : nullptr;
#else
        return nullptr;
#endif
    }
};

ExcavatorCommunication::ExcavatorCommunication(std::string can_mapping_name, bool create_mapping,
                                               std::string imu_mapping_name)
    : impl_(std::make_unique<Impl>()) {
    impl_->can_shm_name = std::move(can_mapping_name);
    impl_->create_mapping = create_mapping;
    impl_->imu_shm_name = std::move(imu_mapping_name);
}

ExcavatorCommunication::~ExcavatorCommunication() = default;

bool ExcavatorCommunication::open() {
    has_valid_imu_ = false;
    return impl_ && impl_->open_impl();
}

bool ExcavatorCommunication::close() {
    if (!impl_) {
        return false;
    }
    impl_->close_impl();
    has_valid_imu_ = false;
    clear_imu_hw(hw_state_);
    return true;
}

bool ExcavatorCommunication::read() {
    const auto* sh = impl_ ? impl_->can_layout() : nullptr;
    if (!sh || !impl_->opened) {
        return false;
    }
    for (int i = 0; i < kAxisCount; ++i) {
        hw_state_.motor.motor_rpm(i) = sh->motor_rpm_feedback[static_cast<std::size_t>(i)];
    }
    copy_status_from_10hz(sh->fb_10hz, hw_state_.motor.status);
#if defined(__linux__)
    if (impl_->imu_view) {
        canlib::ImuSharedMemoryLayout snap{};
        bool valid = false;
        for (int attempt = 0; attempt < 2 && !valid; ++attempt) {
            const std::uint64_t s1 = impl_->imu_view->sequence;
            std::memcpy(&snap, impl_->imu_view, sizeof(canlib::ImuSharedMemoryLayout));
            const std::uint64_t s2 = impl_->imu_view->sequence;
            valid = (s1 == s2 && snap.magic == kImuShmMagic);
        }
        if (valid) {
            for (std::size_t i = 0; i < kImuDeviceCount; ++i) {
                hw_state_.imu.devices[i].device_addr = snap.imus[i].device_addr;
                hw_state_.imu.devices[i].online = snap.imus[i].online;
                hw_state_.imu.devices[i].valid_attitude = snap.imus[i].valid_attitude;
                hw_state_.imu.devices[i].valid_gyro = snap.imus[i].valid_gyro;
                hw_state_.imu.devices[i].valid_accel = snap.imus[i].valid_accel;
                hw_state_.imu.devices[i].reserved0 = snap.imus[i].reserved0;
                hw_state_.imu.devices[i].packet_loss_count = snap.imus[i].packet_loss_count;
                hw_state_.imu.devices[i].imu_timestamp_ms = snap.imus[i].imu_timestamp_ms;
                hw_state_.imu.devices[i].host_rx_time_ns = snap.imus[i].host_rx_time_ns;
                hw_state_.imu.devices[i].rpy_rad = snap.imus[i].rpy_rad;
                hw_state_.imu.devices[i].gyro_dps = snap.imus[i].gyro_dps;
                hw_state_.imu.devices[i].accel_mps2 = snap.imus[i].accel_mps2;
                hw_state_.imu.devices[i].quaternion = snap.imus[i].quaternion;
            }
            has_valid_imu_ = true;
        } else {
            // 快照不一致时保持上一帧，避免单周期整帧置零。
            if (!has_valid_imu_) clear_imu_hw(hw_state_);
        }
    } else {
        // IMU 映射暂不可用时保持上一帧；仅冷启动无有效帧时清零。
        if (!has_valid_imu_) clear_imu_hw(hw_state_);
    }
#else
    if (!has_valid_imu_) clear_imu_hw(hw_state_);
#endif
    return true;
}

bool ExcavatorCommunication::write() {
    auto* sh = impl_ ? impl_->can_layout() : nullptr;
    if (!sh || !impl_->opened) {
        return false;
    }
    canlib::Slave50HzDataA cmd_50hz_a{};
    canlib::Slave50HzDataB cmd_50hz_b{};
    canlib::Slave100msData cmd_10hz{};
    fill_50hz_from_motor_rpm(hw_cmd_.motor_rpm, cmd_50hz_a, cmd_50hz_b);
    fill_10hz_from_status(hw_cmd_.status, cmd_10hz);
    sh->cmd_50hz_a = cmd_50hz_a;
    sh->cmd_50hz_b = cmd_50hz_b;
    sh->cmd_10hz = cmd_10hz;
    ++sh->cmd_sequence;
    return true;
}

bool ExcavatorCommunication::isOpen() const {
    return impl_ && impl_->opened;
}

}  // namespace excavator
