#include <excavator/internal/excavator_motor_driver.hpp>

#include <cstring>
#include <utility>

#if defined(__linux__)
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace excavator {
namespace {

inline constexpr std::uint64_t kCanShmMagic = 0x42494C4E41435F32ULL;

std::string normalize_shm_name(const std::string& name) {
    if (name.empty()) {
        return "/canlib_shm";
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

}  // namespace

struct ExcavatorMotorDriver::Impl {
    std::string shm_name;
    bool create_mapping{false};
    bool opened{false};
#if defined(__linux__)
    int shm_fd{-1};
    canlib::CanSharedMemoryLayout* shm{nullptr};
#endif

    bool open_impl() {
#if defined(__linux__)
        close_impl();
        const std::string n = normalize_shm_name(shm_name);
        const int flags = create_mapping ? (O_CREAT | O_RDWR) : O_RDWR;
        shm_fd = shm_open(n.c_str(), flags, 0660);
        if (shm_fd < 0) {
            return false;
        }
        constexpr std::size_t kSize = sizeof(canlib::CanSharedMemoryLayout);
        if (create_mapping && ftruncate(shm_fd, static_cast<off_t>(kSize)) != 0) {
            close_impl();
            return false;
        }
        void* mapped = mmap(nullptr, kSize, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
        if (mapped == MAP_FAILED) {
            close_impl();
            return false;
        }
        shm = static_cast<canlib::CanSharedMemoryLayout*>(mapped);
        if (create_mapping) {
            std::memset(shm, 0, sizeof(canlib::CanSharedMemoryLayout));
            shm->magic = kCanShmMagic;
        } else if (shm->magic != kCanShmMagic) {
            close_impl();
            return false;
        }
        opened = true;
        return true;
#else
        return false;
#endif
    }

    void close_impl() {
#if defined(__linux__)
        if (shm) {
            munmap(shm, sizeof(canlib::CanSharedMemoryLayout));
            shm = nullptr;
        }
        if (shm_fd >= 0) {
            ::close(shm_fd);
            shm_fd = -1;
        }
#endif
        opened = false;
    }
};

ExcavatorMotorDriver::ExcavatorMotorDriver(std::string shm_name, bool create_mapping) : impl_(new Impl()) {
    impl_->shm_name = std::move(shm_name);
    impl_->create_mapping = create_mapping;
}

ExcavatorMotorDriver::~ExcavatorMotorDriver() {
    close();
    delete impl_;
    impl_ = nullptr;
}

bool ExcavatorMotorDriver::open() {
    if (!impl_) {
        return false;
    }
    return impl_->open_impl();
}

void ExcavatorMotorDriver::close() {
    if (!impl_) {
        return;
    }
    impl_->close_impl();
}

bool ExcavatorMotorDriver::isOpen() const {
    return impl_ && impl_->opened;
}

void ExcavatorMotorDriver::writeMotionCommand(const canlib::Slave50HzDataA& cmd_a,
                                              const canlib::Slave50HzDataB& cmd_b) {
    if (!impl_ || !impl_->shm) {
        return;
    }
    impl_->shm->cmd_50hz_a = cmd_a;
    impl_->shm->cmd_50hz_b = cmd_b;
    ++impl_->shm->cmd_sequence;
}

void ExcavatorMotorDriver::writeStatusCommand(const canlib::Slave100msData& cmd_10hz) {
    if (!impl_ || !impl_->shm) {
        return;
    }
    impl_->shm->cmd_10hz = cmd_10hz;
    ++impl_->shm->cmd_sequence;
}

void ExcavatorMotorDriver::readMotorRpm(Vector8d& out) const {
    out.setConstant(kMotorSpeedRawZero);
    if (!impl_ || !impl_->shm) {
        return;
    }
    for (int i = 0; i < kAxisCount; ++i) {
        out(i) = impl_->shm->motor_rpm_feedback[static_cast<std::size_t>(i)];
    }
}

void ExcavatorMotorDriver::readStatus(bool simulation, Vector12i& out) const {
    out.setZero();
    if (!impl_ || !impl_->shm) {
        return;
    }
    const auto& s = simulation ? impl_->shm->cmd_10hz : impl_->shm->fb_10hz;
    copy_status_from_10hz(s, out);
}

}  // namespace excavator
