#include <excavator/internal/excavator_imu_driver.hpp>

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

inline constexpr std::uint64_t kImuShmMagic = 0x494D555F43414E31ULL;

std::string normalize_shm_name(const std::string& name) {
    if (name.empty()) {
        return "/imu_canlib_shm";
    }
    if (name.front() == '/') {
        return name;
    }
    return "/" + name;
}

}  // namespace

struct ExcavatorImuDriver::Impl {
    std::string shm_name;
    bool opened{false};
#if defined(__linux__)
    int shm_fd{-1};
    const canlib::ImuSharedMemoryLayout* shm{nullptr};
#endif

    bool open_impl() {
#if defined(__linux__)
        close_impl();
        const std::string n = normalize_shm_name(shm_name);
        shm_fd = shm_open(n.c_str(), O_RDONLY, 0660);
        if (shm_fd < 0) {
            return false;
        }
        constexpr std::size_t kSize = sizeof(canlib::ImuSharedMemoryLayout);
        void* mapped = mmap(nullptr, kSize, PROT_READ, MAP_SHARED, shm_fd, 0);
        if (mapped == MAP_FAILED) {
            close_impl();
            return false;
        }
        shm = static_cast<const canlib::ImuSharedMemoryLayout*>(mapped);
        if (shm->magic != kImuShmMagic) {
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
            munmap(const_cast<canlib::ImuSharedMemoryLayout*>(shm), sizeof(canlib::ImuSharedMemoryLayout));
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

ExcavatorImuDriver::ExcavatorImuDriver(std::string shm_name) : impl_(new Impl()) {
    impl_->shm_name = std::move(shm_name);
}

ExcavatorImuDriver::~ExcavatorImuDriver() {
    close();
    delete impl_;
    impl_ = nullptr;
}

bool ExcavatorImuDriver::open() {
    if (!impl_) {
        return false;
    }
    return impl_->open_impl();
}

void ExcavatorImuDriver::close() {
    if (!impl_) {
        return;
    }
    impl_->close_impl();
}

bool ExcavatorImuDriver::isOpen() const {
    return impl_ && impl_->opened;
}

bool ExcavatorImuDriver::snapshot(canlib::ImuSharedMemoryLayout& out) const {
    if (!impl_ || !impl_->shm) {
        return false;
    }
    const std::uint64_t s1 = impl_->shm->sequence;
    std::memcpy(&out, impl_->shm, sizeof(canlib::ImuSharedMemoryLayout));
    const std::uint64_t s2 = impl_->shm->sequence;
    return s1 == s2 && out.magic == kImuShmMagic;
}

}  // namespace excavator
