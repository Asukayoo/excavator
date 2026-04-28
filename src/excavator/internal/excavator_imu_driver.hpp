#pragma once

#include <can/internal/imu_canlib.hpp>

#include <string>

namespace excavator {

class ExcavatorImuDriver {
public:
    explicit ExcavatorImuDriver(std::string shm_name);
    ~ExcavatorImuDriver();

    bool open();
    void close();
    bool isOpen() const;
    bool snapshot(canlib::ImuSharedMemoryLayout& out) const;

private:
    struct Impl;
    Impl* impl_{nullptr};
};

}  // namespace excavator
