#pragma once

#include <can/internal/excavator_canlib.hpp>
#include <excavator/internal/excavator_data_type.hpp>

#include <string>

namespace excavator {

class ExcavatorMotorDriver {
public:
    ExcavatorMotorDriver(std::string shm_name, bool create_mapping);
    ~ExcavatorMotorDriver();

    bool open();
    void close();
    bool isOpen() const;

    void writeMotionCommand(const canlib::Slave50HzDataA& cmd_a, const canlib::Slave50HzDataB& cmd_b);
    void writeStatusCommand(const canlib::Slave100msData& cmd_10hz);

    void readMotorRpm(Vector8d& out) const;
    void readStatus(bool simulation, Vector12i& out) const;

private:
    struct Impl;
    Impl* impl_{nullptr};
};

}  // namespace excavator
