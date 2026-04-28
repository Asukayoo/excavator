#pragma once
#include "data_types.hpp"

/** @file communication_interface.hpp 依赖：data_types.hpp */

/** 与伺服侧交换 HardwareState / HardwareCommand（如无参 read/write，缓冲见具体实现） */
class CommunicationInterface {
public:
    virtual ~CommunicationInterface() = default;

    virtual bool open() = 0;
    virtual bool close() = 0;
    virtual bool read() = 0;
    virtual bool write() = 0;
    virtual bool isOpen() const = 0;
};
