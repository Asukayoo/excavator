#pragma once

#include <excavator_api/types.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace excavator_api {

class ExcavatorReceive {
public:
    ExcavatorReceive();
    ~ExcavatorReceive();

    bool connect(const SessionConfig& config);
    bool close();

    // 阻塞获取同拍 ref/resp 快照；超时返回 false。
    bool get(Snapshot& out, std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));

    std::string lastError() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace excavator_api
