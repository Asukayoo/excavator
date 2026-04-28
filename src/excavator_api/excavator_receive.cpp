#include <excavator_api/excavator_receive.hpp>

#include "excavator_runtime.hpp"

namespace excavator_api {
namespace {

std::uint64_t nowNs() {
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch());
    return static_cast<std::uint64_t>(ns.count());
}

}  // namespace

struct ExcavatorReceive::Impl {
    std::shared_ptr<ExcavatorRuntime> runtime{};
    std::string last_error{};
    std::uint64_t last_tick{0};
};

ExcavatorReceive::ExcavatorReceive() : impl_(std::make_unique<Impl>()) {}

ExcavatorReceive::~ExcavatorReceive() = default;

bool ExcavatorReceive::connect(const SessionConfig& config) {
    std::string err;
    auto runtime = RuntimeRegistry::acquire(config, err);
    if (!runtime) {
        impl_->last_error = err;
        return false;
    }
    impl_->runtime = std::move(runtime);
    impl_->last_tick = impl_->runtime->loopTick();
    impl_->last_error.clear();
    return true;
}

bool ExcavatorReceive::close() {
    impl_->runtime.reset();
    RuntimeRegistry::releaseUnused();
    return true;
}

bool ExcavatorReceive::get(Snapshot& out, std::chrono::milliseconds timeout) {
    if (!impl_->runtime) {
        impl_->last_error = "runtime 未连接";
        return false;
    }
    std::uint64_t tick = 0;
    if (!impl_->runtime->waitNextTick(impl_->last_tick, timeout, tick)) {
        impl_->last_error = "接收超时";
        return false;
    }

    excavator::ExcavatorState ref{};
    excavator::ExcavatorState resp{};
    if (!impl_->runtime->client().state()->getRef(ref) || !impl_->runtime->client().state()->getResp(resp)) {
        impl_->last_error = "状态读取失败";
        return false;
    }
    (void)fillRefState(ref, out.ref);
    (void)fillRespState(resp, out.resp);
    out.meta.loop_tick = tick;
    out.meta.recv_time_ns = nowNs();
    impl_->last_tick = tick;
    impl_->last_error.clear();
    return true;
}

std::string ExcavatorReceive::lastError() const {
    if (!impl_->last_error.empty()) return impl_->last_error;
    if (!impl_->runtime) return "runtime 未连接";
    return impl_->runtime->lastError();
}

}  // namespace excavator_api
