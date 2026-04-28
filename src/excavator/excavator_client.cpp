#include <excavator/internal/excavator_client.hpp>

#include <excavator/internal/excavator_control.hpp>

#include <chrono>

namespace excavator {

ExcavatorClient::ExcavatorClient(std::string can_shm_name, bool create_mapping, std::string imu_shm_name)
    : comm_(std::move(can_shm_name), create_mapping, std::move(imu_shm_name)), controller_(channel_) {}

ExcavatorClient::~ExcavatorClient() {
    (void)stop();
}

bool ExcavatorClient::init() {
    if (!comm_.open()) {
        return false;
    }
    ExcavatorState ref{};
    (void)channel_.setRef(ref);
    (void)channel_.getRef(ref);
    {
        std::lock_guard<std::mutex> lock(cached_status_mu_);
        cached_status_ = ref.status;
    }
    return true;
}

bool ExcavatorClient::start() {
    if (!comm_.isOpen() || worker_.joinable()) {
        return false;
    }
    loop_tick_.store(0, std::memory_order_release);
    running_.store(true, std::memory_order_release);
    worker_ = std::thread([this] { loop(); });
    return true;
}

bool ExcavatorClient::stop() {
    running_.store(false, std::memory_order_release);
    if (worker_.joinable()) {
        worker_.join();
    }
    return true;
}

bool ExcavatorClient::reset() {
    device_.fault_code = 0;
    {
        std::lock_guard<std::mutex> lock(ref_command_queue_mu_);
        ref_command_queue_.clear();
    }
    ExcavatorState ref{};
    (void)channel_.setRef(ref);
    {
        std::lock_guard<std::mutex> lock(cached_status_mu_);
        cached_status_.setZero();
    }
    loop_tick_.store(0, std::memory_order_release);
    return true;
}

void ExcavatorClient::submitServo(const ExcavatorCommand& cmd) {
    std::lock_guard<std::mutex> lock(ref_command_queue_mu_);
    ref_command_queue_.push_back(cmd);
}

bool ExcavatorClient::setPidVectors(const std::vector<std::vector<double>>& pid_vectors) {
    return controller_.setPidVectors(pid_vectors);
}

void ExcavatorClient::clearServo() {
    std::lock_guard<std::mutex> lock(ref_command_queue_mu_);
    ref_command_queue_.clear();
}

void ExcavatorClient::toggleStatusBit(int status_idx) {
    if (status_idx < 0 || status_idx > 10) return;
    std::lock_guard<std::mutex> lock(cached_status_mu_);
    if (status_idx == 9) {
        cached_status_(9) = (cached_status_(9) + 1) & 0x3;
        return;
    }
    cached_status_(status_idx) = (cached_status_(status_idx) == 0) ? 1 : 0;
}

void ExcavatorClient::toggleIgnition() { toggleStatusBit(0); }
void ExcavatorClient::toggleFlameout() { toggleStatusBit(1); }
void ExcavatorClient::toggleCrush() { toggleStatusBit(2); }
void ExcavatorClient::toggleChassisLight() { toggleStatusBit(3); }
void ExcavatorClient::toggleRemoteMode() { toggleStatusBit(4); }
void ExcavatorClient::togglePilot() { toggleStatusBit(5); }
void ExcavatorClient::toggleHighSpeed() { toggleStatusBit(6); }
void ExcavatorClient::toggleChassisDozerMode() { toggleStatusBit(7); }
void ExcavatorClient::toggleHorn() { toggleStatusBit(8); }
void ExcavatorClient::toggleMotorGear() { toggleStatusBit(9); }
void ExcavatorClient::toggleEstop() { toggleStatusBit(10); }

void ExcavatorClient::applyStatusToggleMask(std::uint16_t toggle_mask) {
    std::lock_guard<std::mutex> lock(cached_status_mu_);
    for (int i = 0; i <= 10; ++i) {
        if ((toggle_mask & (1U << i)) == 0U) {
            continue;
        }
        if (i == 9) {
            cached_status_(9) = (cached_status_(9) + 1) & 0x3;
        } else {
            cached_status_(i) = (cached_status_(i) == 0) ? 1 : 0;
        }
    }
}

void ExcavatorClient::applyCachedStatusToRef(ExcavatorState& ref) {
    std::lock_guard<std::mutex> lock(cached_status_mu_);
    ref.status = cached_status_;
    // 先导开启时，强制下发点火/熄火为 0，避免模式冲突。
    if (ref.status(5) != 0) {
        ref.status(0) = 0;
        ref.status(1) = 0;
    }
}

bool ExcavatorClient::takeServoCmd(ExcavatorCommand& out_cmd) {
    std::lock_guard<std::mutex> lock(ref_command_queue_mu_);
    if (ref_command_queue_.empty()) {
        return false;
    }
    out_cmd = ref_command_queue_.front();
    if (ref_command_queue_.size() > 1U) {
        ref_command_queue_.pop_front();
    }
    return true;
}

void ExcavatorClient::loop() {
    using clock = std::chrono::steady_clock;
    auto next_tick = clock::now();
    while (running_.load(std::memory_order_acquire)) {
        loop_tick_.fetch_add(1, std::memory_order_acq_rel);
        next_tick += std::chrono::milliseconds(20);
        if (!update()) {
            device_.fault_code = 1;
        }
        std::this_thread::sleep_until(next_tick);
    }
}

bool ExcavatorClient::update() {
    if (!comm_.isOpen()) {
        return false;
    }
    if (!comm_.read()) {
        return false;
    }

    ExcavatorState resp{};
    if (!converter_.hardwareStateToRobotState(comm_.hardwareState(), resp)) {
        return false;
    }
    ExcavatorState ref{};
    {
        auto access = channel_.unsafeAccessForWorker();
        access.resp() = resp;
        ref = access.ref();
    }
    ExcavatorCommand cmd_raw{};
    const ExcavatorCommand* cmd_ptr = nullptr;
    if (takeServoCmd(cmd_raw)) {
        cmd_ptr = &cmd_raw;
    }

    ExcavatorState ref_next{};
    (void)controller_.updateRef(ref, resp, cmd_ptr, ref_next);
    applyCachedStatusToRef(ref_next);
    {
        auto access = channel_.unsafeAccessForWorker();
        access.ref() = ref_next;
    }

    if (!converter_.robotStateToHardwareCmd(ref_next, comm_.mutableHardwareCommand())) {
        return false;
    }
    return comm_.write();
}

}  // namespace excavator
