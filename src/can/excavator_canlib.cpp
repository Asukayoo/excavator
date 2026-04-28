#include <can/internal/excavator_canlib.hpp>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdio>
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

inline constexpr std::size_t kCanPayloadSize = kExcavatorCanPayloadBytes;
static_assert(kCanPayloadSize == kExcavatorCanPayloadBytes, "payload size");
inline constexpr std::uint64_t kShmMagic = 0x42494C4E41435F32ULL;
inline constexpr int kStartupWarmupCycles = 10;
inline constexpr std::uint16_t kAxisMid = static_cast<std::uint16_t>((kAxisMin + kAxisMax) / 2U);

std::uint8_t clamp_bit(std::uint8_t v) { return static_cast<std::uint8_t>(std::min<std::uint8_t>(v, 1)); }

std::uint8_t clamp_2bit(std::uint8_t v) { return static_cast<std::uint8_t>(std::min<std::uint8_t>(v, 3)); }

std::string normalize_shm_name(const std::string& name) {
    if (name.empty()) {
        return "/canlib_shm";
    }
    if (name.front() == '/') {
        return name;
    }
    return "/" + name;
}

void pack_10hz(const Slave100msData& in, std::array<std::uint8_t, kCanPayloadSize>& out) {
    out.fill(0);
    // 18F021F6：仅点火时首字节0x01、次字节 0x00（01 00 …）
    out[0] = static_cast<std::uint8_t>((clamp_bit(in.ignition) << 0) | (clamp_bit(in.flameout) << 2) |
                                       (clamp_bit(in.crush) << 4) | (clamp_bit(in.chassis_light) << 6));
    out[1] = static_cast<std::uint8_t>((clamp_bit(in.remote_mode) << 0) | (clamp_bit(in.pilot) << 2) |
                                       (clamp_bit(in.high_speed) << 4) | (clamp_bit(in.chassis_dozer_mode) << 6));
    out[2] = static_cast<std::uint8_t>((clamp_bit(in.horn) << 0) | (clamp_2bit(in.motor_gear) << 2) |
                                       (clamp_bit(in.estop) << 4) | (clamp_bit(in.standby) << 6));
    out[3] = in.remote_heartbeat;
}

void unpack_10hz(const std::array<std::uint8_t, kCanPayloadSize>& in, Slave100msData& out) {
    out.ignition = static_cast<std::uint8_t>((in[0] >> 0) & 0x3U);
    out.flameout = static_cast<std::uint8_t>((in[0] >> 2) & 0x3U);
    out.crush = static_cast<std::uint8_t>((in[0] >> 4) & 0x3U);
    out.chassis_light = static_cast<std::uint8_t>((in[0] >> 6) & 0x3U);
    out.remote_mode = static_cast<std::uint8_t>((in[1] >> 0) & 0x3U);
    out.pilot = static_cast<std::uint8_t>((in[1] >> 2) & 0x3U);
    out.high_speed = static_cast<std::uint8_t>((in[1] >> 4) & 0x3U);
    out.chassis_dozer_mode = static_cast<std::uint8_t>((in[1] >> 6) & 0x3U);
    out.horn = static_cast<std::uint8_t>((in[2] >> 0) & 0x3U);
    out.motor_gear = static_cast<std::uint8_t>((in[2] >> 2) & 0x3U);
    out.estop = static_cast<std::uint8_t>((in[2] >> 4) & 0x3U);
    out.standby = static_cast<std::uint8_t>((in[2] >> 6) & 0x3U);
    out.remote_heartbeat = in[3];
}

void put_u16_le(std::array<std::uint8_t, kCanPayloadSize>& out, std::size_t idx, std::uint16_t v) {
    out[idx] = static_cast<std::uint8_t>(v & 0xFFU);
    out[idx + 1] = static_cast<std::uint8_t>((v >> 8) & 0xFFU);
}

std::uint16_t get_u16_le(const std::array<std::uint8_t, kCanPayloadSize>& in, std::size_t idx) {
    return static_cast<std::uint16_t>(static_cast<std::uint16_t>(in[idx]) |
                                      static_cast<std::uint16_t>(in[idx + 1] << 8));
}

void pack_50hz_a(const Slave50HzDataA& in, std::array<std::uint8_t, kCanPayloadSize>& out) {
    out.fill(0);
    put_u16_le(out, 0, in.swing);
    put_u16_le(out, 2, in.arm);
    put_u16_le(out, 4, in.boom);
    put_u16_le(out, 6, in.bucket);
}

void unpack_50hz_a(const std::array<std::uint8_t, kCanPayloadSize>& in, Slave50HzDataA& out) {
    out.swing = get_u16_le(in, 0);
    out.arm = get_u16_le(in, 2);
    out.boom = get_u16_le(in, 4);
    out.bucket = get_u16_le(in, 6);
}

void pack_50hz_b(const Slave50HzDataB& in, std::array<std::uint8_t, kCanPayloadSize>& out) {
    out.fill(0);
    put_u16_le(out, 0, in.left_track);
    put_u16_le(out, 2, in.right_track);
    put_u16_le(out, 4, in.boom_offset);
    put_u16_le(out, 6, in.chassis_dozer);
}

void unpack_50hz_b(const std::array<std::uint8_t, kCanPayloadSize>& in, Slave50HzDataB& out) {
    out.left_track = get_u16_le(in, 0);
    out.right_track = get_u16_le(in, 2);
    out.boom_offset = get_u16_le(in, 4);
    out.chassis_dozer = get_u16_le(in, 6);
}

bool axis_valid(std::uint16_t v) { return v >= kAxisMin && v <= kAxisMax; }

void fill_motor_rpm_placeholder(CanSharedMemoryLayout* layout) {
    if (!layout) return;
    for (std::size_t i = 0; i < kCanMotorFeedbackAxisCount; ++i) {
        layout->motor_rpm_feedback[i] = kCanMotorRpmPlaceholder;
    }
}

/** 模拟模式：轴反馈字与 50Hz 指令一致（替代 Client 侧写回） */
void sync_motor_rpm_feedback_from_50hz_cmd(const Slave50HzDataA& a, const Slave50HzDataB& b,
                                           CanSharedMemoryLayout* layout) {
    if (!layout) return;
    layout->motor_rpm_feedback[0] = static_cast<double>(a.swing);
    layout->motor_rpm_feedback[1] = static_cast<double>(a.arm);
    layout->motor_rpm_feedback[2] = static_cast<double>(a.boom);
    layout->motor_rpm_feedback[3] = static_cast<double>(a.bucket);
    layout->motor_rpm_feedback[4] = static_cast<double>(b.left_track);
    layout->motor_rpm_feedback[5] = static_cast<double>(b.right_track);
    layout->motor_rpm_feedback[6] = static_cast<double>(b.boom_offset);
    layout->motor_rpm_feedback[7] = static_cast<double>(b.chassis_dozer);
}

void ensure_valid_cmd_50hz_a(Slave50HzDataA& d) {
    if (!axis_valid(d.swing)) d.swing = kAxisMid;
    if (!axis_valid(d.arm)) d.arm = kAxisMid;
    if (!axis_valid(d.boom)) d.boom = kAxisMid;
    if (!axis_valid(d.bucket)) d.bucket = kAxisMid;
}

void ensure_valid_cmd_50hz_b(Slave50HzDataB& d) {
    if (!axis_valid(d.left_track)) d.left_track = kAxisMid;
    if (!axis_valid(d.right_track)) d.right_track = kAxisMid;
    if (!axis_valid(d.boom_offset)) d.boom_offset = kAxisMid;
    if (!axis_valid(d.chassis_dozer)) d.chassis_dozer = kAxisMid;
}

void ensure_valid_cmd_10hz(Slave100msData& d) {
    if (d.ignition > 1U) d.ignition = 0;
    if (d.flameout > 1U) d.flameout = 1;
    if (d.crush > 1U) d.crush = 0;
    if (d.chassis_light > 1U) d.chassis_light = 0;
    if (d.remote_mode > 1U) d.remote_mode = 1;
    if (d.pilot > 1U) d.pilot = 0;
    if (d.high_speed > 1U) d.high_speed = 0;
    if (d.chassis_dozer_mode > 1U) d.chassis_dozer_mode = 0;
    if (d.horn > 1U) d.horn = 0;
    if (d.estop > 1U) d.estop = 0;
    if (d.standby > 1U) d.standby = 0;
}

void builtin_parse_excavator_fb_10hz(ExcavatorFeedbackSource src,
                                     const std::array<std::uint8_t, kCanPayloadSize>& payload,
                                     const Slave100msData& cmd, Slave100msData& fb_out) {
    if (src == ExcavatorFeedbackSource::kCommandEcho) {
        fb_out = cmd;
        return;
    }
    unpack_10hz(payload, fb_out);
}

void builtin_parse_excavator_fb_50a(ExcavatorFeedbackSource src,
                                    const std::array<std::uint8_t, kCanPayloadSize>& payload,
                                    const Slave50HzDataA& cmd, Slave50HzDataA& fb_out) {
    if (src == ExcavatorFeedbackSource::kCommandEcho) {
        fb_out = cmd;
        return;
    }
    unpack_50hz_a(payload, fb_out);
}

void builtin_parse_excavator_fb_50b(ExcavatorFeedbackSource src,
                                    const std::array<std::uint8_t, kCanPayloadSize>& payload,
                                    const Slave50HzDataB& cmd, Slave50HzDataB& fb_out) {
    if (src == ExcavatorFeedbackSource::kCommandEcho) {
        fb_out = cmd;
        return;
    }
    unpack_50hz_b(payload, fb_out);
}

}  // namespace

void ExcavatorDefaultFeedbackParser::parse10Hz(ExcavatorFeedbackSource src,
                                               const std::array<std::uint8_t, kExcavatorCanPayloadBytes>& payload,
                                               const Slave100msData& cmd, Slave100msData& fb_out) {
    builtin_parse_excavator_fb_10hz(src, payload, cmd, fb_out);
}

void ExcavatorDefaultFeedbackParser::parse50HzA(ExcavatorFeedbackSource src,
                                                const std::array<std::uint8_t, kExcavatorCanPayloadBytes>& payload,
                                                const Slave50HzDataA& cmd, Slave50HzDataA& fb_out) {
    builtin_parse_excavator_fb_50a(src, payload, cmd, fb_out);
}

void ExcavatorDefaultFeedbackParser::parse50HzB(ExcavatorFeedbackSource src,
                                                const std::array<std::uint8_t, kExcavatorCanPayloadBytes>& payload,
                                                const Slave50HzDataB& cmd, Slave50HzDataB& fb_out) {
    builtin_parse_excavator_fb_50b(src, payload, cmd, fb_out);
}

struct CanLib::Impl {
    std::string can_if_name;
    std::string shm_name;
    bool create_mapping{false};
    bool opened{false};
    std::atomic<bool> running{false};
    std::thread worker;
    std::uint8_t heartbeat_counter{0};
    std::atomic<bool> simulation_enabled{true};
    std::atomic<bool> can_bus_enabled{true};
    std::atomic<std::uint64_t> loop_tick{0};
    std::string last_error;
    ExcavatorDefaultFeedbackParser default_feedback_parser_;
    std::unique_ptr<ExcavatorFeedbackParser> feedback_parser_;

    ExcavatorFeedbackParser* effective_parser() {
        return feedback_parser_ ? feedback_parser_.get() : &default_feedback_parser_;
    }

#if defined(__linux__)
    int can_fd{-1};
    int shm_fd{-1};
    CanSharedMemoryLayout* shm_view{nullptr};
#endif

    ~Impl() { close_impl(); }

    bool open_impl() {
#if defined(__linux__)
        last_error.clear();
        close_impl();
        if (!open_shm()) return false;
        // 模拟模式不依赖真实 CAN，行为与 ImuCanLib 一致。
        if (!simulation_enabled.load(std::memory_order_acquire) &&
            can_bus_enabled.load(std::memory_order_acquire)) {
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
            last_error = "shm_open 失败: " + std::string(std::strerror(errno));
            return false;
        }
        constexpr std::size_t kSize = sizeof(CanSharedMemoryLayout);
        if (create_mapping && ftruncate(shm_fd, static_cast<off_t>(kSize)) != 0) {
            last_error = "ftruncate 失败: " + std::string(std::strerror(errno));
            close_shm();
            return false;
        }
        void* mapped = mmap(nullptr, kSize, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
        if (mapped == MAP_FAILED) {
            last_error = "mmap 失败: " + std::string(std::strerror(errno));
            close_shm();
            return false;
        }
        shm_view = static_cast<CanSharedMemoryLayout*>(mapped);
        if (create_mapping) {
            std::memset(shm_view, 0, sizeof(CanSharedMemoryLayout));
            shm_view->magic = kShmMagic;
        } else if (shm_view->magic != kShmMagic) {
            last_error = "共享内存 magic 不匹配";
            close_shm();
            return false;
        }
        fill_motor_rpm_placeholder(shm_view);
        return true;
    }

    bool open_can() {
        can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_fd < 0) {
            last_error = "socket(PF_CAN) 失败: " + std::string(std::strerror(errno));
            return false;
        }
        can_filter filters[3]{};
        filters[0].can_id = kCanIdCmd10Hz;
        filters[0].can_mask = CAN_EFF_MASK;
        filters[1].can_id = kCanIdCmd50HzA;
        filters[1].can_mask = CAN_EFF_MASK;
        filters[2].can_id = kCanIdCmd50HzB;
        filters[2].can_mask = CAN_EFF_MASK;
        if (setsockopt(can_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filters, sizeof(filters)) != 0) {
            last_error = "setsockopt(CAN_RAW_FILTER) 失败: " + std::string(std::strerror(errno));
            close_can();
            return false;
        }
        const int recv_own = 1;
        (void)setsockopt(can_fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own, sizeof(recv_own));

        ifreq ifr{};
        std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", can_if_name.c_str());
        if (ioctl(can_fd, SIOCGIFINDEX, &ifr) < 0) {
            last_error = "ioctl(SIOCGIFINDEX) 失败: " + std::string(std::strerror(errno));
            close_can();
            return false;
        }

        sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(can_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            last_error = "bind(can) 失败: " + std::string(std::strerror(errno));
            close_can();
            return false;
        }

        const int fd_flags = fcntl(can_fd, F_GETFL, 0);
        if (fd_flags < 0 || fcntl(can_fd, F_SETFL, fd_flags | O_NONBLOCK) < 0) {
            last_error = "fcntl(O_NONBLOCK) 失败: " + std::string(std::strerror(errno));
            close_can();
            return false;
        }
        return true;
    }

    void close_shm() {
        if (shm_view) {
            munmap(shm_view, sizeof(CanSharedMemoryLayout));
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

    bool send_frame(std::uint32_t id, const std::array<std::uint8_t, kCanPayloadSize>& payload) {
        if (!can_bus_enabled.load(std::memory_order_acquire)) return true;
        if (can_fd < 0) return false;
        can_frame frame{};
        frame.can_id = id | CAN_EFF_FLAG;
        frame.can_dlc = static_cast<__u8>(kCanPayloadSize);
        std::memcpy(frame.data, payload.data(), kCanPayloadSize);
        const ssize_t n = write(can_fd, &frame, sizeof(frame));
        return n == static_cast<ssize_t>(sizeof(frame));
    }

    void write_feedback_by_id(std::uint32_t id, ExcavatorFeedbackSource src,
                              const std::array<std::uint8_t, kCanPayloadSize>& payload,
                              const Slave100msData& cmd_10hz, const Slave50HzDataA& cmd_50hz_a,
                              const Slave50HzDataB& cmd_50hz_b) {
        if (!shm_view) return;
        ExcavatorFeedbackParser* const p = effective_parser();
        switch (id) {
            case kCanIdCmd10Hz:
                p->parse10Hz(src, payload, cmd_10hz, shm_view->fb_10hz);
                break;
            case kCanIdCmd50HzA:
                p->parse50HzA(src, payload, cmd_50hz_a, shm_view->fb_50hz_a);
                break;
            case kCanIdCmd50HzB:
                p->parse50HzB(src, payload, cmd_50hz_b, shm_view->fb_50hz_b);
                break;
            default:
                return;
        }
        ++shm_view->fb_sequence;
    }

    void drain_feedback() {
        if (can_fd < 0) return;
        while (true) {
            can_frame frame{};
            const ssize_t n = recv(can_fd, &frame, sizeof(frame), MSG_DONTWAIT);
            if (n < 0) break;
            if (n != static_cast<ssize_t>(sizeof(frame)) || frame.can_dlc < kCanPayloadSize) continue;
            const std::uint32_t id = frame.can_id & CAN_EFF_MASK;
            std::array<std::uint8_t, kCanPayloadSize> payload{};
            std::memcpy(payload.data(), frame.data, kCanPayloadSize);
            write_feedback_by_id(id, ExcavatorFeedbackSource::kCanRx, payload, shm_view->cmd_10hz, shm_view->cmd_50hz_a,
                                 shm_view->cmd_50hz_b);
        }
    }

    void loop() {
        using clock = std::chrono::steady_clock;
        constexpr auto kBasePeriod = std::chrono::milliseconds(20);  // 50Hz
        auto next_tick = clock::now();
        std::uint8_t cnt_10hz = 0;
        auto run_one_cycle = [&](bool warmup_cycle) {
            Slave100msData cmd_10hz{};
            Slave50HzDataA cmd_50hz_a{};
            Slave50HzDataB cmd_50hz_b{};
            if (shm_view) {
                cmd_10hz = shm_view->cmd_10hz;
                cmd_50hz_a = shm_view->cmd_50hz_a;
                cmd_50hz_b = shm_view->cmd_50hz_b;
            }
            ensure_valid_cmd_10hz(cmd_10hz);
            ensure_valid_cmd_50hz_a(cmd_50hz_a);
            ensure_valid_cmd_50hz_b(cmd_50hz_b);
            if (shm_view) {
                // 勿写回 cmd_10hz：ExcavatorClient 可能在读本周期后写入，写回会覆盖点火/熄火脉冲
                shm_view->cmd_50hz_a = cmd_50hz_a;
                shm_view->cmd_50hz_b = cmd_50hz_b;
            }

            std::array<std::uint8_t, kCanPayloadSize> payload{};
            pack_50hz_a(cmd_50hz_a, payload);
            if (!warmup_cycle) (void)send_frame(kCanIdCmd50HzA, payload);
            if (warmup_cycle || simulation_enabled.load(std::memory_order_acquire)) {
                write_feedback_by_id(kCanIdCmd50HzA, ExcavatorFeedbackSource::kCommandEcho, payload, cmd_10hz,
                                     cmd_50hz_a, cmd_50hz_b);
            }
            pack_50hz_b(cmd_50hz_b, payload);
            if (!warmup_cycle) (void)send_frame(kCanIdCmd50HzB, payload);
            if (warmup_cycle || simulation_enabled.load(std::memory_order_acquire)) {
                write_feedback_by_id(kCanIdCmd50HzB, ExcavatorFeedbackSource::kCommandEcho, payload, cmd_10hz,
                                     cmd_50hz_a, cmd_50hz_b);
            }

            if (!warmup_cycle) ++cnt_10hz;
            if (!warmup_cycle && cnt_10hz >= 5) {
                cnt_10hz = 0;
                if (shm_view) {
                    cmd_10hz = shm_view->cmd_10hz;
                }
                ensure_valid_cmd_10hz(cmd_10hz);
                cmd_10hz.remote_heartbeat = heartbeat_counter;
                heartbeat_counter = static_cast<std::uint8_t>(heartbeat_counter + 1);
                if (shm_view) {
                    shm_view->cmd_10hz = cmd_10hz;
                    ++shm_view->cmd_sequence;
                }
                pack_10hz(cmd_10hz, payload);
                (void)send_frame(kCanIdCmd10Hz, payload);
                if (simulation_enabled.load(std::memory_order_acquire)) {
                    write_feedback_by_id(kCanIdCmd10Hz, ExcavatorFeedbackSource::kCommandEcho, payload, cmd_10hz,
                                         cmd_50hz_a, cmd_50hz_b);
                }
            }

            if (warmup_cycle) {
                pack_10hz(cmd_10hz, payload);
                write_feedback_by_id(kCanIdCmd10Hz, ExcavatorFeedbackSource::kCommandEcho, payload, cmd_10hz,
                                     cmd_50hz_a, cmd_50hz_b);
                if (shm_view) {
                    fill_motor_rpm_placeholder(shm_view);
                    if (simulation_enabled.load(std::memory_order_acquire)) {
                        sync_motor_rpm_feedback_from_50hz_cmd(cmd_50hz_a, cmd_50hz_b, shm_view);
                    }
                }
                return;
            }
            if (!simulation_enabled.load(std::memory_order_acquire)) drain_feedback();
            if (shm_view && simulation_enabled.load(std::memory_order_acquire)) {
                sync_motor_rpm_feedback_from_50hz_cmd(cmd_50hz_a, cmd_50hz_b, shm_view);
            }
        };

        for (int i = 0; i < kStartupWarmupCycles && running.load(std::memory_order_acquire); ++i) {
            loop_tick.fetch_add(1, std::memory_order_acq_rel);
            next_tick += kBasePeriod;
            // 前 10 拍仅预热，不下发 CAN 帧。
            run_one_cycle(true);
            std::this_thread::sleep_until(next_tick);
        }

        while (running.load(std::memory_order_acquire)) {
            loop_tick.fetch_add(1, std::memory_order_acq_rel);
            next_tick += kBasePeriod;
            run_one_cycle(false);
            std::this_thread::sleep_until(next_tick);
        }
    }
#endif
};

CanLib::CanLib(std::string can_if_name, std::string shm_name, bool create_mapping)
    : impl_(std::make_unique<Impl>()) {
    impl_->can_if_name = std::move(can_if_name);
    impl_->shm_name = std::move(shm_name);
    impl_->create_mapping = create_mapping;
}

CanLib::~CanLib() = default;

bool CanLib::open() {
    if (!impl_) return false;
    return impl_->open_impl();
}

bool CanLib::start() {
    if (!impl_) return false;
    return impl_->start_impl();
}

bool CanLib::stop() {
    if (!impl_) return false;
    return impl_->stop_impl();
}

bool CanLib::close() {
    if (!impl_) return false;
    impl_->close_impl();
    return true;
}

bool CanLib::isOpen() const {
    if (!impl_) return false;
    return impl_->is_open_impl();
}

void CanLib::setSimulationEnabled(bool enabled) {
    if (!impl_) return;
    impl_->simulation_enabled.store(enabled, std::memory_order_release);
}

bool CanLib::isSimulationEnabled() const {
    if (!impl_) return false;
    return impl_->simulation_enabled.load(std::memory_order_acquire);
}

void CanLib::setCanBusEnabled(bool enabled) {
    if (!impl_) return;
    impl_->can_bus_enabled.store(enabled, std::memory_order_release);
}

bool CanLib::isCanBusEnabled() const {
    if (!impl_) return false;
    return impl_->can_bus_enabled.load(std::memory_order_acquire);
}

std::uint64_t CanLib::loopTick() const {
    if (!impl_) return 0;
    return impl_->loop_tick.load(std::memory_order_acquire);
}

std::string CanLib::lastError() const {
    if (!impl_) return "canlib impl 为空";
    return impl_->last_error;
}

void CanLib::setFeedbackParser(std::unique_ptr<ExcavatorFeedbackParser> parser) {
    if (!impl_) return;
    impl_->feedback_parser_ = std::move(parser);
}

ExcavatorFeedbackParser* CanLib::feedbackParser() {
    if (!impl_) return nullptr;
    return impl_->effective_parser();
}

}  // namespace canlib
