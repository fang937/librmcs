#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <thread>

#include <librmcs/client/cboard.hpp>
#include <librmcs/device/dr16.hpp>
#include <librmcs/device/servo_protocol.hpp>

class ServoExample final : public librmcs::client::CBoard {
public:
    ServoExample()
        : CBoard{}
        , transmit_buffer_{*this, 8} {}

    void update_remote() {
        dr16_.update_status();
        const auto current = dr16_.switch_left();
        if (current != last_left_switch_) {
            LOG_INFO("Left switch changed: %u", static_cast<unsigned>(current));
        }
        if (current == librmcs::device::Dr16::Switch::MIDDLE
            && last_left_switch_ != librmcs::device::Dr16::Switch::MIDDLE)
            set_angle(configured_id_, configured_angle_);
        last_left_switch_ = current;
    }

    void set_angle(std::uint8_t id, double angle) {
        if (id == 0 || id > 7 || !std::isfinite(angle)) {
            LOG_WARN("Servo command rejected: id=%u angle=%f", id, angle);
            return;
        }

        const auto bounded_angle = std::clamp(angle, 0.0, 180.0);
        const auto sequence = sequence_;
        std::byte packet[librmcs::device::Servo::command_packet_size];
        librmcs::device::Servo::Config config{id};
        librmcs::device::Servo servo{config};
        const auto packet_size = servo.generate_set_angle_command(bounded_angle, sequence, packet);
        LOG_INFO(
            "Servo frame check: USB=81 field=7d len=%zu data=%02x %02x %02x %02x %02x %02x %02x",
            packet_size, std::to_integer<unsigned>(packet[0]),
            std::to_integer<unsigned>(packet[1]), std::to_integer<unsigned>(packet[2]),
            std::to_integer<unsigned>(packet[3]), std::to_integer<unsigned>(packet[4]),
            std::to_integer<unsigned>(packet[5]), std::to_integer<unsigned>(packet[6]));
        if (!transmit_buffer_.add_servo_transmission(packet, packet_size)) {
            LOG_ERROR("Servo command dropped: id=%u angle=%.1f", id, bounded_angle);
            return;
        }

        ++sequence_;
        pending_id_ = id;
        pending_sequence_ = sequence;
        if (transmit_buffer_.trigger_transmission()) {
            LOG_INFO(
                "Servo command sent: id=%u angle=%.1f sequence=%u",
                id, bounded_angle, sequence);
        } else {
            LOG_WARN(
                "Servo command queued but USB frame was not submitted: id=%u sequence=%u",
                id, sequence);
        }
    }

private:
    void servo_receive_callback(const std::byte* data, std::uint8_t length) override {
        if (!data || length != librmcs::device::Servo::response_packet_size) {
            LOG_WARN("Servo response rejected: length=%u expected=%zu", length,
                librmcs::device::Servo::response_packet_size);
            return;
        }
        const auto id = static_cast<std::uint8_t>(data[5]);
        if (id < 1 || id > 7) {
            LOG_WARN("Servo response rejected: invalid id=%u", id);
            return;
        }
        librmcs::device::Servo::Config config{id};
        librmcs::device::Servo servo{config};
        if (!servo.store_status(data, length)) {
            LOG_WARN(
                "Servo response rejected: id=%u raw=%02x %02x %02x %02x %02x %02x %02x %02x",
                id, std::to_integer<unsigned>(data[0]), std::to_integer<unsigned>(data[1]),
                std::to_integer<unsigned>(data[2]), std::to_integer<unsigned>(data[3]),
                std::to_integer<unsigned>(data[4]), std::to_integer<unsigned>(data[5]),
                std::to_integer<unsigned>(data[6]), std::to_integer<unsigned>(data[7]));
            return;
        }
        if (id != pending_id_ || servo.response_sequence() != pending_sequence_
            || !servo.response_matches(pending_sequence_,
                librmcs::device::Servo::Command::SET_ANGLE)) {
            LOG_WARN(
                "Servo response ignored: id=%u sequence=%u expected_id=%u expected_sequence=%u",
                id, servo.response_sequence(), pending_id_, pending_sequence_);
            return;
        }
        pending_id_ = 0;
        LOG_INFO("Servo response received: id=%u command=0x%02x sequence=%u status=0x%02x",
            id, servo.response_command(), servo.response_sequence(), servo.response_status());
    }

    TransmitBuffer transmit_buffer_;
    librmcs::device::Dr16 dr16_;
    librmcs::device::Dr16::Switch last_left_switch_ = librmcs::device::Dr16::Switch::UNKNOWN;
    static constexpr std::uint8_t configured_id_ = 1;
    static constexpr double configured_angle_ = 45.0;
    std::uint8_t sequence_ = 0;
    std::uint8_t pending_id_ = 0;
    std::uint8_t pending_sequence_ = 0;
};

int main() {
    static std::atomic running{true};
    std::signal(SIGINT, [](int) { running = false; });
    ServoExample board;
    std::thread event_thread{[&board]() { board.handle_events(); }};
    while (running.load()) {
        board.update_remote();
        std::this_thread::sleep_for(std::chrono::milliseconds{5});
    }
    board.stop_handling_events();
    event_thread.join();
}
