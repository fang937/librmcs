#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace librmcs::device {

class Servo {
public:
    static constexpr std::uint8_t command_header = 0xa5;
    static constexpr std::uint8_t response_header = 0x5a;
    static constexpr std::uint8_t field_id = 0x0d;
    static constexpr std::uint8_t response_field_header = 0x8d;

    enum class Command : std::uint8_t {
        SET_ANGLE = 0x01,
        ACTION = 0x02,
        QUERY_STATUS = 0x03,
    };

    enum class Action : std::uint8_t {
        STOP = 0x00,
        FIRE = 0x01,
        RELOAD = 0x02,
    };

    struct Config {
        explicit Config(std::uint8_t id) : id{id} {}
        Config& set_max_angle(double value) { return max_angle = value, *this; }
        Config& set_reversed() { return reversed = true, *this; }
        std::uint8_t id;
        double max_angle = 180.0;
        bool reversed = false;
    };

    static constexpr std::size_t set_angle_packet_size = 7;
    static constexpr std::size_t query_all_packet_size = 5;
    static constexpr std::size_t query_servo_packet_size = 6;
    static constexpr std::size_t action_packet_size = 6;
    static constexpr std::size_t command_packet_size = set_angle_packet_size;
    static constexpr std::size_t response_packet_size = 8;

    Servo() = default;
    explicit Servo(const Config& config) { configure(config); }

    void configure(const Config& config) { config_ = config; angle_ = 0.0; response_ = {}; }

    std::size_t generate_command(double angle, std::uint8_t sequence, std::byte* output) const {
        const auto bounded = std::clamp(angle, 0.0, config_.max_angle);
        const auto normalized = config_.reversed ? config_.max_angle - bounded : bounded;
        const auto value = static_cast<std::uint8_t>(normalized * 180.0 / config_.max_angle + 0.5);
        output[0] = std::byte{command_header};
        output[1] = std::byte{static_cast<std::uint8_t>(Command::SET_ANGLE)};
        output[2] = static_cast<std::byte>(sequence);
        output[3] = std::byte{2};
        output[4] = static_cast<std::byte>(config_.id);
        output[5] = static_cast<std::byte>(value);
        output[6] = static_cast<std::byte>(crc(output + 1, 5));
        return set_angle_packet_size;
    }

    std::size_t generate_set_angle_command(
        double angle, std::uint8_t sequence, std::byte* output) const {
        return generate_command(angle, sequence, output);
    }

    static std::size_t generate_query_command(
        std::uint8_t sequence, std::byte* output, std::uint8_t servo_id = 0) {
        const auto data_length = servo_id == 0 ? 0 : 1;
        output[0] = std::byte{command_header};
        output[1] = std::byte{static_cast<std::uint8_t>(Command::QUERY_STATUS)};
        output[2] = static_cast<std::byte>(sequence);
        output[3] = static_cast<std::byte>(data_length);
        if (data_length)
            output[4] = static_cast<std::byte>(servo_id);
        output[4 + data_length] = static_cast<std::byte>(crc(output + 1, 3 + data_length));
        return 5 + data_length;
    }

    static std::size_t generate_query_all_command(std::uint8_t sequence, std::byte* output) {
        return generate_query_command(sequence, output);
    }

    static std::size_t generate_action_command(
        Action action, std::uint8_t sequence, std::byte* output) {
        output[0] = std::byte{command_header};
        output[1] = std::byte{static_cast<std::uint8_t>(Command::ACTION)};
        output[2] = static_cast<std::byte>(sequence);
        output[3] = std::byte{1};
        output[4] = static_cast<std::byte>(action);
        output[5] = static_cast<std::byte>(crc(output + 1, 4));
        return action_packet_size;
    }

    bool store_status(const std::byte* input, std::uint8_t length) {
        if (!input || length != response_packet_size || !validate_response(input, length))
            return false;
        const auto* packet = reinterpret_cast<const std::uint8_t*>(input);
        if (packet[5] != config_.id) return false;
        response_.command = packet[1]; response_.sequence = packet[2];
        response_.status = packet[4]; response_.id = packet[5]; angle_ = packet[6];
        return true;
    }

    static bool validate_response(const std::byte* input, std::uint8_t length) {
        if (!input || length != response_packet_size
            || static_cast<std::uint8_t>(input[0]) != response_header)
            return false;
        const auto* packet = reinterpret_cast<const std::uint8_t*>(input);
        if (packet[3] != 0x03 || packet[1] < 0x81 || packet[1] > 0x83)
            return false;
        return packet[7] == crc(input + 1, 6);
    }

    bool response_matches(std::uint8_t sequence, Command command) const {
        return response_.sequence == sequence
            && response_.command == (static_cast<std::uint8_t>(command) | 0x80);
    }

    std::uint8_t id() const { return config_.id; }
    double angle() const { return angle_; }
    std::uint8_t response_sequence() const { return response_.sequence; }
    std::uint8_t response_command() const { return response_.command; }
    std::uint8_t response_status() const { return response_.status; }

private:
    static std::uint8_t crc(const std::byte* data, std::size_t length) {
        std::uint8_t result = 0;
        for (std::size_t index = 0; index < length; ++index)
            result ^= static_cast<std::uint8_t>(data[index]);
        return result;
    }

    struct Response {
        std::uint8_t command{};
        std::uint8_t sequence{};
        std::uint8_t status{};
        std::uint8_t id{};
    };
    Config config_{1};
    Response response_;
    double angle_ = 0.0;
};

} // namespace librmcs::device
