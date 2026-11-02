#pragma once

#include "protocol.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>

namespace rh56 {

class Transport {
public:
    virtual ~Transport() = default;
    virtual bool ReadPosition(Position& position, uint8_t id = 1) = 0;
    virtual bool ReadWords(uint16_t address, RawValues& values,
                           uint8_t id = 1) = 0;
    virtual bool ReadBytes(uint16_t address, ByteValues& values,
                           uint8_t id = 1) = 0;
    virtual bool WritePosition(const Position& position, const JointMask& mask,
                               uint8_t id = 1) = 0;
    virtual bool WriteWords(uint16_t address, const RawValues& values,
                            int16_t maximum, uint8_t id = 1) = 0;
    virtual bool ClearErrors(uint8_t id = 1) = 0;
};

class SerialTransport final : public Transport
{
public:
    explicit SerialTransport(const std::string& device)
        : fd_(open(device.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC))
    {
        if (fd_ < 0)
            throw std::runtime_error("Cannot open " + device + ": " +
                                     std::strerror(errno));
        if (flock(fd_, LOCK_EX | LOCK_NB) != 0) {
            const std::string error = std::strerror(errno);
            close(fd_);
            throw std::runtime_error("Serial device is already in use: " +
                                     device + ": " + error);
        }

        termios options{};
        if (tcgetattr(fd_, &options) != 0) {
            const std::string error = std::strerror(errno);
            close(fd_);
            throw std::runtime_error("tcgetattr failed: " + error);
        }
        cfmakeraw(&options);
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        options.c_cflag |= CLOCAL | CREAD;
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 0;
        if (tcsetattr(fd_, TCSANOW, &options) != 0) {
            const std::string error = std::strerror(errno);
            close(fd_);
            throw std::runtime_error("tcsetattr failed: " + error);
        }
    }

    ~SerialTransport() override { close(fd_); }

    SerialTransport(const SerialTransport&) = delete;
    SerialTransport& operator=(const SerialTransport&) = delete;

    bool ReadPosition(Position& position, uint8_t id = 1) override
    {
        const auto request = MakeReadPosition(id);
        ReadResponse response{};
        return Exchange(request, response, 50) &&
               ParsePosition(response, id, position);
    }

    bool ReadWords(uint16_t address, RawValues& values,
                   uint8_t id = 1) override
    {
        const auto request = MakeRead(id, address, 0x0C);
        ReadResponse response{};
        return Exchange(request, response, 50) &&
               ParseWords(response, id, address, values);
    }

    bool ReadBytes(uint16_t address, ByteValues& values,
                   uint8_t id = 1) override
    {
        const auto request = MakeRead(id, address, 0x06);
        ByteReadResponse response{};
        return Exchange(request, response, 50) &&
               ParseBytes(response, id, address, values);
    }

    bool WritePosition(const Position& position, const JointMask& mask,
                       uint8_t id = 1) override
    {
        return WriteAndAcknowledge(MakeWritePosition(id, position, mask),
                                   0x05CE, id);
    }

    bool WriteWords(uint16_t address, const RawValues& values, int16_t maximum,
                    uint8_t id = 1) override
    {
        if (std::any_of(values.begin(), values.end(), [maximum](int16_t value) {
                return value < 0 || value > maximum;
            }))
            return false;
        return WriteAndAcknowledge(MakeWriteWords(id, address, values),
                                   address, id);
    }

    bool ClearErrors(uint8_t id = 1) override
    {
        constexpr uint16_t address = 0x03EC;
        return WriteAndAcknowledge(MakeWriteByte(id, address, uint8_t{1}),
                                   address, id);
    }

private:
    template <std::size_t N, std::size_t M>
    bool Exchange(const std::array<uint8_t, N>& request,
                  std::array<uint8_t, M>& response, int timeout_ms)
    {
        tcflush(fd_, TCIFLUSH);
        if (!WriteExact(request) || tcdrain(fd_) != 0)
            return false;
        return ReadExact(response, timeout_ms);
    }

    template <std::size_t N>
    bool WriteAndAcknowledge(const std::array<uint8_t, N>& request,
                             uint16_t address, uint8_t id)
    {
        std::array<uint8_t, 9> response{};
        if (!Exchange(request, response, 100))
            return false;
        return response[0] == 0x90 && response[1] == 0xEB &&
               response[2] == id && response[4] == 0x12 &&
               response[5] == static_cast<uint8_t>(address & 0xFF) &&
               response[6] == static_cast<uint8_t>(address >> 8) &&
               response.back() == Checksum(response.data(), response.size());
    }

    template <std::size_t N>
    bool WriteExact(const std::array<uint8_t, N>& request)
    {
        std::size_t sent = 0;
        while (sent < request.size()) {
            const ssize_t count =
                write(fd_, request.data() + sent, request.size() - sent);
            if (count > 0)
                sent += static_cast<std::size_t>(count);
            else if (count == 0)
                return false;
            else if (count < 0 && errno != EINTR)
                return false;
        }
        return true;
    }

    template <std::size_t N>
    bool ReadExact(std::array<uint8_t, N>& response, int timeout_ms)
    {
        std::size_t received = 0;
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(timeout_ms);
        while (received < response.size()) {
            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now());
            if (remaining.count() <= 0)
                return false;
            pollfd pfd{fd_, POLLIN, 0};
            if (poll(&pfd, 1, static_cast<int>(remaining.count())) <= 0)
                return false;
            const ssize_t count = read(fd_, response.data() + received,
                                       response.size() - received);
            if (count > 0)
                received += static_cast<std::size_t>(count);
            else if (count < 0 && errno != EINTR)
                return false;
        }
        return true;
    }

    int fd_;
};

}  // namespace rh56
