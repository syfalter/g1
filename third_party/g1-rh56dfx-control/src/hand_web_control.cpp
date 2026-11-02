#include "rh56/hand_client.hpp"

#include <unitree/robot/channel/channel_factory.hpp>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <net/if.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace {

constexpr std::size_t kMaximumRequestBytes = 64 * 1024;
constexpr int16_t kMaximumGripCurrentMa = 300;
constexpr uint32_t kMaximumPoseDelayMs = 3000;
using JointDelays = std::array<uint32_t, rh56::kJointCount>;

volatile std::sig_atomic_t stop_requested = 0;
int server_fd = -1;

void StopHandler(int)
{
    stop_requested = 1;
    if (server_fd >= 0)
        close(server_fd);
}

struct AppConfig
{
    std::string bind_address{"0.0.0.0"};
    std::string network;
    uint16_t port{8080};
    std::string hand{"both"};
    std::filesystem::path assets;
    std::string session_token;
    std::shared_ptr<rh56::HandClient> hand_client;
};

void PrintAccessUrls(const AppConfig& config)
{
    if (config.bind_address != "0.0.0.0") {
        std::cout << "Hand control UI: http://" << config.bind_address << ':'
                  << config.port << '\n';
        return;
    }

    std::cout << "Hand control UI (LAN):\n";
    ifaddrs* interfaces = nullptr;
    if (getifaddrs(&interfaces) != 0) {
        std::cout << "  http://<this-device-ip>:" << config.port << '\n';
        return;
    }
    bool printed = false;
    for (ifaddrs* item = interfaces; item; item = item->ifa_next) {
        if (!item->ifa_addr || item->ifa_addr->sa_family != AF_INET ||
            !(item->ifa_flags & IFF_UP) || (item->ifa_flags & IFF_LOOPBACK))
            continue;
        const std::string interface_name = item->ifa_name;
        if (interface_name.rfind("docker", 0) == 0 ||
            interface_name.rfind("veth", 0) == 0 ||
            interface_name.rfind("br-", 0) == 0 ||
            interface_name.rfind("virbr", 0) == 0)
            continue;
        char address[INET_ADDRSTRLEN]{};
        const auto* ipv4 = reinterpret_cast<sockaddr_in*>(item->ifa_addr);
        if (inet_ntop(AF_INET, &ipv4->sin_addr, address, sizeof(address))) {
            std::cout << "  http://" << address << ':' << config.port
                      << "  (" << item->ifa_name << ")\n";
            printed = true;
        }
    }
    freeifaddrs(interfaces);
    if (!printed)
        std::cout << "  http://<this-device-ip>:" << config.port << '\n';
}

std::string RandomToken()
{
    std::random_device random;
    std::ostringstream token;
    token << std::hex << std::setfill('0');
    for (int i = 0; i < 4; ++i)
        token << std::setw(8) << random();
    return token.str();
}

std::filesystem::path ProjectRoot(const char* executable)
{
    return std::filesystem::weakly_canonical(executable)
        .parent_path().parent_path().parent_path();
}

struct HttpRequest
{
    std::string method;
    std::string path;
    std::map<std::string, std::string> headers;
    std::string body;
};

bool SendAll(int fd, const std::string& data)
{
    std::size_t sent = 0;
    while (sent < data.size()) {
        const ssize_t count = send(fd, data.data() + sent, data.size() - sent,
                                   MSG_NOSIGNAL);
        if (count <= 0) {
            if (count < 0 && errno == EINTR)
                continue;
            return false;
        }
        sent += static_cast<std::size_t>(count);
    }
    return true;
}

void Respond(int fd, int status, const std::string& reason,
             const std::string& content_type, const std::string& body)
{
    std::ostringstream response;
    response << "HTTP/1.1 " << status << ' ' << reason << "\r\n"
             << "Content-Type: " << content_type << "\r\n"
             << "Content-Length: " << body.size() << "\r\n"
             << "Cache-Control: no-store\r\n"
             << "Connection: close\r\n\r\n"
             << body;
    SendAll(fd, response.str());
}

std::optional<HttpRequest> ReadRequest(int fd)
{
    std::string raw;
    std::array<char, 4096> buffer{};
    std::size_t header_end = std::string::npos;
    std::size_t content_length = 0;

    while (raw.size() < kMaximumRequestBytes) {
        const ssize_t count = recv(fd, buffer.data(), buffer.size(), 0);
        if (count <= 0)
            return std::nullopt;
        raw.append(buffer.data(), static_cast<std::size_t>(count));
        header_end = raw.find("\r\n\r\n");
        if (header_end == std::string::npos)
            continue;

        const std::string headers = raw.substr(0, header_end);
        const std::string key = "Content-Length:";
        const auto position = headers.find(key);
        if (position != std::string::npos) {
            const auto start = headers.find_first_not_of(" \t", position + key.size());
            try {
                content_length = std::stoul(headers.substr(start));
            } catch (...) {
                return std::nullopt;
            }
        }
        if (content_length > kMaximumRequestBytes ||
            raw.size() < header_end + 4 + content_length)
            continue;
        break;
    }
    if (header_end == std::string::npos ||
        raw.size() < header_end + 4 + content_length)
        return std::nullopt;

    HttpRequest request;
    std::istringstream stream(raw.substr(0, header_end));
    std::string line;
    if (!std::getline(stream, line))
        return std::nullopt;
    if (!line.empty() && line.back() == '\r')
        line.pop_back();
    std::istringstream request_line(line);
    std::string version;
    if (!(request_line >> request.method >> request.path >> version))
        return std::nullopt;
    request.path = request.path.substr(0, request.path.find('?'));

    while (std::getline(stream, line)) {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        const auto colon = line.find(':');
        if (colon != std::string::npos)
            request.headers[line.substr(0, colon)] = line.substr(colon + 1);
    }
    request.body = raw.substr(header_end + 4, content_length);
    return request;
}

std::string UrlDecode(const std::string& value)
{
    std::string decoded;
    decoded.reserve(value.size());
    for (std::size_t i = 0; i < value.size(); ++i) {
        if (value[i] == '+') {
            decoded.push_back(' ');
        } else if (value[i] == '%' && i + 2 < value.size()) {
            char* end = nullptr;
            const std::string hex = value.substr(i + 1, 2);
            const long byte = std::strtol(hex.c_str(), &end, 16);
            if (end == hex.c_str() + 2) {
                decoded.push_back(static_cast<char>(byte));
                i += 2;
            } else {
                decoded.push_back(value[i]);
            }
        } else {
            decoded.push_back(value[i]);
        }
    }
    return decoded;
}

std::map<std::string, std::string> ParseForm(const std::string& body)
{
    std::map<std::string, std::string> fields;
    std::size_t start = 0;
    while (start <= body.size()) {
        const auto end = body.find('&', start);
        const std::string pair = body.substr(start, end - start);
        const auto equals = pair.find('=');
        if (equals != std::string::npos)
            fields[UrlDecode(pair.substr(0, equals))] =
                UrlDecode(pair.substr(equals + 1));
        if (end == std::string::npos)
            break;
        start = end + 1;
    }
    return fields;
}

std::string JsonEscape(const std::string& value)
{
    std::ostringstream out;
    for (const unsigned char byte : value) {
        switch (byte) {
        case '"': out << "\\\""; break;
        case '\\': out << "\\\\"; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default:
            if (byte < 0x20)
                out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                    << static_cast<int>(byte);
            else
                out << byte;
        }
    }
    return out.str();
}

bool HandEnabled(const AppConfig& config, const std::string& hand)
{
    return hand == config.hand || config.hand == "both";
}

std::string ReadAsset(const std::filesystem::path& path)
{
    std::ifstream input(path, std::ios::binary);
    if (!input)
        throw std::runtime_error("Cannot read asset " + path.string());
    return {std::istreambuf_iterator<char>(input),
            std::istreambuf_iterator<char>()};
}

void ServeStatus(int fd, const AppConfig& config)
{
    std::ostringstream json;
    json << "{\"selection\":\"" << config.hand << "\",\"token\":\""
         << config.session_token << "\",\"states\":{";
    bool first = true;
    std::vector<std::string> errors;

    for (const std::string hand : {"right", "left"}) {
        if (!HandEnabled(config, hand))
            continue;
        rh56::StateReply state;
        const int32_t result = config.hand_client->GetState(hand, state);
        if (result == 0 && state.online) {
            if (!first)
                json << ',';
            first = false;
            json << '"' << hand << "\":"
                 << unitree::common::ToJsonString(state);
        } else {
            const std::string message = state.message.empty()
                                            ? "hand service unavailable"
                                            : state.message;
            errors.push_back(hand + ": " + message);
        }
    }
    json << "},\"errors\":[";
    for (std::size_t i = 0; i < errors.size(); ++i) {
        if (i)
            json << ',';
        json << '"' << JsonEscape(errors[i]) << '"';
    }
    json << "]}";
    Respond(fd, 200, "OK", "application/json; charset=utf-8", json.str());
}

template <typename Values>
std::string RegisterValuesJson(const Values& values)
{
    std::ostringstream out;
    out << '[';
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i)
            out << ',';
        out << +values[i];
    }
    return out.str() + ']';
}

void ServeRegisters(int fd, const AppConfig& config, const std::string& hand)
{
    if ((hand != "left" && hand != "right") || !HandEnabled(config, hand)) {
        Respond(fd, 400, "Bad Request", "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"Hand is not enabled\"}");
        return;
    }

    try {
        rh56::StateReply state;
        const int32_t result = config.hand_client->GetState(hand, state, true);
        if (result != 0 || !state.online)
            throw std::runtime_error(state.message.empty()
                                         ? "hand service unavailable"
                                         : state.message);
        std::vector<int32_t> position;
        position.reserve(state.feedback_q.size());
        std::transform(state.feedback_q.begin(), state.feedback_q.end(),
                       std::back_inserter(position), [](float q) {
                           return static_cast<int32_t>(std::lround(q * 1000.0f));
                       });

        std::ostringstream json;
        json << "{\"ok\":true,\"hand\":\"" << hand
             << "\",\"position\":" << RegisterValuesJson(position)
             << ",\"force\":" << RegisterValuesJson(state.force)
             << ",\"current\":" << RegisterValuesJson(state.current)
             << ",\"force_limit\":" << RegisterValuesJson(state.force_limit)
             << ",\"current_limit\":" << RegisterValuesJson(state.current_limit)
             << ",\"error\":" << RegisterValuesJson(state.error)
             << ",\"status\":" << RegisterValuesJson(state.status)
             << ",\"temperature\":" << RegisterValuesJson(state.temperature)
             << '}';
        Respond(fd, 200, "OK", "application/json; charset=utf-8", json.str());
    } catch (const std::exception& error) {
        const std::string body = "{\"ok\":false,\"error\":\"" +
                                 JsonEscape(error.what()) + "\"}";
        Respond(fd, 503, "Service Unavailable",
                "application/json; charset=utf-8", body);
    }
}

std::optional<rh56::Position> ParseValues(const std::string& text);

std::optional<int> ParseInteger(const std::string& text, int minimum,
                                int maximum)
{
    try {
        std::size_t used = 0;
        const int value = std::stoi(text, &used);
        if (used != text.size() || value < minimum || value > maximum)
            return std::nullopt;
        return value;
    } catch (...) {
        return std::nullopt;
    }
}

void ServeSettings(int fd, const AppConfig& config,
                   const HttpRequest* request = nullptr)
{
    rh56::SettingsMessage settings;
    if (request) {
        const auto fields = ParseForm(request->body);
        const auto token = fields.find("token");
        const auto value = fields.find("settings");
        if (token == fields.end() || token->second != config.session_token ||
            value == fields.end()) {
            Respond(fd, 400, "Bad Request", "application/json; charset=utf-8",
                    "{\"ok\":false,\"error\":\"Invalid settings request\"}");
            return;
        }
        try {
            unitree::common::FromJsonString(value->second, settings.settings);
            settings.write = rh56::ValidSettings(settings.settings);
        } catch (...) {
        }
        if (!settings.write) {
            Respond(fd, 400, "Bad Request", "application/json; charset=utf-8",
                    "{\"ok\":false,\"error\":\"Invalid settings values\"}");
            return;
        }
    }

    rh56::SettingsMessage reply;
    const int32_t result = request
                               ? config.hand_client->SetSettings(
                                     settings.settings, settings.request_id, reply)
                               : config.hand_client->GetSettings(reply);
    if (result != 0) {
        Respond(fd, 503, "Service Unavailable",
                "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"" +
                    JsonEscape(reply.message.empty()
                                   ? "hand service settings call failed"
                                   : reply.message) + "\"}");
        return;
    }
    Respond(fd, 200, "OK", "application/json; charset=utf-8",
            "{\"ok\":true,\"settings\":" +
                unitree::common::ToJsonString(reply.settings) + '}');
}

void ServeGrip(int fd, const AppConfig& config, const HttpRequest& request)
{
    const auto fields = ParseForm(request.body);
    const auto hand = fields.find("hand");
    const auto token = fields.find("token");
    const auto confirm = fields.find("confirm");
    const auto force = fields.find("force");
    const auto current = fields.find("current");
    const auto values = fields.find("values");
    if (hand == fields.end() || token == fields.end() ||
        confirm == fields.end() || force == fields.end() ||
        current == fields.end() || values == fields.end() ||
        token->second != config.session_token || confirm->second != "grip" ||
        (hand->second != "left" && hand->second != "right") ||
        !HandEnabled(config, hand->second)) {
        Respond(fd, 400, "Bad Request", "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"Invalid grip request\"}");
        return;
    }
    const auto force_grams = ParseInteger(force->second, 50, 1000);
    const auto current_ma =
        ParseInteger(current->second, 50, kMaximumGripCurrentMa);
    const auto targets = ParseValues(values->second);
    if (!force_grams || !current_ma || !targets) {
        Respond(fd, 400, "Bad Request", "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"Grip force, current, or targets are outside the safe range\"}");
        return;
    }

    try {
        rh56::GripRequest grip;
        grip.hand = hand->second;
        grip.q.assign(targets->begin(), targets->end());
        grip.force_grams = *force_grams;
        grip.current_ma = *current_ma;
        grip.command_id = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        grip.timeout_ms = 5000;
        rh56::OperationReply reply;
        const int32_t result = config.hand_client->ApplyGrip(grip, reply);
        if (result != 0)
            throw std::runtime_error(reply.message.empty()
                                         ? "hand service grip call failed"
                                         : reply.message);
        Respond(fd, 200, "OK", "application/json; charset=utf-8",
                "{\"ok\":true,\"force\":" + std::to_string(*force_grams) +
                    ",\"current\":" + std::to_string(*current_ma) + '}');
    } catch (const std::exception& error_message) {
        Respond(fd, 503, "Service Unavailable",
                "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"" +
                    JsonEscape(error_message.what()) + "\"}");
    }
}

void ServeClearFault(int fd, const AppConfig& config,
                     const HttpRequest& request)
{
    const auto fields = ParseForm(request.body);
    const auto hand = fields.find("hand");
    const auto joint = fields.find("joint");
    const auto token = fields.find("token");
    const auto confirm = fields.find("confirm");
    if (hand == fields.end() || joint == fields.end() ||
        token == fields.end() || confirm == fields.end() ||
        token->second != config.session_token || confirm->second != "clear-fault" ||
        (hand->second != "left" && hand->second != "right") ||
        !HandEnabled(config, hand->second)) {
        Respond(fd, 400, "Bad Request", "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"Invalid clear-fault request\"}");
        return;
    }
    const auto joint_index = ParseInteger(joint->second, 0, 5);
    if (!joint_index) {
        Respond(fd, 400, "Bad Request", "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"Invalid joint index\"}");
        return;
    }

    try {
        rh56::ClearFaultRequest clear;
        clear.hand = hand->second;
        clear.joint_mask = uint32_t{1} << *joint_index;
        clear.request_id = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        rh56::OperationReply reply;
        const int32_t result = config.hand_client->ClearFault(clear, reply);
        const bool cleared = result == 0 && reply.affected_mask != 0;
        const bool ok = result == 0;
        const std::string body =
            "{\"ok\":" + std::string(ok ? "true" : "false") +
            ",\"cleared\":" + std::string(cleared ? "true" : "false") +
            ",\"error\":" + RegisterValuesJson(reply.error) +
            ",\"status\":" + RegisterValuesJson(reply.status) +
            ",\"temperature\":" + RegisterValuesJson(reply.temperature) +
            ",\"message\":\"" + JsonEscape(reply.message) + "\"}";
        const bool conflict = result == static_cast<int32_t>(
            rh56::ResultCode::kFaultRemains);
        Respond(fd, ok ? 200 : (conflict ? 409 : 503),
                ok ? "OK" : (conflict ? "Conflict" : "Service Unavailable"),
                "application/json; charset=utf-8", body);
    } catch (const std::exception& error_message) {
        Respond(fd, 503, "Service Unavailable",
                "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"" +
                    JsonEscape(error_message.what()) + "\"}");
    }
}

std::optional<float> ParseQ(const std::string& text)
{
    try {
        std::size_t used = 0;
        const float value = std::stof(text, &used);
        if (used != text.size() || !std::isfinite(value) || value < 0.0f ||
            value > 1.0f)
            return std::nullopt;
        return value;
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<rh56::Position> ParseValues(const std::string& text)
{
    rh56::Position values{};
    std::istringstream input(text);
    std::string item;
    std::size_t count = 0;
    while (std::getline(input, item, ',')) {
        if (count >= values.size())
            return std::nullopt;
        const auto value = ParseQ(item);
        if (!value)
            return std::nullopt;
        values[count++] = *value;
    }
    if (count != values.size())
        return std::nullopt;
    return values;
}

std::optional<JointDelays> ParseDelays(const std::string& text)
{
    JointDelays delays{};
    std::istringstream input(text);
    std::string item;
    std::size_t count = 0;
    while (std::getline(input, item, ',')) {
        if (count >= delays.size())
            return std::nullopt;
        const auto delay = ParseInteger(item, 0, kMaximumPoseDelayMs);
        if (!delay)
            return std::nullopt;
        delays[count++] = static_cast<uint32_t>(*delay);
    }
    return count == delays.size() ? std::optional<JointDelays>{delays}
                                  : std::nullopt;
}

void ServePoseList(int fd, const AppConfig& config)
{
    rh56::PoseReply reply;
    const int32_t result = config.hand_client->ListPoses(reply);
    if (result != 0) {
        Respond(fd, 503, "Service Unavailable", "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"" + JsonEscape(reply.message) +
                    "\"}");
        return;
    }
    Respond(fd, 200, "OK", "application/json; charset=utf-8",
            "{\"ok\":true,\"poses\":" +
                unitree::common::ToJsonString(reply.poses) + '}');
}

bool ValidPoseId(const std::string& id)
{
    return !id.empty() && id.size() <= 80 &&
           std::all_of(id.begin(), id.end(), [](unsigned char byte) {
               return std::isalnum(byte) || byte == '-';
           });
}

void ServePoseSave(int fd, const AppConfig& config,
                   const HttpRequest& request)
{
    const auto fields = ParseForm(request.body);
    const auto token = fields.find("token");
    const auto name = fields.find("name");
    const auto right = fields.find("right");
    const auto left = fields.find("left");
    if (token == fields.end() || token->second != config.session_token ||
        name == fields.end() || name->second.empty() || name->second.size() > 96 ||
        right == fields.end() || left == fields.end()) {
        Respond(fd, 400, "Bad Request", "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"Invalid pose fields\"}");
        return;
    }
    const auto right_values = ParseValues(right->second);
    const auto left_values = ParseValues(left->second);
    if (!right_values || !left_values) {
        Respond(fd, 400, "Bad Request", "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"Invalid pose values\"}");
        return;
    }

    rh56::PoseRequest save;
    save.action = "save";
    const auto requested_id = fields.find("id");
    if (requested_id != fields.end())
        save.id = requested_id->second;
    save.name = name->second;
    save.right.assign(right_values->begin(), right_values->end());
    save.left.assign(left_values->begin(), left_values->end());
    save.delays_ms.assign(rh56::kJointCount, 0);
    save.request_id = std::chrono::duration_cast<std::chrono::microseconds>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();
    rh56::PoseReply reply;
    const int32_t result = config.hand_client->Poses(save, reply);
    if (result != 0) {
        Respond(fd, result == static_cast<int32_t>(rh56::ResultCode::kInvalidArgument)
                        ? 400 : 503,
                result == static_cast<int32_t>(rh56::ResultCode::kInvalidArgument)
                    ? "Bad Request" : "Service Unavailable",
                "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"" + JsonEscape(reply.message) +
                    "\"}");
        return;
    }
    Respond(fd, 200, "OK", "application/json; charset=utf-8",
            "{\"ok\":true,\"pose\":" +
                unitree::common::ToJsonString(reply.pose) + '}');
}

void ServePoseDelete(int fd, const AppConfig& config,
                     const HttpRequest& request)
{
    const auto fields = ParseForm(request.body);
    const auto token = fields.find("token");
    const auto id = fields.find("id");
    if (token == fields.end() || token->second != config.session_token ||
        id == fields.end() || !ValidPoseId(id->second)) {
        Respond(fd, 400, "Bad Request", "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"Invalid pose id\"}");
        return;
    }
    rh56::PoseRequest remove;
    remove.action = "delete";
    remove.id = id->second;
    remove.request_id = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
    rh56::PoseReply reply;
    const int32_t result = config.hand_client->Poses(remove, reply);
    if (result != 0) {
        const bool missing = result == static_cast<int32_t>(
            rh56::ResultCode::kPoseNotFound);
        Respond(fd, missing ? 404 : 503,
                missing ? "Not Found" : "Service Unavailable",
                "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"" + JsonEscape(reply.message) +
                    "\"}");
        return;
    }
    Respond(fd, 200, "OK", "application/json; charset=utf-8",
            "{\"ok\":true}");
}

void ServePoseRename(int fd, const AppConfig& config,
                     const HttpRequest& request)
{
    const auto fields = ParseForm(request.body);
    const auto token = fields.find("token");
    const auto id = fields.find("id");
    const auto name = fields.find("name");
    if (token == fields.end() || token->second != config.session_token ||
        id == fields.end() || !ValidPoseId(id->second) ||
        name == fields.end() || name->second.empty() || name->second.size() > 96) {
        Respond(fd, 400, "Bad Request", "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"Invalid pose name\"}");
        return;
    }
    rh56::PoseRequest rename;
    rename.action = "rename";
    rename.id = id->second;
    rename.name = name->second;
    rename.request_id = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
    rh56::PoseReply reply;
    const int32_t result = config.hand_client->Poses(rename, reply);
    if (result != 0) {
        const bool missing = result == static_cast<int32_t>(
            rh56::ResultCode::kPoseNotFound);
        Respond(fd, missing ? 404 : 503,
                missing ? "Not Found" : "Service Unavailable",
                "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"" + JsonEscape(reply.message) +
                    "\"}");
        return;
    }
    Respond(fd, 200, "OK", "application/json; charset=utf-8",
            "{\"ok\":true,\"pose\":" +
                unitree::common::ToJsonString(reply.pose) + '}');
}

void ServePoseDelays(int fd, const AppConfig& config,
                     const HttpRequest& request)
{
    const auto fields = ParseForm(request.body);
    const auto token = fields.find("token");
    const auto id = fields.find("id");
    const auto delays = fields.find("delays_ms");
    if (token == fields.end() || token->second != config.session_token ||
        id == fields.end() || !ValidPoseId(id->second) ||
        delays == fields.end()) {
        Respond(fd, 400, "Bad Request", "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"Invalid pose delay request\"}");
        return;
    }
    const auto values = ParseDelays(delays->second);
    if (!values) {
        Respond(fd, 400, "Bad Request", "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"Expected six delays in 0..3000 ms\"}");
        return;
    }

    rh56::PoseRequest update;
    update.action = "set_delays";
    update.id = id->second;
    update.delays_ms.assign(values->begin(), values->end());
    update.request_id = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
    rh56::PoseReply reply;
    const int32_t result = config.hand_client->Poses(update, reply);
    if (result != 0) {
        const bool missing = result == static_cast<int32_t>(
            rh56::ResultCode::kPoseNotFound);
        Respond(fd, missing ? 404 : 503,
                missing ? "Not Found" : "Service Unavailable",
                "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"" + JsonEscape(reply.message) +
                    "\"}");
        return;
    }
    Respond(fd, 200, "OK", "application/json; charset=utf-8",
            "{\"ok\":true,\"pose\":" +
                unitree::common::ToJsonString(reply.pose) + '}');
}

void ServePoseExecute(int fd, const AppConfig& config,
                      const HttpRequest& request)
{
    const auto fields = ParseForm(request.body);
    const auto token = fields.find("token");
    const auto id = fields.find("id");
    const auto confirm = fields.find("confirm");
    if (token == fields.end() || token->second != config.session_token ||
        id == fields.end() || !ValidPoseId(id->second) ||
        confirm == fields.end() || confirm->second != "execute-pose") {
        Respond(fd, 400, "Bad Request", "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"Invalid pose execution request\"}");
        return;
    }

    const uint64_t request_id =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    rh56::PoseReply reply;
    const int32_t result = config.hand_client->ExecutePose(
        id->second, config.hand, request_id, reply);
    if (result != 0) {
        const bool missing = result == static_cast<int32_t>(
            rh56::ResultCode::kPoseNotFound);
        const bool conflict = result == static_cast<int32_t>(
            rh56::ResultCode::kBusy);
        Respond(fd, missing ? 404 : (conflict ? 409 : 503),
                missing ? "Not Found" :
                          (conflict ? "Conflict" : "Service Unavailable"),
                "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"" + JsonEscape(reply.message) +
                    "\"}");
        return;
    }
    Respond(fd, 200, "OK", "application/json; charset=utf-8",
            "{\"ok\":true,\"id\":\"" + JsonEscape(reply.pose.id) +
                "\",\"duration_ms\":" +
                std::to_string(reply.duration_ms) + '}');
}

int32_t SendTargets(const AppConfig& config, const std::string& hand,
                    const rh56::Position& position, uint8_t joint_mask,
                    rh56::OperationReply& reply)
{
    rh56::SetTargetsRequest command;
    command.hand = hand;
    command.joint_mask = joint_mask;
    command.q.assign(position.begin(), position.end());
    command.command_id = std::chrono::duration_cast<std::chrono::microseconds>(
                             std::chrono::system_clock::now().time_since_epoch())
                             .count();
    command.timeout_ms = 5000;
    return config.hand_client->SetTargets(command, reply);
}

void ServeCommand(int fd, const AppConfig& config, const HttpRequest& request)
{
    const auto fields = ParseForm(request.body);
    const auto hand_it = fields.find("hand");
    const auto confirm_it = fields.find("confirm");
    const auto token_it = fields.find("token");
    if (hand_it == fields.end() || confirm_it == fields.end() ||
        token_it == fields.end() || confirm_it->second != "move" ||
        token_it->second != config.session_token ||
        (hand_it->second != "left" && hand_it->second != "right") ||
        !HandEnabled(config, hand_it->second)) {
        Respond(fd, 400, "Bad Request", "application/json; charset=utf-8",
                "{\"ok\":false,\"error\":\"Invalid hand or confirmation\"}");
        return;
    }

    rh56::Position position{};
    rh56::JointMask mask{};
    const auto values_it = fields.find("values");
    if (values_it != fields.end()) {
        const auto values = ParseValues(values_it->second);
        if (!values) {
            Respond(fd, 400, "Bad Request", "application/json; charset=utf-8",
                    "{\"ok\":false,\"error\":\"Expected six q values in [0,1]\"}");
            return;
        }
        position = *values;
        mask.fill(true);
    } else {
        const auto joint_it = fields.find("joint");
        const auto q_it = fields.find("q");
        if (joint_it == fields.end() || q_it == fields.end()) {
            Respond(fd, 400, "Bad Request", "application/json; charset=utf-8",
                    "{\"ok\":false,\"error\":\"Missing joint or q\"}");
            return;
        }
        try {
            std::size_t used = 0;
            const unsigned long joint = std::stoul(joint_it->second, &used);
            const auto q = ParseQ(q_it->second);
            if (used != joint_it->second.size() || joint >= position.size() || !q)
                throw std::invalid_argument("range");
            position[joint] = *q;
            mask[joint] = true;
        } catch (...) {
            Respond(fd, 400, "Bad Request", "application/json; charset=utf-8",
                    "{\"ok\":false,\"error\":\"Invalid joint or q\"}");
            return;
        }
    }

    try {
        rh56::OperationReply reply;
        const int32_t result = SendTargets(
            config, hand_it->second, position, rh56::EncodeMask(mask), reply);
        if (result != 0) {
            const bool busy = result == static_cast<int32_t>(
                rh56::ResultCode::kBusy);
            Respond(fd, busy ? 409 : 503,
                    busy ? "Conflict" : "Service Unavailable",
                    "application/json; charset=utf-8",
                    "{\"ok\":false,\"error\":\"" +
                        JsonEscape(reply.message.empty()
                                       ? "hand service command failed"
                                       : reply.message) + "\"}");
            return;
        }
        Respond(fd, 200, "OK", "application/json; charset=utf-8",
                "{\"ok\":true}");
    } catch (const std::exception& error) {
        const std::string body = "{\"ok\":false,\"error\":\"" +
                                 JsonEscape(error.what()) + "\"}";
        Respond(fd, 503, "Service Unavailable",
                "application/json; charset=utf-8", body);
    }
}

void HandleClient(int fd, const AppConfig& config)
{
    const auto request = ReadRequest(fd);
    if (!request) {
        Respond(fd, 400, "Bad Request", "text/plain; charset=utf-8",
                "Bad request\n");
        return;
    }
    if (request->method == "GET" && request->path == "/api/status") {
        ServeStatus(fd, config);
        return;
    }
    if (request->method == "GET" && request->path == "/api/settings") {
        ServeSettings(fd, config);
        return;
    }
    if (request->method == "POST" && request->path == "/api/settings") {
        ServeSettings(fd, config, &*request);
        return;
    }
    if (request->method == "GET" && request->path == "/api/poses") {
        ServePoseList(fd, config);
        return;
    }
    if (request->method == "POST" && request->path == "/api/poses/save") {
        ServePoseSave(fd, config, *request);
        return;
    }
    if (request->method == "POST" && request->path == "/api/poses/delete") {
        ServePoseDelete(fd, config, *request);
        return;
    }
    if (request->method == "POST" && request->path == "/api/poses/rename") {
        ServePoseRename(fd, config, *request);
        return;
    }
    if (request->method == "POST" && request->path == "/api/poses/delays") {
        ServePoseDelays(fd, config, *request);
        return;
    }
    if (request->method == "POST" && request->path == "/api/poses/execute") {
        ServePoseExecute(fd, config, *request);
        return;
    }
    if (request->method == "GET" &&
        request->path.rfind("/api/registers/", 0) == 0) {
        ServeRegisters(fd, config,
                       request->path.substr(std::string("/api/registers/").size()));
        return;
    }
    if (request->method == "POST" && request->path == "/api/command") {
        ServeCommand(fd, config, *request);
        return;
    }
    if (request->method == "POST" && request->path == "/api/grip") {
        ServeGrip(fd, config, *request);
        return;
    }
    if (request->method == "POST" && request->path == "/api/faults/clear") {
        ServeClearFault(fd, config, *request);
        return;
    }

    std::filesystem::path asset;
    std::string content_type;
    if (request->method == "GET" &&
        (request->path == "/" || request->path == "/index.html")) {
        asset = config.assets / "index.html";
        content_type = "text/html; charset=utf-8";
    } else if (request->method == "GET" && request->path == "/app.css") {
        asset = config.assets / "app.css";
        content_type = "text/css; charset=utf-8";
    } else if (request->method == "GET" && request->path == "/app.js") {
        asset = config.assets / "app.js";
        content_type = "text/javascript; charset=utf-8";
    } else {
        Respond(fd, 404, "Not Found", "text/plain; charset=utf-8",
                "Not found\n");
        return;
    }

    try {
        Respond(fd, 200, "OK", content_type, ReadAsset(asset));
    } catch (const std::exception& error) {
        Respond(fd, 500, "Internal Server Error", "text/plain; charset=utf-8",
                std::string(error.what()) + '\n');
    }
}

void PrintUsage(const char* executable)
{
    std::cerr << "Usage: " << executable
              << " [--hand left|right|both]"
                 " [--bind ADDRESS] [--port PORT] [--assets PATH]"
                 " [--network INTERFACE]\n";
}

std::optional<AppConfig> ParseArguments(int argc, char** argv)
{
    AppConfig config;
    const auto project_root = ProjectRoot(argv[0]);
    config.assets = project_root / "web" / "hand_control";
    config.session_token = RandomToken();
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if ((argument == "--hand" || argument == "--bind" ||
             argument == "--port" || argument == "--assets" ||
             argument == "--network") &&
            i + 1 < argc) {
            const std::string value = argv[++i];
            if (argument == "--hand")
                config.hand = value;
            else if (argument == "--bind")
                config.bind_address = value;
            else if (argument == "--assets")
                config.assets = value;
            else if (argument == "--network")
                config.network = value;
            else {
                try {
                    const unsigned long port = std::stoul(value);
                    if (port == 0 || port > 65535)
                        return std::nullopt;
                    config.port = static_cast<uint16_t>(port);
                } catch (...) {
                    return std::nullopt;
                }
            }
        } else {
            return std::nullopt;
        }
    }
    if (config.hand != "left" && config.hand != "right" &&
        config.hand != "both")
        return std::nullopt;
    return config;
}

int RunServer(const AppConfig& config)
{
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0)
        throw std::runtime_error("socket failed: " + std::string(std::strerror(errno)));
    const int reuse = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(config.port);
    if (inet_pton(AF_INET, config.bind_address.c_str(), &address.sin_addr) != 1)
        throw std::runtime_error("Invalid IPv4 bind address: " + config.bind_address);
    if (bind(server_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0)
        throw std::runtime_error("bind failed: " + std::string(std::strerror(errno)));
    if (listen(server_fd, 8) != 0)
        throw std::runtime_error("listen failed: " + std::string(std::strerror(errno)));

    PrintAccessUrls(config);
    std::cout << "Hand: " << config.hand << '\n'
              << "Keep the physical E-stop within immediate reach.\n";

    while (!stop_requested) {
        sockaddr_in peer{};
        socklen_t peer_size = sizeof(peer);
        const int client = accept(server_fd, reinterpret_cast<sockaddr*>(&peer),
                                  &peer_size);
        if (client < 0) {
            if (errno == EINTR || stop_requested)
                continue;
            throw std::runtime_error("accept failed: " +
                                     std::string(std::strerror(errno)));
        }
        const timeval receive_timeout{2, 0};
        setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &receive_timeout,
                   sizeof(receive_timeout));
        pollfd ready{client, POLLIN, 0};
        if (poll(&ready, 1, 500) > 0 && (ready.revents & POLLIN))
            HandleClient(client, config);
        close(client);
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv)
{
    auto config = ParseArguments(argc, argv);
    if (!config) {
        PrintUsage(argv[0]);
        return 2;
    }

    std::signal(SIGINT, StopHandler);
    std::signal(SIGTERM, StopHandler);
    try {
        unitree::robot::ChannelFactory::Instance()->Init(0, config->network);
        config->hand_client = std::make_shared<rh56::HandClient>();
        config->hand_client->SetTimeout(8.0f);
        config->hand_client->Init();
        for (const std::string hand : {"right", "left"}) {
            if (!HandEnabled(*config, hand))
                continue;
            rh56::StateReply state;
            const int32_t result = config->hand_client->GetState(hand, state);
            if (result != 0)
                throw std::runtime_error(
                    "Cannot connect to hand_service for " + hand +
                    " (code=" + std::to_string(result) + ")");
        }
        return RunServer(*config);
    } catch (const std::exception& error) {
        if (server_fd >= 0)
            close(server_fd);
        std::cerr << error.what() << '\n';
        return 1;
    }
}
