#include "vision/vision_api.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/server/server.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <sys/socket.h>
#include <sys/file.h>
#include <fcntl.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

namespace {

std::atomic<bool> running{true};

void HandleSignal(int)
{
    running = false;
}

std::filesystem::path ProjectRoot(const std::filesystem::path& executable)
{
    return std::filesystem::weakly_canonical(executable)
        .parent_path().parent_path().parent_path();
}

struct Config {
    std::string network;
    std::string python{"/home/unitree/.miniconda3/envs/Unitree/bin/python"};
    std::filesystem::path worker;
    std::filesystem::path socket;
    std::filesystem::path captures_dir;
    std::filesystem::path stage1;
    std::filesystem::path stage2;
    std::filesystem::path target_extrinsics;
    std::string serial{"243322073672"};
    std::string device{"0"};
    int width{1280};
    int height{720};
    int fps{30};
    int warmup_frames{15};
    int depth_patch_radius{2};
    int worker_timeout_ms{60000};
};

int PositiveInteger(const std::string& value, const char* name)
{
    const int parsed = std::stoi(value);
    if (parsed <= 0)
        throw std::invalid_argument(std::string(name) + " must be positive");
    return parsed;
}

int NonnegativeInteger(const std::string& value, const char* name)
{
    const int parsed = std::stoi(value);
    if (parsed < 0)
        throw std::invalid_argument(std::string(name) + " cannot be negative");
    return parsed;
}

Config ParseArgs(int argc, char** argv)
{
    Config config;
    const auto root = ProjectRoot(argv[0]);
    config.worker = root / "recognize/pedal_center_package_v1/vision_worker.py";
    config.socket = root / "run/vision_service.sock";
    config.captures_dir = root / "captures";
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--help") {
            throw std::invalid_argument(
                "Usage: vision_service [--network INTERFACE] [--python PATH] "
                "[--serial SERIAL] [--device DEVICE] [--target-extrinsics FILE]");
        }
        if (index + 1 >= argc)
            throw std::invalid_argument("missing value for " + argument);
        const std::string value = argv[++index];
        if (argument == "--network") config.network = value;
        else if (argument == "--python") config.python = value;
        else if (argument == "--worker") config.worker = value;
        else if (argument == "--socket") config.socket = value;
        else if (argument == "--captures-dir") config.captures_dir = value;
        else if (argument == "--stage1") config.stage1 = value;
        else if (argument == "--stage2") config.stage2 = value;
        else if (argument == "--target-extrinsics") config.target_extrinsics = value;
        else if (argument == "--serial") config.serial = value;
        else if (argument == "--device") config.device = value;
        else if (argument == "--width") config.width = PositiveInteger(value, "width");
        else if (argument == "--height") config.height = PositiveInteger(value, "height");
        else if (argument == "--fps") config.fps = PositiveInteger(value, "fps");
        else if (argument == "--warmup-frames") config.warmup_frames = NonnegativeInteger(value, "warmup-frames");
        else if (argument == "--depth-patch-radius") config.depth_patch_radius = NonnegativeInteger(value, "depth-patch-radius");
        else if (argument == "--worker-timeout-ms") config.worker_timeout_ms = PositiveInteger(value, "worker-timeout-ms");
        else throw std::invalid_argument("unknown argument: " + argument);
    }
    if (!std::filesystem::is_regular_file(config.worker))
        throw std::invalid_argument("vision worker not found: " + config.worker.string());
    if (!std::filesystem::is_regular_file(config.python))
        throw std::invalid_argument("Python interpreter not found: " + config.python);
    return config;
}

class PidFile
{
public:
    explicit PidFile(const std::filesystem::path& root)
        : path_(root / "run/vision_service.pid")
    {
        std::filesystem::create_directories(path_.parent_path());
        descriptor_ = open(path_.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0664);
        if (descriptor_ < 0)
            throw std::runtime_error("Cannot open vision_service owner lock");
        if (flock(descriptor_, LOCK_EX | LOCK_NB) != 0) {
            close(descriptor_);
            descriptor_ = -1;
            throw std::runtime_error("Another vision_service holds the owner lock");
        }
        const std::string owner = std::to_string(getpid()) + "\n";
        if (ftruncate(descriptor_, 0) != 0 ||
            write(descriptor_, owner.data(), owner.size()) != static_cast<ssize_t>(owner.size())) {
            close(descriptor_);
            descriptor_ = -1;
            throw std::runtime_error("Cannot record vision_service owner PID");
        }
    }

    ~PidFile()
    {
        // Keep the inode: unlinking a lock file can let two owners lock different files.
        if (descriptor_ >= 0) close(descriptor_);
    }

private:
    std::filesystem::path path_;
    int descriptor_{-1};
    PidFile(const PidFile&) = delete;
    PidFile& operator=(const PidFile&) = delete;
};

std::vector<std::string> WorkerArguments(const Config& config)
{
    std::vector<std::string> values{
        config.python, config.worker.string(), "--socket", config.socket.string(),
        "--serial", config.serial, "--width", std::to_string(config.width),
        "--height", std::to_string(config.height), "--fps", std::to_string(config.fps),
        "--warmup-frames", std::to_string(config.warmup_frames), "--device", config.device,
        "--depth-patch-radius", std::to_string(config.depth_patch_radius),
        "--captures-dir", config.captures_dir.string(),
    };
    if (!config.stage1.empty()) { values.push_back("--stage1"); values.push_back(config.stage1.string()); }
    if (!config.stage2.empty()) { values.push_back("--stage2"); values.push_back(config.stage2.string()); }
    if (!config.target_extrinsics.empty()) {
        values.push_back("--target-extrinsics");
        values.push_back(config.target_extrinsics.string());
    }
    return values;
}

class WorkerProcess
{
public:
    explicit WorkerProcess(const Config& config) : socket_(config.socket)
    {
        std::filesystem::create_directories(socket_.parent_path());
        std::error_code error;
        std::filesystem::remove(socket_, error);
        auto arguments = WorkerArguments(config);
        pid_ = fork();
        if (pid_ < 0)
            throw std::runtime_error("failed to fork vision worker");
        if (pid_ == 0) {
            std::vector<char*> argv;
            argv.reserve(arguments.size() + 1);
            for (auto& argument : arguments)
                argv.push_back(argument.data());
            argv.push_back(nullptr);
            execv(argv[0], argv.data());
            std::cerr << "vision worker exec failed: " << std::strerror(errno) << '\n';
            _exit(127);
        }
        WaitUntilReady(config.worker_timeout_ms);
    }

    ~WorkerProcess()
    {
        if (pid_ <= 0)
            return;
        kill(pid_, SIGTERM);
        for (int attempt = 0; attempt < 30; ++attempt) {
            if (waitpid(pid_, nullptr, WNOHANG) == pid_)
                return;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        kill(pid_, SIGKILL);
        waitpid(pid_, nullptr, 0);
    }

    const std::filesystem::path& Socket() const { return socket_; }

private:
    void WaitUntilReady(int timeout_ms)
    {
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(timeout_ms);
        while (std::chrono::steady_clock::now() < deadline) {
            int status = 0;
            if (waitpid(pid_, &status, WNOHANG) == pid_) {
                pid_ = -1;
                throw std::runtime_error("vision worker exited during startup");
            }
            if (std::filesystem::exists(socket_))
                return;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        kill(pid_, SIGTERM);
        waitpid(pid_, nullptr, 0);
        pid_ = -1;
        throw std::runtime_error("timed out waiting for vision worker socket");
    }

    pid_t pid_{-1};
    std::filesystem::path socket_;
};

class UnixWorkerClient
{
public:
    UnixWorkerClient(std::filesystem::path socket, int timeout_ms)
        : socket_(std::move(socket)), timeout_ms_(timeout_ms)
    {}

    std::string Call(const std::string& action, const std::string& request)
    {
        std::lock_guard<std::mutex> guard(mutex_);
        const int descriptor = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if (descriptor < 0)
            throw std::runtime_error("cannot create worker socket");
        try {
            timeval timeout{timeout_ms_ / 1000, (timeout_ms_ % 1000) * 1000};
            setsockopt(descriptor, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
            setsockopt(descriptor, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
            sockaddr_un address{};
            address.sun_family = AF_UNIX;
            const std::string path = socket_.string();
            if (path.size() >= sizeof(address.sun_path))
                throw std::runtime_error("worker socket path is too long");
            std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
            if (connect(descriptor, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0)
                throw std::runtime_error("cannot connect to vision worker: " +
                                         std::string(std::strerror(errno)));
            const std::string envelope =
                "{\"action\":\"" + action + "\",\"request\":" + request + "}\n";
            std::size_t sent = 0;
            while (sent < envelope.size()) {
                const ssize_t count = send(descriptor, envelope.data() + sent,
                                           envelope.size() - sent, MSG_NOSIGNAL);
                if (count <= 0)
                    throw std::runtime_error("failed to send request to vision worker");
                sent += static_cast<std::size_t>(count);
            }
            std::string response;
            char buffer[4096];
            while (response.size() <= 1024 * 1024) {
                const ssize_t count = recv(descriptor, buffer, sizeof(buffer), 0);
                if (count <= 0)
                    throw std::runtime_error("vision worker closed or timed out");
                response.append(buffer, static_cast<std::size_t>(count));
                const auto newline = response.find('\n');
                if (newline != std::string::npos) {
                    close(descriptor);
                    return response.substr(0, newline);
                }
            }
            throw std::runtime_error("vision worker response is too large");
        } catch (...) {
            close(descriptor);
            throw;
        }
    }

private:
    std::filesystem::path socket_;
    int timeout_ms_;
    std::mutex mutex_;
};

class VisionRpcServer final : public unitree::robot::Server
{
public:
    explicit VisionRpcServer(UnixWorkerClient& worker)
        : Server(vision::kServiceName), worker_(worker)
    {}

    void Init() override
    {
        SetApiVersion(vision::kApiVersion);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(
            vision::kApiRecognizeOnce, &VisionRpcServer::HandleRecognizeOnce);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(
            vision::kApiGetStatus, &VisionRpcServer::HandleGetStatus);
    }

private:
    int32_t HandleRecognizeOnce(const std::string& parameter, std::string& data)
    {
        return Forward<vision::RecognizeRequest, vision::ObservationReply>(
            "recognize_once", parameter, data);
    }

    int32_t HandleGetStatus(const std::string& parameter, std::string& data)
    {
        return Forward<vision::StatusRequest, vision::StatusReply>(
            "get_status", parameter, data);
    }

    template <typename Request, typename Reply>
    int32_t Forward(const std::string& action, const std::string& parameter,
                    std::string& data)
    {
        Reply reply;
        Request request;
        try {
            unitree::common::FromJsonString(parameter, request);
        } catch (const std::exception& error) {
            reply.code = static_cast<int32_t>(vision::ResultCode::kInvalidArgument);
            reply.message = std::string("invalid request: ") + error.what();
            data = unitree::common::ToJsonString(reply);
            return 0;
        }
        try {
            data = worker_.Call(action, unitree::common::ToJsonString(request));
            unitree::common::FromJsonString(data, reply);
        } catch (const std::exception& error) {
            reply.code = static_cast<int32_t>(vision::ResultCode::kWorkerUnavailable);
            reply.message = error.what();
            data = unitree::common::ToJsonString(reply);
        }
        return 0;
    }

    UnixWorkerClient& worker_;
};

}  // namespace

int main(int argc, char** argv)
{
    try {
        const Config config = ParseArgs(argc, argv);
        PidFile pid_file(ProjectRoot(argv[0]));
        std::signal(SIGINT, HandleSignal);
        std::signal(SIGTERM, HandleSignal);
        WorkerProcess worker_process(config);
        UnixWorkerClient worker(worker_process.Socket(), config.worker_timeout_ms);
        unitree::robot::ChannelFactory::Instance()->Init(0, config.network);
        VisionRpcServer rpc(worker);
        rpc.Init();
        rpc.Start();
        std::cout << "vision_service ready: camera=resident inference=on-demand network="
                  << (config.network.empty() ? "default" : config.network) << '\n';
        while (running)
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "vision_service: " << error.what() << '\n';
        return 1;
    }
}
