#include <unitree/robot/go2/video/video_client.hpp>

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Config {
    std::filesystem::path output;
    std::string network;
};

std::filesystem::path DefaultOutputPath()
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  now.time_since_epoch()) %
                              1000;

    std::tm local_time{};
    localtime_r(&time, &local_time);

    std::ostringstream filename;
    filename << "head_camera_" << std::put_time(&local_time, "%Y%m%d_%H%M%S")
             << '_' << std::setfill('0') << std::setw(3)
             << milliseconds.count() << ".jpg";
    return std::filesystem::path("captures") / filename.str();
}

void PrintUsage(const char* program)
{
    std::cout << "Usage: " << program
              << " [--output FILE.jpg] [--network INTERFACE]\n";
}

Config ParseArguments(int argc, char** argv)
{
    Config config{DefaultOutputPath(), ""};
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--help" || argument == "-h") {
            PrintUsage(argv[0]);
            std::exit(0);
        }
        if ((argument == "--output" || argument == "-o") && index + 1 < argc) {
            config.output = argv[++index];
            continue;
        }
        if (argument == "--network" && index + 1 < argc) {
            config.network = argv[++index];
            continue;
        }
        throw std::invalid_argument("Unknown or incomplete argument: " + argument);
    }
    return config;
}

void SaveImage(const std::filesystem::path& output,
               const std::vector<uint8_t>& image)
{
    if (image.size() < 4 || image[0] != 0xff || image[1] != 0xd8 ||
        image[image.size() - 2] != 0xff || image.back() != 0xd9) {
        throw std::runtime_error("Camera returned invalid JPEG data");
    }

    if (!output.parent_path().empty()) {
        std::filesystem::create_directories(output.parent_path());
    }
    if (std::filesystem::exists(output)) {
        throw std::runtime_error("Refusing to overwrite existing file: " +
                                 output.string());
    }

    const std::filesystem::path temporary = output.string() + ".tmp";
    if (std::filesystem::exists(temporary)) {
        throw std::runtime_error("Temporary file already exists: " +
                                 temporary.string());
    }
    {
        std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
        if (!stream) {
            throw std::runtime_error("Cannot open output file: " + temporary.string());
        }
        stream.write(reinterpret_cast<const char*>(image.data()),
                     static_cast<std::streamsize>(image.size()));
        if (!stream) {
            throw std::runtime_error("Failed while writing image: " + temporary.string());
        }
    }
    std::filesystem::rename(temporary, output);
}

}  // namespace

int main(int argc, char** argv)
{
    try {
        const Config config = ParseArguments(argc, argv);
        unitree::robot::ChannelFactory::Instance()->Init(0, config.network);

        unitree::robot::go2::VideoClient video_client;
        video_client.SetTimeout(3.0F);
        video_client.Init();

        std::vector<uint8_t> image;
        const int32_t result = video_client.GetImageSample(image);
        if (result != 0) {
            std::cerr << "Failed to get a head-camera frame (error " << result
                      << "). Check that videohub is running and --network is correct.\n";
            return 1;
        }

        SaveImage(config.output, image);
        std::cout
            << std::filesystem::absolute(config.output).lexically_normal().string()
            << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "head_camera_capture: " << error.what() << '\n';
        return 2;
    }
}
