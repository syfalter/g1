// Status queries and JPEG capture only. No motor, pose, hold, or fault commands.
#include "rh56/hand_client.hpp"
#include <unitree/robot/go2/video/video_client.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>

int main(int argc, char** argv)
{
    try {
        if (argc != 3) throw std::runtime_error("Usage: probe_hand_camera_readonly NETWORK OUTPUT_DIR");
        const std::filesystem::path root(argv[2]);
        std::filesystem::create_directories(root);
        unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
        rh56::HandClient hand;
        hand.SetTimeout(3.0f); hand.Init();
        unitree::robot::go2::VideoClient camera;
        camera.SetTimeout(5.0f); camera.Init();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        for (const std::string side : {"left", "right"}) {
            rh56::StateReply state;
            const int result = hand.GetState(side, state, true);
            const auto path = root / ("hand_rpc_" + side + ".json");
            if (std::filesystem::exists(path)) throw std::runtime_error("Output already exists");
            std::ofstream output(path);
            output << "{\"rpc_result\":" << result << ",\"online\":"
                   << (result == 0 && state.online ? "true" : "false") << "}\n";
            if (!output) throw std::runtime_error("Cannot save hand status");
            std::cout << side << " GetState result=" << result << std::endl;
        }
        std::vector<uint8_t> image;
        const int result = camera.GetImageSample(image);
        const bool jpeg = result == 0 && image.size() >= 4 && image[0] == 0xff
            && image[1] == 0xd8 && image[image.size()-2] == 0xff && image.back() == 0xd9;
        std::ofstream meta(root / "head_camera_rpc.json");
        meta << "{\"rpc_result\":" << result << ",\"bytes\":" << image.size()
             << ",\"valid_jpeg\":" << (jpeg ? "true" : "false") << "}\n";
        if (jpeg) {
            const auto path = root / "head_camera.jpg";
            if (std::filesystem::exists(path)) throw std::runtime_error("Image already exists");
            std::ofstream output(path, std::ios::binary);
            output.write(reinterpret_cast<const char*>(image.data()), image.size());
            if (!output) throw std::runtime_error("Cannot save JPEG");
        }
        std::cout << "Head JPEG result=" << result << " bytes=" << image.size() << std::endl;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n'; return 1;
    }
}
