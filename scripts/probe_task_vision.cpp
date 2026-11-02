// Query the existing vision service only; no camera ownership or motor commands.
#include "vision/vision_client.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <cstdlib>

static uint64_t unix_ms()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

int main(int argc, char** argv)
{
    try {
        if (argc < 3 || argc > 5)
            throw std::runtime_error("Usage: probe_task_vision NETWORK NEW_JSON_PATH [--save-artifacts] [--camera-only|--status-only]");
        bool save_artifacts=false, camera_only=false, status_only=false;
        for(int i=3;i<argc;++i) {
            const std::string option(argv[i]);
            if(option=="--save-artifacts" && !save_artifacts)save_artifacts=true;
            else if(option=="--camera-only" && !camera_only)camera_only=true;
            else if(option=="--status-only" && !status_only)status_only=true;
            else throw std::runtime_error("Unknown or repeated capture option");
        }
        if(status_only && (save_artifacts || camera_only))throw std::runtime_error("Status-only cannot request images");
        const std::filesystem::path output(argv[2]);
        if (std::filesystem::exists(output)) throw std::runtime_error("Refusing to overwrite capture");
        const char* config = std::getenv("CYCLONEDDS_URI");
        if (config && *config) {
            unitree::common::JsonMap parameters;
            parameters[UT_DDS_PARAM_KEY_DOMAINID] = int32_t(0);
            parameters[UT_DDS_PARAM_KEY_CONFIG] = std::string(config);
            unitree::robot::ChannelFactory::Instance()->Init(parameters);
        } else {
            unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
        }
        vision::VisionClient client;
        client.SetTimeout(5.0F);
        client.Init();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        const uint64_t begin = unix_ms();
        vision::StatusReply status;
        const int status_result = client.GetStatus(begin, status);
        vision::RecognizeRequest request;
        request.request_id = begin + 1;
        request.require_3d = !camera_only;
        request.require_target_frame = false;
        request.save_artifacts = save_artifacts;
        const char* fresh_stream = std::getenv("CAMERA_C_FRESH_STREAM");
        request.restart_stream = fresh_stream && std::string(fresh_stream)=="1";
        vision::ObservationReply observation;
        int result = -1;
        uint64_t request_started = 0;
        if (!status_only && status_result == 0 && status.camera_ready && status.model_ready) {
            client.SetTimeout(60.0F);
            request_started = unix_ms();
            result = client.RecognizeOnce(request, observation);
        }
        std::filesystem::create_directories(output.parent_path());
        std::ofstream stream(output);
        stream << "{\"client_begin_unix_ms\":" << begin
               << ",\"request_started_unix_ms\":" << request_started
               << ",\"client_end_unix_ms\":" << unix_ms()
               << ",\"request_id\":" << request.request_id
               << ",\"status_result\":" << status_result
               << ",\"status\":" << unitree::common::ToJsonString(status)
               << ",\"observation_result\":" << result << ",\"observation\":";
        if (request_started) stream << unitree::common::ToJsonString(observation);
        else stream << "null";
        stream << "}\n";
        if (!stream) throw std::runtime_error("Cannot save capture");
        std::cout << "status=" << status_result << " recognize=" << result
                  << " output=" << output << '\n';
        return status_result == 0 && (status_only || result == 0) ? 0 : 2;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
