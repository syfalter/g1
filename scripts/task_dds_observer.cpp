// Read-only discovery and command-source monitor. No application writers.
#include <dds/dds.hpp>
#include <dds/dds.h>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/common/json/json.hpp>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <thread>

using unitree::common::JsonArray;
using unitree::common::JsonMap;

static std::string guid(const dds_guid_t& key) {
    std::ostringstream out;
    for (auto byte : key.v) out << std::hex << std::setw(2) << std::setfill('0') << unsigned(byte);
    return out.str();
}

static std::string property(const dds_qos_t* qos, const char* name) {
    char* value = nullptr;
    if (!dds_qget_prop(qos, name, &value)) return {};
    std::string result(value);
    dds_free(value);
    return result;
}

template<class T, class F> static void drain(dds_entity_t reader, F apply) {
    void* samples[64] = {};
    dds_sample_info_t info[64];
    int n = dds_take(reader, samples, info, 64, 64);
    if (n < 0) throw std::runtime_error("DDS discovery read failed");
    for (int i = 0; i < n; ++i) apply(*static_cast<T*>(samples[i]), info[i].valid_data);
    if (n) dds_return_loan(reader, samples, n);
}

int main() {
    try {
        const char* domain = std::getenv("ROS_DOMAIN_ID");
        dds::domain::DomainParticipant participant(domain ? std::stoul(domain) : 0);
        const auto native = participant.delegate()->get_ddsc_entity();
        auto pr = dds_create_reader(native, DDS_BUILTIN_TOPIC_DCPSPARTICIPANT, nullptr, nullptr);
        auto wr = dds_create_reader(native, DDS_BUILTIN_TOPIC_DCPSPUBLICATION, nullptr, nullptr);
        auto rr = dds_create_reader(native, DDS_BUILTIN_TOPIC_DCPSSUBSCRIPTION, nullptr, nullptr);
        if (pr < 0 || wr < 0 || rr < 0) throw std::runtime_error("DDS discovery reader creation failed");
        using LowCmd = unitree_hg::msg::dds_::LowCmd_;
        dds::topic::Topic<LowCmd> topic(participant, "rt/arm_sdk");
        dds::sub::Subscriber subscriber(participant);
        auto qos = subscriber.default_datareader_qos();
        qos << dds::core::policy::Reliability::BestEffort();
        qos << dds::core::policy::History::KeepLast(256);
        dds::sub::DataReader<LowCmd> commands(subscriber, topic, qos);
        std::map<std::string, JsonMap> participants, writers, readers;
        std::map<std::string, uint64_t> counts;
        uint64_t unknown = 0;
        const auto started = std::chrono::steady_clock::now();
        while (true) {
            drain<dds_builtintopic_participant_t>(pr, [&](const auto& p, bool valid) {
                if (!valid) { participants.erase(guid(p.key)); return; }
                participants[guid(p.key)] = JsonMap{{"process", property(p.qos, "__ProcessName")},
                    {"pid", property(p.qos, "__Pid")}, {"host", property(p.qos, "__Hostname")}};
            });
            auto endpoints = [&](auto& table, const auto& p, bool valid) {
                const auto id = guid(p.key);
                if (!valid) { table.erase(id); return; }
                if (std::string(p.topic_name) != "rt/arm_sdk") return;
                table[id] = JsonMap{{"guid", id}, {"participant", guid(p.participant_key)},
                                     {"type", std::string(p.type_name)}};
            };
            drain<dds_builtintopic_endpoint_t>(wr, [&](const auto& p, bool v) { endpoints(writers, p, v); });
            drain<dds_builtintopic_endpoint_t>(rr, [&](const auto& p, bool v) { endpoints(readers, p, v); });
            for (const auto& sample : commands.take()) {
                if (!sample.info().valid()) continue;
                auto* endpoint = dds_get_matched_publication_data(commands.delegate()->get_ddsc_entity(),
                    sample.info().publication_handle().delegate().handle());
                if (endpoint) {
                    ++counts[guid(endpoint->key)];
                    dds_builtintopic_free_endpoint(endpoint);
                } else ++unknown;
            }
            JsonMap peers, observed;
            JsonArray pubs, subs;
            for (const auto& p : participants) peers[p.first] = p.second;
            for (const auto& p : writers) pubs.push_back(p.second);
            for (const auto& p : readers) subs.push_back(p.second);
            for (const auto& p : counts) observed[p.first] = p.second;
            const auto age = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
            JsonMap result{{"participants", peers}, {"writers", pubs}, {"readers", subs},
                {"messages", observed}, {"unknown_messages", unknown}, {"observed_seconds", age}};
            std::cout << unitree::common::ToJsonString(result) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}
