#pragma once
// Foxy mistakes Unitree's RPC header for std_msgs/Header. It has no ROS stamp.
// Only generic topic statistics are disabled for this type; the gateway's own
// service response freshness and LowState watchdogs are unchanged.
#include <unitree_api/msg/response.hpp>
#include <libstatistics_collector/topic_statistics_collector/received_message_age.hpp>
namespace libstatistics_collector { namespace topic_statistics_collector {
template<>
struct HasHeader<unitree_api::msg::Response, void> : public std::false_type {};
}}
