// C-only read-only receiver. stdout is a local pipe, never a robot command topic.
#include <array>
#include <cmath>
#include <iostream>
#include <csignal>
#include <cstring>
#include <cstdlib>
#include <map>
#include <sys/prctl.h>
#include <time.h>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <unitree_hg/msg/low_state.hpp>
#include <dds/dds.h>
#include <rmw/rmw.h>

using Json=nlohmann::json;
// This installed RMW emits a publication handle, not a GUID. Resolve it
// from DDS built-in samples, never from the sole publisher in the ROS graph.
class PublisherMap {
  dds_entity_t participant_;
  dds_entity_t reader_;
  std::map<dds_instance_handle_t,std::vector<uint8_t>> endpoints_;
public:
  PublisherMap() {
    if(std::strcmp(rmw_get_implementation_identifier(),"rmw_cyclonedds_cpp")!=0)
      throw std::runtime_error("C receiver requires installed CycloneDDS handle contract");
    const auto domain=std::getenv("ROS_DOMAIN_ID");
    participant_=dds_create_participant(domain?std::stoul(domain):0,nullptr,nullptr);
    if(participant_<0)throw std::runtime_error("DDS discovery participant failed");
    reader_=dds_create_reader(participant_,DDS_BUILTIN_TOPIC_DCPSPUBLICATION,nullptr,nullptr);
    if(reader_<0){dds_delete(participant_);throw std::runtime_error("DDS publication discovery failed");}
  }
  ~PublisherMap(){dds_delete(participant_);}
  Json resolve(const uint8_t *raw) {
    void *samples[32]{};dds_sample_info_t infos[32]{};
    int count;
    while((count=dds_take(reader_,samples,infos,32,32))>0){
      for(int i=0;i<count;++i){
        if(infos[i].valid_data && infos[i].instance_state==DDS_ALIVE_INSTANCE_STATE){
          const auto *endpoint=static_cast<dds_builtintopic_endpoint_t *>(samples[i]);
          std::vector<uint8_t> guid(RMW_GID_STORAGE_SIZE,0);
          std::memcpy(guid.data(),endpoint->key.v,sizeof(endpoint->key.v));
          endpoints_[infos[i].instance_handle]=std::move(guid);
        }else endpoints_.erase(infos[i].instance_handle);
      }
      dds_return_loan(reader_,samples,count);
      std::memset(samples,0,sizeof(samples));
    }
    if(count<0)throw std::runtime_error("DDS publication discovery read failed");
    dds_instance_handle_t handle{};std::memcpy(&handle,raw,sizeof(handle));
    for(size_t i=sizeof(handle);i<RMW_GID_STORAGE_SIZE;++i)
      if(raw[i]!=0)throw std::runtime_error("Unexpected RMW publication handle layout");
    auto entry=endpoints_.find(handle);
    return entry==endpoints_.end()?Json(nullptr):Json(entry->second);
  }
};
double seconds(clockid_t clock) {
  timespec value{};
  if(clock_gettime(clock,&value)!=0)throw std::runtime_error("clock_gettime failed");
  return double(value.tv_sec)+double(value.tv_nsec)*1e-9;
}
Json metadata(const rclcpp::MessageInfo &info,PublisherMap &publishers) {
  const auto &raw=info.get_rmw_message_info();
  const auto received=seconds(CLOCK_MONOTONIC),wall=seconds(CLOCK_REALTIME);
  return Json{{"publisher_gid",publishers.resolve(raw.publisher_gid.data)},
    {"raw_rmw_publisher_gid",std::vector<uint8_t>(raw.publisher_gid.data,raw.publisher_gid.data+RMW_GID_STORAGE_SIZE)},
    {"publisher_identity_contract","cyclonedds_builtin_publication_handle_v1"},
    {"source_timestamp",raw.source_timestamp},{"rmw_received_timestamp",raw.received_timestamp},
    {"received_monotonic",received},{"received_realtime",wall},
    {"reader_pid",getpid()},{"clock_basis","local_monotonic_receipt_v2"}};
}
Json finite(float value) {return std::isfinite(value)?Json(value):Json(nullptr);}

int main(int argc,char **argv) {
  const auto parent=getppid();
  if(prctl(PR_SET_PDEATHSIG,SIGTERM)!=0 || getppid()!=parent)return 1;
  rclcpp::init(argc,argv);
  auto options=rclcpp::NodeOptions().enable_rosout(false).start_parameter_services(false).start_parameter_event_publisher(false);
  auto node=std::make_shared<rclcpp::Node>("g1_camera_c_actual_reader",options);
  PublisherMap publishers;
  auto emit=[](const Json &value){std::cout<<value.dump()<<'\n'<<std::flush;};
  auto actual=node->create_subscription<unitree_hg::msg::LowState>("/lowstate",rclcpp::SensorDataQoS().keep_last(1),
    [&](unitree_hg::msg::LowState::ConstSharedPtr msg,const rclcpp::MessageInfo &info){
      auto record=metadata(info,publishers);record["kind"]="actual";record["tick"]=msg->tick;
      record["mode_machine"]=msg->mode_machine;record["mode_pr"]=msg->mode_pr;
      auto motors=Json::array();
      for(const auto &m:msg->motor_state)motors.push_back({{"q",finite(m.q)},{"dq",finite(m.dq)},{"vol",finite(m.vol)}});
      record["motor_state"]=std::move(motors);emit(record);
    });
  auto owner=node->create_subscription<std_msgs::msg::String>("/g1/task/console_status",rclcpp::QoS(1).transient_local(),
    [&](std_msgs::msg::String::ConstSharedPtr msg,const rclcpp::MessageInfo &info){
      auto record=metadata(info,publishers);record["kind"]="owner";record["data"]=msg->data;emit(record);
    });
  auto model=node->create_subscription<std_msgs::msg::String>("/robot_description",rclcpp::QoS(1).transient_local(),
    [&](std_msgs::msg::String::ConstSharedPtr msg,const rclcpp::MessageInfo &info){
      auto record=metadata(info,publishers);record["kind"]="model";record["data"]=msg->data;emit(record);
    });
  rclcpp::spin(node);rclcpp::shutdown();return 0;
}
