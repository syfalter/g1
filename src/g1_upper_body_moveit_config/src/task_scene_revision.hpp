#pragma once
#include <map>
#include <string>
#include <utility>
#include <moveit_msgs/msg/planning_scene.hpp>

// MoveIt repeats padding/scaling in state-only diffs. Only changes to those
// values invalidate geometry; TF timestamps alone do not change geometry.
class TaskSceneRevision
{
public:
  bool changed(const moveit_msgs::msg::PlanningScene& message)
  {
    bool changed = !message.is_diff || !message.world.collision_objects.empty() ||
      !message.world.octomap.octomap.data.empty() ||
      !message.allowed_collision_matrix.entry_names.empty() ||
      !message.allowed_collision_matrix.default_entry_names.empty() ||
      !message.robot_state.attached_collision_objects.empty();
    if (!message.is_diff) {
      padding_.clear(); scale_.clear(); transforms_.clear();
    }
    for (const auto& item : message.link_padding)
      changed = update(padding_, item.link_name, item.padding) || changed;
    for (const auto& item : message.link_scale)
      changed = update(scale_, item.link_name, item.scale) || changed;
    for (const auto& item : message.fixed_frame_transforms)
      changed = update(transforms_, std::make_pair(item.header.frame_id, item.child_frame_id), item.transform) || changed;
    return changed;
  }

private:
  template<typename Key, typename Value>
  static bool update(std::map<Key, Value>& values, const Key& key, const Value& value)
  {
    const auto found = values.find(key);
    if (found != values.end() && found->second == value) return false;
    values[key] = value;
    return true;
  }
  std::map<std::string, double> padding_, scale_;
  std::map<std::pair<std::string, std::string>, geometry_msgs::msg::Transform> transforms_;
};
