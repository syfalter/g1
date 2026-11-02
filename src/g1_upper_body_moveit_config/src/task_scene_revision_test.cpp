#include <iostream>
#include <stdexcept>
#include "task_scene_revision.hpp"

int main()
{
  auto require = [](bool result, const char* name) {
    if (!result) throw std::runtime_error(name);
  };
  TaskSceneRevision revision;
  moveit_msgs::msg::PlanningScene state;
  state.is_diff = true;
  state.link_padding.resize(103);
  state.link_scale.resize(103);
  for (int i=0; i<103; ++i) {
    state.link_padding[i].link_name = "link_"+std::to_string(i);
    state.link_padding[i].padding = 0.;
    state.link_scale[i].link_name = "link_"+std::to_string(i);
    state.link_scale[i].scale = 1.;
  }
  require(revision.changed(state), "initial geometry observed");
  for (int i=0; i<100; ++i) {
    state.robot_state.joint_state.position = {i*.001};
    require(!revision.changed(state), "repeated 103 padding/scale entries are state-only");
  }
  state.link_padding[0].padding = .002;
  require(revision.changed(state), "actual padding change invalidates");
  require(!revision.changed(state), "same changed padding is not new");
  state.link_scale[0].scale = 1.1;
  require(revision.changed(state), "actual scale change invalidates");
  state.fixed_frame_transforms.resize(1);
  auto& tf = state.fixed_frame_transforms[0];
  tf.header.frame_id = "pelvis"; tf.child_frame_id = "table"; tf.transform.rotation.w=1.;
  require(revision.changed(state), "new transform invalidates");
  tf.header.stamp.sec++;
  require(!revision.changed(state), "only transform timestamp changed");
  tf.transform.translation.x=.001;
  require(revision.changed(state), "actual transform change invalidates");
  auto geometry = state;
  geometry.world.collision_objects.resize(1);
  require(revision.changed(geometry), "world geometry invalidates");
  geometry = state; geometry.robot_state.attached_collision_objects.resize(1);
  require(revision.changed(geometry), "attachment invalidates");
  geometry = state; geometry.allowed_collision_matrix.default_entry_names={"link"};
  require(revision.changed(geometry), "default collision permissions invalidate");
  geometry = state; geometry.world.octomap.octomap.data={1};
  require(revision.changed(geometry), "octomap invalidates");
  geometry = state; geometry.is_diff=false;
  require(revision.changed(geometry), "full snapshot invalidates");
  require(!revision.changed(state), "subsequent identical diff is stable");
  std::cout << "PASS: repeated state diffs stay stable; actual scene changes invalidate\n";
}
