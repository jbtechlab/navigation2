// Copyright (c) 2021 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <vector>

#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "lanelet2_matching/LaneletMatching.h"

#include "nav2_behavior_tree/plugins/action/compute_path_through_poses_action.hpp"

namespace nav2_behavior_tree
{

ComputePathThroughPosesAction::ComputePathThroughPosesAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::ComputePathThroughPoses>(xml_tag_name, action_name, conf)
{
      tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ComputePathThroughPosesAction::on_tick()
{
  std::vector<geometry_msgs::msg::PoseStamped> goals_temp;
  auto exampleMapPath = "/overlay_ws/src/shadow/maps/lanelet2/map_v2.osm";
  getInput("goals", goals_temp);
  lanelet::LaneletMapPtr map = lanelet::load(exampleMapPath, 
  lanelet::projection::UtmProjector(lanelet::Origin({0.000022, 0.00005})));

  lanelet::matching::Object2d obj;
  auto lanelet_tf = this->tf_buffer_->lookupTransform(
            "map_lanelet", goals_temp[0].header.frame_id,
            node_->get_clock()->now());
  geometry_msgs::msg::PoseStamped pose;
  tf2::doTransform(goals_temp[0], pose, lanelet_tf);
  obj.pose.translation() = lanelet::BasicPoint2d(pose.pose.position.x,pose.pose.position.y);
  // obj.pose.linear() = Eigen::Rotation2D<double>(150. / 180. * M_PI).matrix();
    RCLCPP_INFO(
      node_->get_logger(),
      "Found matches %d", lanelet::matching::getDeterministicMatches(*map, obj, 0.1).size());
  
  auto matches = lanelet::matching::getDeterministicMatches(*map, obj, 0.1);
  lanelet::Lanelet first_close;
  geometry_msgs::msg::PoseStamped goal_lanelet, goal_freespace;
  // Outside lanelet
  if (matches.size() == 0){
    first_close = lanelet::matching::getDeterministicMatches(*map, obj, 1.5)[0].lanelet;
    goal_freespace = goals_temp[0];
    
  }
  else{
    first_close = matches[0].lanelet;
  }

  
  goal_lanelet.pose.position.x = first_close.centerline()[0].x();
  goal_lanelet.pose.position.y = first_close.centerline()[0].y();
  goal_lanelet.header = goals_temp[0].header;
  goal_lanelet.header.frame_id = "map_lanelet";
  
  auto inv_lanelet_tf = this->tf_buffer_->lookupTransform(
            goals_temp[0].header.frame_id,"map_lanelet",
            node_->get_clock()->now());
  
  tf2::doTransform(goal_lanelet, goal_lanelet, inv_lanelet_tf);


  goal_.goals = {goal_lanelet};
  goal_.planner_ids = {"LaneletPlanner"};

  if (matches.size() == 0){
    goal_.goals.push_back(goal_freespace);
    goal_.planner_ids.push_back("SmacPlanner");
      RCLCPP_INFO(
      node_->get_logger(),
      "free %s", goal_freespace.header.frame_id.c_str());
  }
      RCLCPP_INFO(
      node_->get_logger(),
      "last %s", goal_lanelet.header.frame_id.c_str());
  for (auto obj_map: matches)
  {
      RCLCPP_INFO(
      node_->get_logger(),
      "%d", obj_map.lanelet.id());
  }

  // getInput("planner_ids", goal_.planner_ids);
  // goal_.planner_ids = {"GridBased", "LolKek"};
  if (getInput("start", goal_.start)) {
    goal_.use_start = true;
  }
}

BT::NodeStatus ComputePathThroughPosesAction::on_success()
{
  setOutput("path", result_.result->path);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::ComputePathThroughPosesAction>(
        name, "compute_path_through_poses", config);
    };

  factory.registerBuilder<nav2_behavior_tree::ComputePathThroughPosesAction>(
    "ComputePathThroughPoses", builder);
}
