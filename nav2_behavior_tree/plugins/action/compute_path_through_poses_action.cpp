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
}

void ComputePathThroughPosesAction::on_tick()
{
  std::vector<geometry_msgs::msg::PoseStamped> goals_temp;
  auto exampleMapPath = "/overlay_ws/src/shadow/maps/lanelet2/map_v2.osm";
  getInput("goals", goals_temp);
  lanelet::LaneletMapPtr map = lanelet::load(exampleMapPath, lanelet::projection::UtmProjector(lanelet::Origin({0, 0})));

  lanelet::matching::Object2d obj;
  obj.pose.translation() = lanelet::BasicPoint2d(0.0000029,0.0000789);
  // obj.pose.linear() = Eigen::Rotation2D<double>(150. / 180. * M_PI).matrix();
    RCLCPP_INFO(
      node_->get_logger(),
      "Found matches %d", lanelet::matching::utils::findWithin(map->laneletLayer, obj, 0.1).size());
  for (auto obj_map: lanelet::matching::utils::findWithin(map->laneletLayer, obj, 0.1))
  {
      RCLCPP_INFO(
      node_->get_logger(),
      "%d", obj_map);
  }

  getInput("planner_ids", goal_.planner_ids);
  goal_.planner_ids = {"GridBased", "LolKek"};
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
