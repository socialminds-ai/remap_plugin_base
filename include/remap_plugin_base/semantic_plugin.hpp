// Copyright 2025 PAL Robotics, S.L.
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

#ifndef REMAP_PLUGIN_BASE__SEMANTIC_PLUGIN_HPP_
#define REMAP_PLUGIN_BASE__SEMANTIC_PLUGIN_HPP_

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <remap_map_handler/semantic_map_handler.hpp>
#include <remap_regions_register/regions_register.hpp>

#include <vdb2pc/vdb2pc_publisher.hpp>

#include "remap_plugin_base/plugin_base.hpp"

namespace remap
{
namespace plugins
{
class SemanticPlugin : public PluginBase
{
protected:
  rclcpp::TimerBase::SharedPtr timer_;

  bool simulation_;
  double simulated_time_;
  double step_size_;

  std::shared_ptr<map_handler::SemanticMapHandler> semantic_map_;
  std::shared_ptr<vdb2pc::ros_utils::VDB2PCPublisher<openvdb::Int32Grid>> map_publisher_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_simulation_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_simulation_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr step_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr forward_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr backward_subscription_;

public:
  SemanticPlugin();
  SemanticPlugin(
    std::shared_ptr<map_handler::SemanticMapHandler> & semantic_map,
    std::shared_ptr<remap::regions_register::RegionsRegister> & regions_register);
  virtual ~SemanticPlugin();
  virtual void run() = 0;
  virtual void storeRegionsRelationships(
    std::map<int, std::map<int,
    std::string>> relationships_matrix);
  virtual void storeEntitiesRelationships(
    std::map<std::string, std::map<std::string,
    std::string>> relationships_matrix);

  inline void setSemanticMapHandler(
    std::shared_ptr<map_handler::SemanticMapHandler> & semantic_map)
  {
    semantic_map_ = semantic_map;
  }
  inline void setRegionsRegister(
    std::shared_ptr<remap::regions_register::RegionsRegister> & regions_register)
  {
    regions_register_ = regions_register;
  }

  void initializeSimulationStructures();

  void startSimulationCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void stopSimulationCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void stepCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void forwardCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void backwardCallback(const std_msgs::msg::Bool::SharedPtr msg);
};
}  // namespace plugins
}  // namespace remap
#endif  // REMAP_PLUGIN_BASE__SEMANTIC_PLUGIN_HPP_
