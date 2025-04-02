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

#include <std_msgs/msg/string.hpp>

#include "remap_plugin_base/semantic_plugin.hpp"

namespace remap
{
namespace plugins
{
SemanticPlugin::SemanticPlugin()
: PluginBase(),
  simulation_(false) {}

SemanticPlugin::SemanticPlugin(
  std::shared_ptr<map_handler::SemanticMapHandler> & semantic_map,
  std::shared_ptr<remap::regions_register::RegionsRegister> & regions_register)
: PluginBase(regions_register)
{
  semantic_map_ = semantic_map;
}

SemanticPlugin::~SemanticPlugin()
{
  timer_.reset();
  regions_register_.reset();
  semantic_map_.reset();
}

void SemanticPlugin::storeRegionsRelationships(
  std::map<int, std::map<int,
  std::string>> relationships_matrix)
{
  (void) relationships_matrix;
}

void SemanticPlugin::storeEntitiesRelationships(
  std::map<std::string, std::map<std::string,
  std::string>> relationships_matrix)
{
  (void) relationships_matrix;
}

void SemanticPlugin::initializeSimulationStructures()
{
  start_simulation_subscription_ = plugin_node_ptr_->create_subscription<std_msgs::msg::Bool>(
    "start_simulation", 10,
    std::bind(&SemanticPlugin::startSimulationCallback, this, std::placeholders::_1));
  stop_simulation_subscription_ = plugin_node_ptr_->create_subscription<std_msgs::msg::Bool>(
    "stop_simulation", 10,
    std::bind(&SemanticPlugin::stopSimulationCallback, this, std::placeholders::_1));
  step_subscription_ = plugin_node_ptr_->create_subscription<std_msgs::msg::Float32>(
    "step_size", 10,
    std::bind(&SemanticPlugin::stepCallback, this, std::placeholders::_1));
  forward_subscription_ = plugin_node_ptr_->create_subscription<std_msgs::msg::Bool>(
    "forward_step", 10,
    std::bind(&SemanticPlugin::forwardCallback, this, std::placeholders::_1));
  backward_subscription_ = plugin_node_ptr_->create_subscription<std_msgs::msg::Bool>(
    "backward_step", 10,
    std::bind(&SemanticPlugin::backwardCallback, this, std::placeholders::_1));
}

void SemanticPlugin::startSimulationCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  (void) msg;

  if (simulation_) {
    return;
  }
  simulation_ = true;
  simulated_time_ = plugin_node_ptr_->get_clock()->now().seconds();
}

void SemanticPlugin::stopSimulationCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  (void) msg;

  simulation_ = false;
}

void SemanticPlugin::stepCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  (void) msg;

  step_size_ = msg->data;
}

void SemanticPlugin::forwardCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  (void) msg;

  simulated_time_ += step_size_;
}

void SemanticPlugin::backwardCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  (void) msg;

  simulated_time_ -= step_size_;
}
}          // namespace plugins
}  // namespace remap
