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

#include <remap_plugin_base/plugin_base.hpp>

namespace remap
{
namespace plugins
{
PluginBase::PluginBase() {}

PluginBase::PluginBase(std::shared_ptr<regions_register::RegionsRegister> & regions_register)
{
  regions_register_ = regions_register;
}

PluginBase::~PluginBase()
{
  remove_client_.reset();
  register_client_.reset();
  node_ptr_.reset();
}

void PluginBase::setup(
  const std::shared_ptr<rclcpp::Node> & node_ptr,
  const std::string & name,
  const bool & threaded)
{
  node_ptr_ = node_ptr;
  plugin_node_ptr_ = std::make_shared<rclcpp::Node>(name);
  name_ = name;
  threaded_ = threaded;
  regions_register_ = std::make_shared<regions_register::RegionsRegister>(threaded_);
  register_client_ = plugin_node_ptr_->create_client<reg_of_space_server::srv::RegOfSpace>(
    "register_region_of_space");
  remove_client_ = plugin_node_ptr_->create_client<reg_of_space_server::srv::RemoveRegOfSpace>(
    "remove_region_of_space");
  facts_pub_ = plugin_node_ptr_->create_publisher<std_msgs::msg::String>("/kb/add_fact", 10);
  remove_facts_pub_ = plugin_node_ptr_->create_publisher<std_msgs::msg::String>(
    "/kb/remove_fact",
    10);
}

void PluginBase::pushFact(const std::string & fact) const
{
  auto fact_msg = std_msgs::msg::String();
  fact_msg.data = fact;
  facts_pub_->publish(fact_msg);
}

void PluginBase::removeFact(const std::string & fact) const
{
  auto fact_msg = std_msgs::msg::String();
  fact_msg.data = fact;
  remove_facts_pub_->publish(fact_msg);
}
}          // namespace plugins
}  // namespace remap
