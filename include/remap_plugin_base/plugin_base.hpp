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

#ifndef REMAP_PLUGIN_BASE__PLUGIN_BASE_HPP_
#define REMAP_PLUGIN_BASE__PLUGIN_BASE_HPP_

#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "remap_plugin_base/regions_register.hpp"

#include "reg_of_space_server/srv/reg_of_space.hpp"
#include "reg_of_space_server/srv/remove_reg_of_space.hpp"
#include <std_msgs/msg/string.hpp>

namespace remap
{
namespace plugins
{
class PluginBase
{
protected:
  std::shared_ptr<rclcpp::Node> node_ptr_;
  std::shared_ptr<rclcpp::Node> plugin_node_ptr_;
  std::string name_;
  bool threaded_;

  std::shared_ptr<RegionsRegister> regions_register_;

  rclcpp::Client<reg_of_space_server::srv::RegOfSpace>::SharedPtr register_client_;
  rclcpp::Client<reg_of_space_server::srv::RemoveRegOfSpace>::SharedPtr remove_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr facts_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr remove_facts_pub_;

public:
  PluginBase();
  explicit PluginBase(std::shared_ptr<RegionsRegister> & regions_register);
  virtual ~PluginBase();
  void setup(
    const std::shared_ptr<rclcpp::Node> & node_ptr,
    const std::string & name,
    const bool & threaded);
  virtual void initialize() = 0;
  void pushFact(const std::string & fact) const;
  void removeFact(const std::string & fact) const;
};
}  // namespace plugins
}  // namespace remap
#endif  // REMAP_PLUGIN_BASE__PLUGIN_BASE_HPP_
