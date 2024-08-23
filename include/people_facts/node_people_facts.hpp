// Copyright 2024 PAL Robotics S.L.
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

#ifndef PEOPLE_FACTS__NODE_PEOPLE_FACTS_HPP_
#define PEOPLE_FACTS__NODE_PEOPLE_FACTS_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <hri/hri.hpp>
#include <hri/person.hpp>
#include <hri/types.hpp>
#include <hri_msgs/msg/ids_match.hpp>
#include <kb_msgs/srv/revise.hpp>

namespace people_facts
{

using LifecycleCallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class NodePeopleFacts : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit NodePeopleFacts(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~NodePeopleFacts();


  LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &);
  LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &);
  LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

private:
  void publish_diagnostics();

  void internal_cleanup();
  void internal_deactivate();

  void update();
  bool needsUpdate(const hri::ID &, const std::string &, size_t);

  std::shared_ptr<rclcpp::TimerBase> update_timer_;

  void onPerson(hri::PersonPtr);
  void onTrackedPerson(hri::PersonPtr);
  void onTrackedPersonLost(const hri::ID &);

  std::shared_ptr<hri::HRIListener> hri_listener_;
  rclcpp::Client<kb_msgs::srv::Revise>::SharedPtr kb_revise_;

  std::map<hri::ID, std::map<std::string, size_t>> previous_state_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  std::shared_ptr<rclcpp::TimerBase> diagnostics_timer_;
};

}  // namespace people_facts

#endif  // PEOPLE_FACTS__NODE_PEOPLE_FACTS_HPP_
