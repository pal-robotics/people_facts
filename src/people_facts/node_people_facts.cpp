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

#include "people_facts/node_people_facts.hpp"

#include <memory>
#include <chrono>
#include <functional>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <lifecycle_msgs/msg/state.hpp>

using namespace std::chrono_literals;

using std::string;
using std::vector;
using std::hash;
using hri::ID;
using hri::PersonPtr;
using hri::EngagementLevel;


namespace people_facts
{

NodePeopleFacts::NodePeopleFacts(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("people_facts", "", options)
{
  RCLCPP_INFO(this->get_logger(), "State: Unconfigured");
}

NodePeopleFacts::~NodePeopleFacts()
{
  on_shutdown(get_current_state());
}

LifecycleCallbackReturn
NodePeopleFacts::on_configure(const rclcpp_lifecycle::State &)
{
  diagnostics_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 1);
  diagnostics_timer_ = rclcpp::create_timer(
    this, this->get_clock(), 1s,
    std::bind(&NodePeopleFacts::publish_diagnostics, this));

  RCLCPP_INFO(this->get_logger(), "State: Inactive");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn
NodePeopleFacts::on_cleanup(const rclcpp_lifecycle::State &)
{
  internal_cleanup();
  RCLCPP_INFO(this->get_logger(), "State: Unconfigured");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn
NodePeopleFacts::on_activate(const rclcpp_lifecycle::State &)
{
  kb_revise_ = this->create_client<kb_msgs::srv::Revise>("/kb/revise");

  while (!kb_revise_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"),
        "Interrupted while waiting for the service. Exiting.");
      return LifecycleCallbackReturn::FAILURE;
    }
    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp"),
      "Knowledge base's 'revise' service not available yet, waiting...");
  }

  hri_listener_ = hri::HRIListener::create(shared_from_this());

  // bind the callback to this class' 'onPerson' method.
  hri_listener_->onPerson(
    bind(&NodePeopleFacts::onPerson, this, std::placeholders::_1));

  hri_listener_->onTrackedPerson(
    bind(&NodePeopleFacts::onTrackedPerson, this, std::placeholders::_1));

  hri_listener_->onTrackedPersonLost(
    bind(&NodePeopleFacts::onTrackedPersonLost, this, std::placeholders::_1));

  update_timer_ =
    rclcpp::create_timer(
    this, this->get_clock(), 100ms,
    std::bind(&NodePeopleFacts::update, this));

  RCLCPP_INFO(this->get_logger(), "State: Active");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn
NodePeopleFacts::on_deactivate(const rclcpp_lifecycle::State &)
{
  internal_deactivate();
  RCLCPP_INFO(this->get_logger(), "State: Inactive");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn
NodePeopleFacts::on_shutdown(const rclcpp_lifecycle::State & state)
{
  if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    internal_deactivate();
  }
  internal_cleanup();
  RCLCPP_INFO(this->get_logger(), "State: Finalized");
  return LifecycleCallbackReturn::SUCCESS;
}

void NodePeopleFacts::internal_deactivate()
{
  update_timer_.reset();
  hri_listener_.reset();
  kb_revise_.reset();
}

void NodePeopleFacts::internal_cleanup()
{
  diagnostics_timer_.reset();
  diagnostics_pub_.reset();
}

void NodePeopleFacts::update()
{
  vector<string> stmts_to_add;
  vector<string> stmts_to_remove;

  for (auto person : hri_listener_->getTrackedPersons()) {
    auto p = person.second;

    auto id = p->id();

    string predicate(" hasEngagementLevel ");
    if (p->engagementStatus()) {
      auto engagement = *(p->engagementStatus());

      if (needsUpdate(id, predicate, hash<EngagementLevel>{}(engagement))) {
        RCLCPP_INFO_STREAM(
          this->get_logger(), "Revising value of predicate <"
            << predicate << "> for "
            << id);
        switch (engagement) {
          case EngagementLevel::kEngaged:
            stmts_to_add.push_back(id + predicate + "engaged");
            break;
          case EngagementLevel::kEngaging:
            stmts_to_add.push_back(id + predicate + "engaging");
            break;
          case EngagementLevel::kDisengaging:
            stmts_to_add.push_back(id + predicate + "disengaging");
            break;
          case EngagementLevel::kDisengaged:
            stmts_to_add.push_back(id + predicate + "disengaged");
            break;
        }
      }

    } else {
      if (needsUpdate(id, predicate, 0)) {
        RCLCPP_INFO_STREAM(
          this->get_logger(), "Revising value of predicate <"
            << predicate << "> for "
            << id);
        stmts_to_remove.push_back(id + predicate + "engaged");
        stmts_to_remove.push_back(id + predicate + "disengaged");
        stmts_to_remove.push_back(id + predicate + "engaging");
        stmts_to_remove.push_back(id + predicate + "disengaging");
      }
    }
  }

  if (!stmts_to_add.empty()) {
    auto revise = std::make_shared<kb_msgs::srv::Revise::Request>();
    revise->method = kb_msgs::srv::Revise::Request::UPDATE;
    revise->statements = stmts_to_add;

    kb_revise_->async_send_request(revise);
  }
  if (!stmts_to_remove.empty()) {
    auto revise = std::make_shared<kb_msgs::srv::Revise::Request>();
    revise->method = kb_msgs::srv::Revise::Request::RETRACT;
    revise->statements = stmts_to_remove;

    kb_revise_->async_send_request(revise);
  }
}

void NodePeopleFacts::onPerson(PersonPtr person)
{
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "New person detected. ID: " << person->id());

  auto revise = std::make_shared<kb_msgs::srv::Revise::Request>();
  revise->method = kb_msgs::srv::Revise::Request::UPDATE;
  revise->statements = {person->id() + " rdf:type Human"};

  kb_revise_->async_send_request(revise);
}

void NodePeopleFacts::onTrackedPerson(PersonPtr person)
{
  auto revise = std::make_shared<kb_msgs::srv::Revise::Request>();
  revise->method = kb_msgs::srv::Revise::Request::UPDATE;
  revise->statements = {person->id() + " currentlyTracked true"};

  kb_revise_->async_send_request(revise);
}

void NodePeopleFacts::onTrackedPersonLost(const ID & id)
{
  auto revise = std::make_shared<kb_msgs::srv::Revise::Request>();
  revise->method = kb_msgs::srv::Revise::Request::RETRACT;
  revise->statements = {id + " currentlyTracked false"};

  kb_revise_->async_send_request(revise);
}

/** returns true if a specific 'key' has changed and, accordingly, the
 * knowledge base needs update.
 *
 * Attention: *not* idempotent. Calling `needsUpdate` also updates the
 * internal keystore with the new hash value.
 */
bool NodePeopleFacts::needsUpdate(
  const ID & id, const string & key,
  size_t hash_value)
{
  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Checking value of " << key << " for " << id
                         << " (hash:" << hash_value << ")");
  if (previous_state_.count(id) == 0) {
    previous_state_[id][key] = hash_value;
    return true;
  }
  if (previous_state_[id].count(key) == 0) {
    previous_state_[id][key] = hash_value;
    return true;
  }

  if (previous_state_[id][key] != hash_value) {
    previous_state_[id][key] = hash_value;
    return true;
  }

  return false;
}

void NodePeopleFacts::publish_diagnostics()
{
  diagnostic_updater::DiagnosticStatusWrapper status;
  status.name = "/reasoning/kb/people_facts";
  status.summary(
    diagnostic_msgs::msg::DiagnosticStatus::OK,
    "Publishing new detected humans to knowledge base");
  status.add("Current lifecycle state", this->get_current_state().label());

  diagnostic_msgs::msg::DiagnosticArray msg;
  msg.header.stamp = this->get_clock()->now();
  msg.status.push_back(status);
  diagnostics_pub_->publish(msg);
}

}  // namespace people_facts
