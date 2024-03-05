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

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <std_msgs/msg/string.h>
#include <hri_msgs/msg/ids_list.h>

#include <chrono>
#include <map>
#include <thread>
#include <vector>

#include <lifecycle_msgs/srv/change_state.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <hri/types.hpp>
#include <kb_msgs/srv/query.hpp>


#include "people_facts/node_people_facts.hpp"

using namespace std::chrono_literals;

const int kMaxNumberPeople = 2;
const int kNodeCycleMs = 100;

struct Person
{
  hri::EngagementLevel engagement_level;
};

MATCHER(RevisionEq, "") {
  auto req = std::get<0>(arg);
  std::string method = req->method;
  std::vector<std::string> statements = req->statements;

  auto expect = std::get<1>(arg);
  std::string expected_method = expect.first;
  std::vector<std::string> expected_statements = expect.second;

  auto UnorderedMatch =
    testing::Matches(testing::UnorderedElementsAreArray(expected_statements));

  return method == expected_method && UnorderedMatch(statements);
}

class NodePeopleFactsTestBase : public ::testing::Test
{
protected:
  explicit NodePeopleFactsTestBase(
    const std::vector<rclcpp::Parameter> & parameters)
  : parameters_(parameters) {}

  void SetUp() override
  {
    people_facts_node_ = std::make_shared<people_facts::NodePeopleFacts>();
    people_facts_node_->set_parameter({"use_sim_time", true});

    people_facts_executor_ =
      rclcpp::executors::SingleThreadedExecutor::make_shared();
    people_facts_executor_->add_node(
      people_facts_node_->get_node_base_interface());

    tester_node_ = rclcpp::Node::make_shared("tester_node");
    tester_executor_ = rclcpp::executors::SingleThreadedExecutor::make_shared();
    tester_executor_->add_node(tester_node_);

    time_ = tester_node_->get_clock()->now();
    clock_pub_ =
      tester_node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());

    //////////////////////////////////////////////////////
    // mock-up hri_person_manager person publishing
    persons_tracked_pub_ =
      tester_node_->create_publisher<hri_msgs::msg::IdsList>(
      "/humans/persons/tracked", 1);
    persons_known_pub_ = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
      "/humans/persons/known", 1);

    for (int id = 0; id < kMaxNumberPeople; ++id) {
      person_engagement_pubs_[id] =
        tester_node_->create_publisher<hri_msgs::msg::EngagementLevel>(
        "/humans/persons/person" + std::to_string(id) +
        "/engagement_status",
        1);
    }

    //////////////////////////////////////////////////////
    // mock-up the knowledge base 'Revise' service
    kb_revise_srv_ = tester_node_->create_service<kb_msgs::srv::Revise>(
      "kb/revise", [&revision_reqs_ = revision_reqs_](
        const std::shared_ptr<kb_msgs::srv::Revise::Request> req,
        std::shared_ptr<kb_msgs::srv::Revise::Response> res) {
        std::cout << "emitted revise method: " << req->method << std::endl;
        for (auto s : req->statements) {
          std::cout << " - " << s << std::endl;
        }

        revision_reqs_.push_back(req);
        res->success = true;
      });

    //////////////////////////////////////////////////////

    auto parameters_client =
      tester_node_->create_client<rcl_interfaces::srv::SetParameters>(
      "/people_facts/set_parameters");
    auto set_parameters_req =
      std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    for (auto const & parameter : parameters_) {
      set_parameters_req->parameters.emplace_back(parameter.to_parameter_msg());
    }
    auto set_parameters_future =
      parameters_client->async_send_request(set_parameters_req);
    spin();
    ASSERT_THAT(
      set_parameters_future.get()->results,
      testing::Each(
        testing::Field(
          &rcl_interfaces::msg::SetParametersResult::successful,
          testing::Eq(true))));

    ////////////////////////////////////////////////////////////////////////
    // Configure and start the people_facts node via lifecycle transitions

    auto change_state_client =
      tester_node_->create_client<lifecycle_msgs::srv::ChangeState>(
      "/people_facts/change_state");
    auto change_state_req =
      std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    change_state_req->transition.id =
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
    auto change_state_future =
      change_state_client->async_send_request(change_state_req);
    spin();
    ASSERT_TRUE(change_state_future.get()->success);
    change_state_req->transition.id =
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
    change_state_future =
      change_state_client->async_send_request(change_state_req);
    spin();
    ASSERT_TRUE(change_state_future.get()->success);
  }

  void TearDown() override
  {
    persons_tracked_pub_->publish(hri_msgs::msg::IdsList());
    persons_known_pub_->publish(hri_msgs::msg::IdsList());
    spin();

    kb_revise_srv_.reset();
    person_engagement_pubs_.clear();
    persons_tracked_pub_.reset();
    persons_known_pub_.reset();

    revision_reqs_.clear();
    people_facts_node_.reset();
    people_facts_executor_.reset();
    tester_node_.reset();
    tester_executor_.reset();
  }

  void spin(std::chrono::nanoseconds timeout = 10s)
  {
    people_facts_executor_->spin_all(timeout);

    time_ += rclcpp::Duration(std::chrono::milliseconds(kNodeCycleMs + 1));
    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = time_;
    clock_pub_->publish(clock_msg);

    people_facts_executor_->spin_all(timeout);
    tester_executor_->spin_all(timeout);
  }

  void test(
    const std::vector<Person> & people, std::vector<std::pair<std::string,
    std::vector<std::string>>> expected_result)
  {
    ASSERT_LE(static_cast<int>(people.size()), kMaxNumberPeople)
      << "Unsupported number of people";

    hri_msgs::msg::IdsList person_ids_msg;

    for (size_t id = 0; id < static_cast<size_t>(people.size()); ++id) {
      person_ids_msg.ids.emplace_back("person" + std::to_string(id));
    }

    persons_tracked_pub_->publish(person_ids_msg);
    persons_known_pub_->publish(person_ids_msg);
    spin();

    EXPECT_THAT(
      revision_reqs_,
      testing::UnorderedPointwise(RevisionEq(), expected_result));
  }

  rclcpp::Publisher<hri_msgs::msg::IdsList>::SharedPtr persons_tracked_pub_;
  rclcpp::Publisher<hri_msgs::msg::IdsList>::SharedPtr persons_known_pub_;
  std::map<int, rclcpp::Publisher<hri_msgs::msg::EngagementLevel>::SharedPtr>
  person_engagement_pubs_;
  std::vector<std::shared_ptr<kb_msgs::srv::Revise::Request>> revision_reqs_;
  rclcpp::Service<kb_msgs::srv::Revise>::SharedPtr kb_revise_srv_;

private:
  std::shared_ptr<people_facts::NodePeopleFacts> people_facts_node_;
  rclcpp::Node::SharedPtr tester_node_;
  rclcpp::Executor::SharedPtr people_facts_executor_;
  rclcpp::Executor::SharedPtr tester_executor_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Time time_;
  std::vector<rclcpp::Parameter> parameters_;
};

//////////////////////////////////////////////////////////////////////////////
//
//                          Start of actual tests
//
//////////////////////////////////////////////////////////////////////////////

// with default parameters
class NodePeopleFactsTestDefault : public NodePeopleFactsTestBase
{
protected:
  NodePeopleFactsTestDefault()
  : NodePeopleFactsTestBase({}) {}
};

TEST_F(NodePeopleFactsTestDefault, OnePerson)
{
  std::vector<Person> people(1);

  std::vector<std::pair<std::string, std::vector<std::string>>> expected_result{
    {"update", {"person0 rdf:type Human"}},
    {"update", {"person0 currentlyTracked true"}},
    {"retract",
      {"person0 hasEngagementLevel engaged",
        "person0 hasEngagementLevel engaging",
        "person0 hasEngagementLevel disengaging",
        "person0 hasEngagementLevel disengaged", }},
  };

  test(people, expected_result);
};

TEST_F(NodePeopleFactsTestDefault, TwoPersons)
{
  std::vector<Person> people(2);

  std::vector<std::pair<std::string, std::vector<std::string>>> expected_result{
    // person 1
    {"update", {"person0 rdf:type Human"}},
    {"update", {"person0 currentlyTracked true"}},
    // person 2
    {"update", {"person1 rdf:type Human"}},
    {"update", {"person1 currentlyTracked true"}},
    {"retract",
      {"person0 hasEngagementLevel engaged",
        "person0 hasEngagementLevel engaging",
        "person0 hasEngagementLevel disengaging",
        "person0 hasEngagementLevel disengaged",
        "person1 hasEngagementLevel engaged",
        "person1 hasEngagementLevel engaging",
        "person1 hasEngagementLevel disengaging",
        "person1 hasEngagementLevel disengaged", }},
  };

  test(people, expected_result);
};

TEST_F(NodePeopleFactsTestDefault, OnePersonEngaged)
{
  std::vector<Person> people(1);

  std::vector<std::pair<std::string, std::vector<std::string>>> expected_result{
    {"update", {"person0 rdf:type Human"}},
    {"update", {"person0 currentlyTracked true"}},
    {"retract",
      {"person0 hasEngagementLevel engaged",
        "person0 hasEngagementLevel engaging",
        "person0 hasEngagementLevel disengaging",
        "person0 hasEngagementLevel disengaged", }},
  };

  test(people, expected_result);

  // engaged...
  hri_msgs::msg::EngagementLevel msg;
  msg.level = hri_msgs::msg::EngagementLevel::ENGAGED;

  revision_reqs_.clear();
  expected_result = {
    {"update", {"person0 hasEngagementLevel engaged"}},
  };

  person_engagement_pubs_[0]->publish(msg);
  spin();

  EXPECT_THAT(
    revision_reqs_,
    testing::UnorderedPointwise(RevisionEq(), expected_result));

  // disengaged...
  msg.level = hri_msgs::msg::EngagementLevel::DISENGAGED;

  revision_reqs_.clear();
  expected_result = {
    {"update", {"person0 hasEngagementLevel disengaged"}},
  };

  person_engagement_pubs_[0]->publish(msg);
  spin();

  EXPECT_THAT(
    revision_reqs_,
    testing::UnorderedPointwise(RevisionEq(), expected_result));
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
