// Copyright 2022 PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PAL Robotics S.L. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <knowledge_core/Query.h>
#include <ros/ros.h>

#include <chrono>
#include <map>
#include <thread>
#include <vector>

#include "hri_msgs/IdsList.h"
#include "json.hpp"
using json = nlohmann::json;

using namespace std;
using namespace ros;

// waiting time for the libhri callback to process their inputs
#define WAIT(X) std::this_thread::sleep_for(std::chrono::milliseconds(X))

class PeopleFactsTest : public ::testing::Test {
   public:
    PeopleFactsTest()
        : spinner(1),
          kb_query(nh.serviceClient<knowledge_core::Query>("/kb/query"))

    {
        //    ros::service::waitForService("/kb/query", ros::Duration(1));
    }

    ~PeopleFactsTest() {}

    void checkKB(vector<string> patterns,
                 vector<map<string, string>> expected_result) {
        query.request.patterns = patterns;
        kb_query.call(query);
        json j = json::parse(query.response.json);

        auto res = j.get<vector<map<string, string>>>();

        ASSERT_THAT(res, testing::UnorderedElementsAreArray(expected_result));
    }

    void SetUp() override {
        spinner.start();
        // wait for the people_facts and knowledge_core nodes to be up
        WAIT(500);
    }
    void TearDown() override { spinner.stop(); }

    ServiceClient kb_query;
    knowledge_core::Query query;

    ros::NodeHandle nh;

   private:
    ros::AsyncSpinner spinner;
};

TEST_F(PeopleFactsTest, BasicFacts) {
    Publisher known_pub =
        nh.advertise<hri_msgs::IdsList>("/humans/persons/known", 1);
    Publisher tracked_pub =
        nh.advertise<hri_msgs::IdsList>("/humans/persons/tracked", 1);

    auto ids = hri_msgs::IdsList();
    ids.ids = {"p1"};
    known_pub.publish(ids);
    tracked_pub.publish(ids);
    WAIT(30);

    checkKB({"?p rdf:type Human"}, {{{"p", "p1"}}});
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::Time::init();  // needed for ros::Time::now()
    ros::init(argc, argv, "test_people_facts");
    return RUN_ALL_TESTS();
}

