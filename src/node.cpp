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

#include <hri/hri.h>
#include <hri/person.h>
#include <knowledge_core/Revise.h>
#include <ros/ros.h>

#include <functional>

using namespace std;
using namespace ros;
using namespace hri;

class PeopleFacts {
   public:
    PeopleFacts(NodeHandle nh)
        : kb_revise(nh.serviceClient<knowledge_core::Revise>("kb/revise")) {
        // bind the callback to this class' 'onPerson' method.
        hri_listener.onPerson(
            bind(&PeopleFacts::onPerson, this, std::placeholders::_1));
        hri_listener.onTrackedPerson(
            bind(&PeopleFacts::onTrackedPerson, this, std::placeholders::_1));
        hri_listener.onTrackedPersonLost(bind(&PeopleFacts::onTrackedPersonLost,
                                              this, std::placeholders::_1));
    }

    void update() {
        vector<string> stmts_to_add;
        vector<string> stmts_to_remove;

        for (auto person : hri_listener.getTrackedPersons()) {
            auto p = person.second.lock();

            auto id = p->id();

            EngagementLevel engagement = hri::UNKNOWN;
            if (p->engagement_status()) {
                engagement = *(p->engagement_status());
            }
            string predicate(" hasEngagementLevel ");
            if (needsUpdate(id, predicate,
                            hash<EngagementLevel>{}(engagement))) {
                ROS_INFO_STREAM("Revising value of predicate <"
                                << predicate << "> for " << id);
                switch (engagement) {
                    case hri::UNKNOWN:
                        stmts_to_remove.push_back(id + predicate + "engaged");
                        stmts_to_remove.push_back(id + predicate +
                                                  "disengaged");
                        stmts_to_remove.push_back(id + predicate + "engaging");
                        stmts_to_remove.push_back(id + predicate +
                                                  "disengaging");
                        break;
                    case hri::ENGAGED:
                        stmts_to_add.push_back(id + predicate + "engaged");
                        break;
                    case hri::ENGAGING:
                        stmts_to_add.push_back(id + predicate + "engaging");
                        break;
                    case hri::DISENGAGING:
                        stmts_to_add.push_back(id + predicate + "disengaging");
                        break;
                    case hri::DISENGAGED:
                        stmts_to_add.push_back(id + predicate + "disengaged");
                        break;
                }
            }
        }

        if (!stmts_to_add.empty()) {
            knowledge_core::Revise revise;
            revise.request.method = knowledge_core::ReviseRequest::UPDATE;
            revise.request.statements = stmts_to_add;

            kb_revise.call(revise);
        }
        if (!stmts_to_remove.empty()) {
            knowledge_core::Revise revise;
            revise.request.method = knowledge_core::ReviseRequest::RETRACT;
            revise.request.statements = stmts_to_remove;

            kb_revise.call(revise);
        }
    }

    void onPerson(PersonWeakConstPtr person_weak) {
        if (auto person = person_weak.lock()) {
            ROS_INFO_STREAM("New person detected. ID: " << person->id());

            knowledge_core::Revise revise;
            revise.request.method = knowledge_core::ReviseRequest::UPDATE;
            revise.request.statements = {person->id() + " rdf:type Human"};

            kb_revise.call(revise);
        }
    }

    void onTrackedPerson(PersonWeakConstPtr person_weak) {
        if (auto person = person_weak.lock()) {
            knowledge_core::Revise revise;
            revise.request.method = knowledge_core::ReviseRequest::UPDATE;
            revise.request.statements = {person->id() +
                                         " currentlyTracked true"};

            kb_revise.call(revise);
        }
    }
    void onTrackedPersonLost(ID id) {
        knowledge_core::Revise revise;
        revise.request.method = knowledge_core::ReviseRequest::RETRACT;
        revise.request.statements = {id + " currentlyTracked false"};

        kb_revise.call(revise);
    }

   private:
    /** returns true if a specific 'key' has changed and, accordingly, the
     * knowledge base needs update.
     *
     * Attention: *not* idempotent. Calling `needsUpdate` also updates the
     * internal keystore with the new hash value.
     */
    bool needsUpdate(ID id, string key, size_t hash_value) {
        ROS_DEBUG_STREAM("Checking value of " << key << " for " << id
                                              << " (hash:" << hash_value
                                              << ")");
        if (_previous_state.count(id) == 0) {
            _previous_state[id][key] = hash_value;
            return true;
        }
        if (_previous_state[id].count(key) == 0) {
            _previous_state[id][key] = hash_value;
            return true;
        }

        if (_previous_state[id][key] != hash_value) {
            _previous_state[id][key] = hash_value;
            return true;
        }

        return false;
    }

    ServiceClient kb_revise;
    HRIListener hri_listener;

    map<ID, map<string, size_t>> _previous_state;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "people_facts");

    ros::NodeHandle nh;

    PeopleFacts pf(nh);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();

        pf.update();
    }

    cout << "Bye bye!" << endl;

    return 0;
}
