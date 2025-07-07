# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg = 'people_facts'

    people_facts_node = LifecycleNode(
        package='people_facts', executable='people_facts', namespace='',
        name='people_facts')

    configure_event = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(people_facts_node),
        transition_id=Transition.TRANSITION_CONFIGURE))

    activate_event = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=people_facts_node, goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(people_facts_node),
            transition_id=Transition.TRANSITION_ACTIVATE))]))

    people_facts_analyzer = Node(
        package='diagnostic_aggregator',
        executable='add_analyzer',
        namespace=pkg,
        output='screen',
        emulate_tty=True,
        parameters=[
            os.path.join(get_package_share_directory(pkg), 'config', f'{pkg}_analyzers.yaml')],
    )

    return LaunchDescription([
        people_facts_node,
        configure_event,
        activate_event,
        people_facts_analyzer
    ])
