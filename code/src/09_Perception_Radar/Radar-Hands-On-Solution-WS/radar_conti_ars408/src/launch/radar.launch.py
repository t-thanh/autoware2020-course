# Copyright [2020] [Daniel Peter, peter@fh-aachen.de, Fachhochschule Aachen]
#
#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(package='radar_conti_ars408', node_executable='radar_conti_ars408_composition', node_name='radar_conti_ars408', output='screen')
    ])
