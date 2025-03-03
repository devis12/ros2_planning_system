# Copyright 2019 Intelligent Robotics Lab
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')
    
    # log_namespace = LogInfo(
    #     msg=[
    #         TextSubstitution(text="In planner_launch, using namespace: "),
    #         LaunchConfiguration('namespace')
    #     ]
    # )
    
    # log_params_file = LogInfo(
    #     msg=[
    #         TextSubstitution(text="In planner_launch, using parameter file: "),
    #         LaunchConfiguration('params_file')
    #     ]
    # )

    # Specify the actions
    planner_cmd = Node(
        package='plansys2_planner',
        executable='planner_node',
        name='planner',
        namespace=namespace,
        output='screen',
        parameters=[params_file])

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    # ld.add_action(log_params_file) 
    # ld.add_action(log_namespace)

    # Declare the launch options
    ld.add_action(planner_cmd)

    return ld
