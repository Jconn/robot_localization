# Copyright 2018 Open Source Robotics Foundation, Inc.
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

""" ekf launch.py """

import launch
from launch import LaunchDescription
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    parameters_file_dir = pathlib.Path(os.path.join(get_package_share_directory('robot_localization'), 'params'))
    print(f"params file dir is {parameters_file_dir}")
    parameters_file_path = parameters_file_dir / 'ekf.yaml'
    print(f"params file path is {parameters_file_path}")
    os.environ['FILE_PATH'] = str(parameters_file_dir)

         #*****test_ekf_localization_node_interfaces.test***** 
    se_node = launch_ros.actions.Node(
            package='robot_localization', node_executable='se_node', node_name='se_node',
            output='screen',
            parameters=[
                parameters_file_path,
                str(parameters_file_path),
                [EnvironmentVariable(name='FILE_PATH'), os.sep, 'ekf.yaml']
                ]
            )


    return LaunchDescription([
        se_node
        ])
