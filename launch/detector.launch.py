# Copyright (c) 2023 José Miguel Guerrero Hernández
#
# Licensed under the Attribution-ShareAlike 4.0 International (CC BY-SA 4.0) License;
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://creativecommons.org/licenses/by-sa/4.0/
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='small_objects_detector',
            namespace='',
            executable='detector',
            output='both',
            emulate_tty=True,
            parameters=[{
                "plane_size": 5000,     # minimun number of points in plane
                "downsampling": 0.02,   # distance between points in meters
            }],
            # Use topics from robot
            remappings=[
                ('/pointcloud_in', '/camera/depth_registered/points'),
            ],
        )
    ])
