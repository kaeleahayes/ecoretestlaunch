# Copyright 2024 Husarion sp. z o.o.
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
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)

import launch_ros.actions

from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ParseMultiRobotPose


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    mecanum = LaunchConfiguration("mecanum").perform(context)
    world = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless").perform(context)
    use_gpu = LaunchConfiguration("use_gpu").perform(context)
    x = LaunchConfiguration("x", default="0.0").perform(context)
    y = LaunchConfiguration("y", default="2.0").perform(context)
    z = LaunchConfiguration("z", default="0.0").perform(context)
    roll = LaunchConfiguration("roll", default="0.0").perform(context)
    pitch = LaunchConfiguration("pitch", default="0.0").perform(context)
    yaw = LaunchConfiguration("yaw", default="0.0").perform(context)

    gz_args = f"--headless-rendering -s -v 4 -r {world}" if eval(headless) else f"-r {world}"

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gz_args": gz_args,
            "on_exit_shutdown": "True",
        }.items(),
    )

    robots_list = ParseMultiRobotPose("robots").value()
    if len(robots_list) == 0:
        robots_list = {
            namespace: {
                "x": x,
                "y": y,
                "z": z,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
            }
        }

    spawn_group = []
    for robot_name in robots_list:
        init_pose = robots_list[robot_name]
        group = GroupAction(
            [
                LogInfo(
                    msg=[
                        "Launching namespace=",
                        robot_name,
                        " init_pose=",
                        str(init_pose),
                    ]
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution(
                            [
                                FindPackageShare("rosbot_gazebo"),
                                "launch",
                                "spawn.launch.py",
                            ]
                        )
                    ),
                    launch_arguments={
                        "mecanum": mecanum,
                        "use_sim": "True",
                        "use_gpu": use_gpu,
                        "simulation_engine": "ignition-gazebo",
                        "namespace": TextSubstitution(text=robot_name),
                        "x": TextSubstitution(text=str(init_pose["x"])),
                        "y": TextSubstitution(text=str(init_pose["y"])),
                        "z": TextSubstitution(text=str(init_pose["z"])),
                        "roll": TextSubstitution(text=str(init_pose["roll"])),
                        "pitch": TextSubstitution(text=str(init_pose["pitch"])),
                        "yaw": TextSubstitution(text=str(init_pose["yaw"])),
                    }.items(),
                ),
            ]
        )
        spawn_group.append(group)

    return [gz_sim, *spawn_group]


def generate_launch_description():
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Namespace for all topics and tfs",
    )

    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="False",
        description=(
            "Whether to use mecanum drive controller (otherwise diff drive controller is used)"
        ),
    )

    world_package = FindPackageShare("ecore_launch")
    world_file = PathJoinSubstitution([world_package, "ecore2.sdf"])
    declare_world_arg = DeclareLaunchArgument(
        "world", default_value=world_file, description="SDF world file"
    )

    declare_headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="False",
        description="Run Gazebo Ignition in the headless mode",
        choices=["True", "False"],
    )

    declare_use_gpu_arg = DeclareLaunchArgument(
        "use_gpu",
        default_value="True",
        description="Whether GPU acceleration is used",
    )

    declare_robots_arg = DeclareLaunchArgument(
        "robots",
        default_value="",
        description=(
            "Spawning multiple robots at positions with yaw orientations e. g. robots:='robot1={x:"
            " 0.0, y: -1.0}; robot2={x: 1.0, y: -1.0}; robot3={x: 2.0, y: -1.0}; robot4={x: 3.0,"
            " y: -1.0}'"
        ),
    )
    
    
    robot_descrip_pub = launch_ros.actions.Node(
    	name='robot_state_publisher',
    	package='robot_state_publisher',
    	executable='robot_state_publisher',
    	parameters=[{'robot_description': open('src/rosbot_description/urdf/rosbot.urdf').read()}]
    )
    
    joint_descrip = launch_ros.actions.Node(
    	package='joint_state_publisher',
    	executable='joint_state_publisher',
    	arguments='src/rosbot_description/urdf/rosbot.urdf'
    )

    return LaunchDescription(
        [
            declare_namespace_arg,
            declare_mecanum_arg,
            declare_world_arg,
            declare_headless_arg,
            declare_use_gpu_arg,
            robot_descrip_pub,
            joint_descrip,
            declare_robots_arg,
            # Sets use_sim_time for all nodes started below
            # (doesn't work for nodes started from ignition gazebo)
            SetParameter(name="use_sim_time", value=True),
            OpaqueFunction(function=launch_setup),
        ]
    )
