"""
Minimal multi-agent validation launch for Temporis.

What this launches:
  - 1 temporis_shaper node
  - N test agents
  - Agents publish to /temporis/raw/states
  - Shaper delays + republishes to /fleet/states
  - Agents subscribe to /fleet/states

Validation:
  enabled=true  -> latency ~= configured delay
  enabled=false -> latency ~= DDS baseline (<1-2ms local)

Usage:
  ros2 launch temporis_ros2 multi_agent_test.launch.py

Optional overrides:
  ros2 launch temporis_ros2 multi_agent_test.launch.py \
      num_agents:=5 \
      model:=QUEUE \
      enabled:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------

    config_path = os.path.join(
        get_package_share_directory('temporis_ros2'),
        'config',
        'temporis_shaper.yaml'
    )

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    num_agents_value = config['temporis_shaper']['ros__parameters']['num_agents']
    enable_value = config['temporis_shaper']['ros__parameters']['enabled']
    model = config['temporis_shaper']['ros__parameters']['model']

    def launch_setup(context, *args, **kwargs):

        shaper_input = '/temporis/raw/states'
        fleet_topic = '/fleet/states'

        nodes = []

        # --------------------------------------------------------------
        # Temporis shaper
        # --------------------------------------------------------------

        nodes.append(
            Node(
                package='temporis_ros2',
                executable='temporis_shaper',
                name='temporis_shaper',
                output='screen',

                parameters=[
                    PathJoinSubstitution([
                        FindPackageShare('temporis_ros2'),
                        'config',
                        'temporis_shaper.yaml'
                    ])
                ]
            )
        )



        # --------------------------------------------------------------
        # Test agents
        # --------------------------------------------------------------

        for i in range(num_agents_value):

            nodes.append(
                Node(
                    package='temporis_ros2',
                    executable='test_agent',

                    namespace=f'/robot_{i}',
                    name='test_agent',

                    output='screen',

                    parameters=[{
                        'agent_id': i,
                        'num_agents': num_agents_value,
                        'rate_hz': 10.0,

                        'pub_topic': shaper_input,
                        'sub_topic': fleet_topic,
                    }]
                )
            )

        return nodes

    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
