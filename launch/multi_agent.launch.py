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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------

    num_agents_arg = DeclareLaunchArgument(
        'num_agents',
        default_value='5'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='QUEUE'
    )

    enabled_arg = DeclareLaunchArgument(
        'enabled',
        default_value='true'
    )

    num_agents = LaunchConfiguration('num_agents')
    model = LaunchConfiguration('model')
    enabled = LaunchConfiguration('enabled')

    def launch_setup(context, *args, **kwargs):

        num_agents_value = int(num_agents.perform(context))
        model_value = model.perform(context)
        enabled_value = enabled.perform(context).lower() == 'true'

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
                    ]),
                    {
                        'model': model_value,
                        'enabled': enabled_value,
                        'num_agents': num_agents_value,
                    }
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
        num_agents_arg,
        model_arg,
        enabled_arg,
        OpaqueFunction(function=launch_setup)
    ])
