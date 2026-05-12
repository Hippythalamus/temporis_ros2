"""
Production launch file — direct DDS communication.

No Temporis shaper.
Agents communicate directly on /fleet/states.

Used as baseline for latency comparison.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # --------------------------------------------------------------
    # Launch arguments
    # --------------------------------------------------------------

    num_agents_arg = DeclareLaunchArgument(
        'num_agents',
        default_value='5'
    )

    rate_hz_arg = DeclareLaunchArgument(
        'rate_hz',
        default_value='10.0'
    )

    num_agents = LaunchConfiguration('num_agents')
    rate_hz = LaunchConfiguration('rate_hz')

    # --------------------------------------------------------------
    # Runtime launch setup
    # --------------------------------------------------------------

    def launch_setup(context, *args, **kwargs):

        num_agents_value = int(num_agents.perform(context))
        rate_hz_value = float(rate_hz.perform(context))

        fleet_topic = '/fleet/states'

        nodes = []

        # ----------------------------------------------------------
        # Test agents
        # ----------------------------------------------------------

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
                        'rate_hz': rate_hz_value,

                        # Direct DDS communication
                        'pub_topic': fleet_topic,
                        'sub_topic': fleet_topic,
                    }]
                )
            )

        return nodes

    return LaunchDescription([
        num_agents_arg,
        rate_hz_arg,
        OpaqueFunction(function=launch_setup)
    ])