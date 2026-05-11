"""
Simulation launch file with Temporis network shaper.

Launches N agents with topic remapping so that their output goes
through the Temporis shaper, which adds realistic latency before
re-publishing on the original topic.

Agent code is unchanged — the only difference from production is
the remap and the presence of the shaper node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    num_agents = 10  # adjust as needed
    agent_topic = '/fleet/states'
    shaper_input = '/temporis/raw/states'

    nodes = []

    # ---- Temporis shaper node ----
    nodes.append(Node(
        package='temporis_ros2',
        executable='temporis_shaper',
        name='temporis_shaper',
        parameters=[{
            'input_topic': shaper_input,
            'output_topic': agent_topic,
            'msg_type': 'std_msgs/msg/ByteMultiArray',
            'model': 'ZENOH_QUEUE',
            'topology': 'random_k',
            'topology_k': 5,
            'num_agents': num_agents,
            'namespace_prefix': 'robot_',
            'enabled': True,
            'publish_diagnostics': True,
            # Calibrated from real Zenoh all-to-all benchmark
            'client_bandwidth': 1000.0,
            'packet_size': 1.0,
            'propagation_client_router': 0.000150,
            'propagation_router_subscriber': 0.000150,
            'router_base_cost': 0.000108,
            'router_per_sub_cost': 0.0000027,
            'bandwidth_logstd': 0.0,
            'bandwidth_rho': 0.0,
            'seed': 42,
        }],
        output='screen',
    ))

    # ---- Agent nodes ----
    for i in range(num_agents):
        nodes.append(Node(
            package='my_agent',          # replace with your agent package
            executable='agent',          # replace with your agent executable
            namespace=f'/robot_{i}',
            name='agent',
            remappings=[
                # Agent publishes to /fleet/states, but remap sends it
                # to /temporis/raw/states. Shaper picks it up, delays,
                # and re-publishes on /fleet/states.
                (agent_topic, shaper_input),
            ],
            parameters=[{
                'agent_id': i,
                'num_agents': num_agents,
            }],
            output='screen',
        ))

    return LaunchDescription(nodes)
