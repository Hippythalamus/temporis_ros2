"""
Production launch file — no Temporis shaper.

Agents communicate directly on /fleet/states with no added latency.
Same agent code as sim launch, zero changes needed.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    num_agents = 10

    nodes = []

    for i in range(num_agents):
        nodes.append(Node(
            package='my_agent',          # replace with your agent package
            executable='agent',          # replace with your agent executable
            namespace=f'/robot_{i}',
            name='agent',
            # No remapping — agents talk directly on /fleet/states
            parameters=[{
                'agent_id': i,
                'num_agents': num_agents,
            }],
            output='screen',
        ))

    return LaunchDescription(nodes)
