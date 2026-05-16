"""
Consensus experiment launch — Step 7 (per-agent shaper).

Identical to consensus.launch.py except shaper executable. Use this to
A/B compare Step 7 vs Step 6 results on the same workload.

What this launches:
  - 1 temporis_per_agent_shaper node (N internal client queues, 1 router)
  - N consensus_agent nodes (unchanged from Step 6)
  - Each agent publishes its scalar state to /temporis/raw/states
  - Shaper routes each message into the sender's own queue, applies
    Temporis latency, and re-publishes to /fleet/states
  - Each agent applies consensus update on received remote states

Validation goal:
  Compare against Step 6 (single shaper) on:
    * Startup transient duration (expect: shorter)
    * recv_count spread across agents (expect: tighter)
    * Final-state drift between early/late agents (expect: smaller)
    * Convergence step (expect: similar — Temporis core unchanged)

Usage:
  ros2 launch temporis_ros2 consensus_per_agent.launch.py num_agents:=50 topology:=random_k topology_k:=5
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    num_agents_arg = DeclareLaunchArgument('num_agents', default_value='10')
    rate_hz_arg = DeclareLaunchArgument(
        'rate_hz', default_value='1.0',
        description='Consensus step rate. Use 1Hz to match standalone dt=1.0')
    alpha_arg = DeclareLaunchArgument('alpha', default_value='0.1')
    max_steps_arg = DeclareLaunchArgument('max_steps', default_value='6387')
    init_seed_arg = DeclareLaunchArgument('init_seed', default_value='42')
    output_dir_arg = DeclareLaunchArgument(
        'output_dir', default_value='/tmp/temporis_ros2_consensus_step7',
        description='Directory for per-agent CSV output. '
                    'Use a different dir from Step 6 to avoid mixing logs.')

    # Topology — exposed so we can A/B test ring vs random_k easily.
    topology_arg = DeclareLaunchArgument(
        'topology', default_value='random_k',
        description='all_to_all | ring | grid | random_k')
    topology_k_arg = DeclareLaunchArgument(
        'topology_k', default_value='5',
        description='Used when topology=random_k')

    def launch_setup(context, *args, **kwargs):
        num_agents = int(LaunchConfiguration('num_agents').perform(context))
        rate_hz = float(LaunchConfiguration('rate_hz').perform(context))
        alpha = float(LaunchConfiguration('alpha').perform(context))
        max_steps = int(LaunchConfiguration('max_steps').perform(context))
        init_seed = int(LaunchConfiguration('init_seed').perform(context))
        output_dir = LaunchConfiguration('output_dir').perform(context)
        topology = LaunchConfiguration('topology').perform(context)
        topology_k = int(LaunchConfiguration('topology_k').perform(context))

        os.makedirs(output_dir, exist_ok=True)

        nodes = []

        # ---- Per-agent shaper (Step 7) ----
        nodes.append(Node(
            package='temporis_ros2',
            executable='temporis_per_agent_shaper',
            name='temporis_per_agent_shaper',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('temporis_ros2'),
                    'config',
                    'temporis_shaper.yaml'  # same yaml works
                ]),
                {
                    'num_agents': num_agents,
                    'topology': topology,
                    'topology_k': topology_k,
                }
            ]
        ))

        # ---- Consensus agents (unchanged from Step 6) ----
        for i in range(num_agents):
            csv_path = os.path.join(output_dir, f'agent_{i}.csv')
            nodes.append(Node(
                package='temporis_ros2',
                executable='consensus_agent',
                namespace=f'/robot_{i}',
                name='consensus_agent',
                output='screen',
                parameters=[{
                    'agent_id': i,
                    'num_agents': num_agents,
                    'alpha': alpha,
                    'rate_hz': rate_hz,
                    'max_steps': max_steps,
                    'init_seed': init_seed,
                    'pub_topic': '/temporis/raw/states',
                    'sub_topic': '/fleet/states',
                    'output_csv': csv_path,
                }]
            ))

        return nodes

    return LaunchDescription([
        num_agents_arg,
        rate_hz_arg,
        alpha_arg,
        max_steps_arg,
        init_seed_arg,
        output_dir_arg,
        topology_arg,
        topology_k_arg,
        OpaqueFunction(function=launch_setup),
    ])