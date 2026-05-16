"""
Consensus experiment launch — ROS2 port of consensus_demo.

What this launches:
  - 1 temporis_shaper node (with ZENOH_QUEUE or QUEUE model)
  - N consensus_agent nodes
  - Each agent publishes its scalar state to /temporis/raw/states
  - Shaper delays + republishes to /fleet/states
  - Each agent applies consensus update on received remote states

Validation:
  Compare convergence step (when local_var < 1e-6) against standalone
  consensus_demo with the same parameters.

Usage:
  ros2 launch temporis_ros2 consensus.launch.py num_agents:=10
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    num_agents_arg = DeclareLaunchArgument(
        'num_agents', default_value='10')
    rate_hz_arg = DeclareLaunchArgument(
        'rate_hz', default_value='1.0',
        description='Consensus step rate. Use 1Hz to match standalone dt=1.0')
    alpha_arg = DeclareLaunchArgument(
        'alpha', default_value='0.1')
    max_steps_arg = DeclareLaunchArgument(
        'max_steps', default_value='6387')
    init_seed_arg = DeclareLaunchArgument(
        'init_seed', default_value='42')
    output_dir_arg = DeclareLaunchArgument(
        'output_dir', default_value='/tmp/temporis_ros2_consensus',
        description='Directory for per-agent CSV output')

    def launch_setup(context, *args, **kwargs):
        num_agents = int(LaunchConfiguration('num_agents').perform(context))
        rate_hz = float(LaunchConfiguration('rate_hz').perform(context))
        alpha = float(LaunchConfiguration('alpha').perform(context))
        max_steps = int(LaunchConfiguration('max_steps').perform(context))
        init_seed = int(LaunchConfiguration('init_seed').perform(context))
        output_dir = LaunchConfiguration('output_dir').perform(context)

        os.makedirs(output_dir, exist_ok=True)

        nodes = []

        # ---- Temporis shaper ----
        nodes.append(Node(
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
                    'num_agents': num_agents,
                }
            ]
        ))

        # ---- Consensus agents ----
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
        OpaqueFunction(function=launch_setup),
    ])