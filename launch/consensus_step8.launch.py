"""
Step 8 v2 launch — adds qos_reliability + qos_depth parameters.

Three validation runs to do:

  # Run M (MODEL): shaper enabled, doesn't matter what QoS — shaper isolates
  ros2 launch temporis_ros2 consensus_step8_v2.launch.py \
      num_agents:=50 topology:=random_k topology_k:=5 \
      enabled:=true \
      output_dir:=/tmp/step8v2_model

  # Run R-RELIABLE: real Zenoh, reliable+10 (= original Step 8 result)
  ros2 launch temporis_ros2 consensus_step8_v2.launch.py \
      num_agents:=50 topology:=random_k topology_k:=5 \
      enabled:=false qos_reliability:=reliable qos_depth:=10 \
      output_dir:=/tmp/step8v2_real_reliable

  # Run R-BESTEFFORT: real Zenoh, best_effort+1 (= transport-only)
  ros2 launch temporis_ros2 consensus_step8_v2.launch.py \
      num_agents:=50 topology:=random_k topology_k:=5 \
      enabled:=false qos_reliability:=best_effort qos_depth:=1 \
      output_dir:=/tmp/step8v2_real_besteffort

Then:
  python3 step8_compare.py /tmp/step8v2_model /tmp/step8v2_real_besteffort
  python3 step8_compare.py /tmp/step8v2_model /tmp/step8v2_real_reliable
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    num_agents_arg = DeclareLaunchArgument('num_agents', default_value='10')
    rate_hz_arg = DeclareLaunchArgument('rate_hz', default_value='1.0')
    alpha_arg = DeclareLaunchArgument('alpha', default_value='0.1')
    max_steps_arg = DeclareLaunchArgument('max_steps', default_value='6387')
    init_seed_arg = DeclareLaunchArgument('init_seed', default_value='42')

    enabled_arg = DeclareLaunchArgument(
        'enabled', default_value='true',
        description='Temporis shaper enabled?')

    # Step 8 v3: agent QoS knobs — what agents themselves use
    qos_reliability_arg = DeclareLaunchArgument(
        'qos_reliability', default_value='reliable',
        description='Agent pub/sub: reliable | best_effort')
    qos_depth_arg = DeclareLaunchArgument(
        'qos_depth', default_value='10',
        description='Agent pub/sub: KeepLast queue depth')

    # Step 8 v3: shaper QoS mode (default: auto = match agents)
    shaper_qos_mode_arg = DeclareLaunchArgument(
        'shaper_qos_mode', default_value='auto',
        description='Shaper QoS: auto | reliable | best_effort. '
                    'Default "auto" auto-detects publisher QoS.')

    output_dir_arg = DeclareLaunchArgument(
        'output_dir', default_value='/tmp/step8v2_run')

    topology_arg = DeclareLaunchArgument(
        'topology', default_value='random_k')
    topology_k_arg = DeclareLaunchArgument(
        'topology_k', default_value='5')

    def launch_setup(context, *args, **kwargs):
        num_agents = int(LaunchConfiguration('num_agents').perform(context))
        rate_hz = float(LaunchConfiguration('rate_hz').perform(context))
        alpha = float(LaunchConfiguration('alpha').perform(context))
        max_steps = int(LaunchConfiguration('max_steps').perform(context))
        init_seed = int(LaunchConfiguration('init_seed').perform(context))
        output_dir = LaunchConfiguration('output_dir').perform(context)
        topology = LaunchConfiguration('topology').perform(context)
        topology_k = int(LaunchConfiguration('topology_k').perform(context))
        enabled_str = LaunchConfiguration('enabled').perform(context).lower()
        enabled = enabled_str in ('true', '1', 'yes')
        qos_rel = LaunchConfiguration('qos_reliability').perform(context)
        qos_depth = int(LaunchConfiguration('qos_depth').perform(context))
        shaper_qos_mode = LaunchConfiguration('shaper_qos_mode').perform(context)

        os.makedirs(output_dir, exist_ok=True)

        meta_path = os.path.join(output_dir, 'run_metadata.txt')
        with open(meta_path, 'w') as f:
            f.write(f'num_agents={num_agents}\n')
            f.write(f'topology={topology}\n')
            f.write(f'topology_k={topology_k}\n')
            f.write(f'rate_hz={rate_hz}\n')
            f.write(f'alpha={alpha}\n')
            f.write(f'init_seed={init_seed}\n')
            f.write(f'shaper_enabled={enabled}\n')
            f.write(f'qos_reliability={qos_rel}\n')
            f.write(f'qos_depth={qos_depth}\n')
            f.write(f'shaper_qos_mode={shaper_qos_mode}\n')
            mode = 'MODEL' if enabled else f'REAL_ZENOH_{qos_rel.upper()}{qos_depth}'
            f.write(f'mode={mode}\n')

        nodes = []

        nodes.append(Node(
            package='temporis_ros2',
            executable='temporis_per_agent_shaper',
            name='temporis_per_agent_shaper',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('temporis_ros2'),
                    'config',
                    'temporis_shaper.yaml'
                ]),
                {
                    'num_agents': num_agents,
                    'topology': topology,
                    'topology_k': topology_k,
                    'enabled': enabled,
                    'publish_diagnostics': False,
                    'qos_reliability': shaper_qos_mode,   # "auto" by default
                    'qos_depth': qos_depth,
                }
            ]
        ))

        for i in range(num_agents):
            csv_path = os.path.join(output_dir, f'agent_{i}.csv')
            latency_path = os.path.join(output_dir, f'agent_{i}_latency.csv')
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
                    'latency_csv': latency_path,
                    'qos_reliability': qos_rel,
                    'qos_depth': qos_depth,
                }]
            ))

        return nodes

    return LaunchDescription([
        num_agents_arg,
        rate_hz_arg,
        alpha_arg,
        max_steps_arg,
        init_seed_arg,
        enabled_arg,
        qos_reliability_arg,
        qos_depth_arg,
        shaper_qos_mode_arg,
        output_dir_arg,
        topology_arg,
        topology_k_arg,
        OpaqueFunction(function=launch_setup),
    ])