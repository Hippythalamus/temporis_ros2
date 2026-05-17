"""
Temporis ROS2 launch — config-driven (Step 9, fixed).

Reads all tunable parameters from config/temporis.yaml.
CLI arguments OVERRIDE YAML values only when explicitly provided.

Default behavior:
  ros2 launch temporis_ros2 consensus.launch.py
  → uses ALL values from config/temporis.yaml as-is

With overrides:
  ros2 launch temporis_ros2 consensus.launch.py num_agents:=50
  → uses YAML for everything, but num_agents=50

Implementation note:
  CLI args default to empty string. Only non-empty values are
  injected as overrides. This way YAML wins when no CLI is given.

Special case for output_dir: it has a hard-coded default because it's
not in the YAML (it's a run-specific path, not a tunable parameter).
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # All tunable CLI args default to empty string.
    # If user doesn't specify them, they stay empty, and we do NOT
    # inject anything into ROS params (so YAML wins).
    num_agents_arg = DeclareLaunchArgument(
        'num_agents', default_value='',
        description='Override num_agents (default: from YAML)')

    topology_arg = DeclareLaunchArgument(
        'topology', default_value='',
        description='Override topology (all_to_all|ring|grid|random_k)')

    topology_k_arg = DeclareLaunchArgument(
        'topology_k', default_value='',
        description='Override topology_k for random_k')

    enabled_arg = DeclareLaunchArgument(
        'enabled', default_value='',
        description='Override shaper enabled (true|false)')

    # output_dir IS hardcoded here — it's a run-specific path, not a
    # tunable parameter, so it doesn't live in YAML.
    output_dir_arg = DeclareLaunchArgument(
        'output_dir', default_value='/tmp/temporis_run',
        description='Directory for per-agent CSV output')

    config_file_arg = DeclareLaunchArgument(
        'config_file', default_value='',
        description='Path to custom temporis.yaml (empty = package default)')

    def launch_setup(context, *args, **kwargs):
        num_agents_str = LaunchConfiguration('num_agents').perform(context)
        topology_str = LaunchConfiguration('topology').perform(context)
        topology_k_str = LaunchConfiguration('topology_k').perform(context)
        enabled_str = LaunchConfiguration('enabled').perform(context)
        output_dir = LaunchConfiguration('output_dir').perform(context)
        custom_config = LaunchConfiguration('config_file').perform(context)

        # Resolve config path
        if custom_config:
            config_path = custom_config
        else:
            from ament_index_python.packages import get_package_share_directory
            config_path = os.path.join(
                get_package_share_directory('temporis_ros2'),
                'config', 'temporis.yaml')

        # We MUST know num_agents to know how many agent nodes to spawn.
        # If user overrode it on CLI, use that. Otherwise read from YAML.
        if num_agents_str:
            num_agents = int(num_agents_str)
        else:
            num_agents = _read_yaml_param(
                config_path, 'temporis_per_agent_shaper', 'num_agents',
                default=10)

        # Build shaper overrides — only include non-empty CLI values.
        # This is the KEY FIX: YAML wins when CLI is not provided.
        shaper_overrides = {}
        if num_agents_str:
            shaper_overrides['num_agents'] = int(num_agents_str)
        if topology_str:
            shaper_overrides['topology'] = topology_str
        if topology_k_str:
            shaper_overrides['topology_k'] = int(topology_k_str)
        if enabled_str:
            shaper_overrides['enabled'] = enabled_str.lower() in (
                'true', '1', 'yes')

        # Agent overrides: only num_agents flows from CLI to all agents
        # (the rest is per-agent: agent_id, csv paths).
        agent_common_overrides = {}
        if num_agents_str:
            agent_common_overrides['num_agents'] = int(num_agents_str)

        os.makedirs(output_dir, exist_ok=True)

        # Metadata
        meta_path = os.path.join(output_dir, 'run_metadata.txt')
        with open(meta_path, 'w') as f:
            f.write(f'config_file={config_path}\n')
            f.write(f'num_agents={num_agents}\n')
            f.write(f'cli_overrides={shaper_overrides}\n')

        nodes = []

        # --- Shaper ---
        shaper_params = [config_path]
        if shaper_overrides:
            shaper_params.append(shaper_overrides)

        nodes.append(Node(
            package='temporis_ros2',
            executable='temporis_per_agent_shaper',
            name='temporis_per_agent_shaper',
            output='screen',
            parameters=shaper_params,
        ))

        # --- Consensus agents ---
        for i in range(num_agents):
            csv_path = os.path.join(output_dir, f'agent_{i}.csv')
            latency_path = os.path.join(output_dir, f'agent_{i}_latency.csv')

            agent_params_dict = {
                'agent_id': i,
                'output_csv': csv_path,
                'latency_csv': latency_path,
                **agent_common_overrides,
            }

            nodes.append(Node(
                package='temporis_ros2',
                executable='consensus_agent',
                namespace=f'/robot_{i}',
                name='consensus_agent',
                output='screen',
                parameters=[config_path, agent_params_dict],
            ))

        return nodes

    return LaunchDescription([
        num_agents_arg,
        topology_arg,
        topology_k_arg,
        enabled_arg,
        output_dir_arg,
        config_file_arg,
        OpaqueFunction(function=launch_setup),
    ])


def _read_yaml_param(yaml_path, node_key, param_name, default=None):
    """Read one parameter from a ROS2 YAML config without requiring PyYAML.

    Used to determine how many agent nodes to spawn when num_agents
    isn't overridden on the CLI.
    """
    try:
        import yaml
        with open(yaml_path) as f:
            data = yaml.safe_load(f)
        return data[node_key]['ros__parameters'][param_name]
    except ImportError:
        import re
        with open(yaml_path) as f:
            content = f.read()
        node_re = re.search(
            rf'^{re.escape(node_key)}:\s*\n(.*?)(?=^\S|\Z)',
            content, re.MULTILINE | re.DOTALL)
        if not node_re:
            return default
        section = node_re.group(1)
        param_re = re.search(
            rf'^\s+{re.escape(param_name)}:\s*(\S+)',
            section, re.MULTILINE)
        if not param_re:
            return default
        val = param_re.group(1)
        try:
            return int(val)
        except ValueError:
            return val
    except (KeyError, FileNotFoundError):
        return default