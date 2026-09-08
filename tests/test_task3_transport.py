from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PLANNING_LAUNCH = (
    ROOT / "src/cylinder_push_planner/launch/task3_planning.launch.py"
)
VISUALIZATION_LAUNCH = (
    ROOT / "src/color_object_sorter/launch/task3_visualization.launch.py"
)
ENV_HELPER = ROOT / "scripts/task3_ros_env.sh"


def test_task3_launches_select_udp_before_starting_nodes():
    for launch_file in (PLANNING_LAUNCH, VISUALIZATION_LAUNCH):
        source = launch_file.read_text(encoding="utf-8")
        environment_index = source.index("SetEnvironmentVariable(")
        node_index = source.index("Node(")

        assert "FASTDDS_BUILTIN_TRANSPORTS" in source
        assert 'default_value="UDPv4"' in source
        assert environment_index < node_index


def test_interactive_helper_uses_project_ros_domain_and_udp():
    source = ENV_HELPER.read_text(encoding="utf-8")

    assert "source /opt/tros/humble/setup.bash" in source
    assert "source /opt/ros/humble/setup.bash" in source
    assert 'export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"' in source
    assert "unset ROS_DISCOVERY_SERVER" in source
    assert "export FASTDDS_BUILTIN_TRANSPORTS=UDPv4" in source
