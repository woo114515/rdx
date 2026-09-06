from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PATCH_ROOT = ROOT / "reference/vendor-patches/2026-09-05"


def test_workspace_patch_has_fail_closed_contracts():
    source = (PATCH_ROOT / "yahboomcar-workspace-hardening.patch").read_text()
    for contract in (
        "motion_command_topic",
        '"motion_command_topic": "/cmd_vel"',
        "vel_raw_stamped",
        "driver_health",
        "MultiThreadedExecutor",
        "waiting_for_feedback",
        "max_sample_interval",
    ):
        assert contract in source


def test_serial_library_patch_has_bounded_io_and_lazy_camera_import():
    source = (PATCH_ROOT / "sunrise-robot-lib-hardening.patch").read_text()
    for contract in (
        "timeout=0.2",
        "write_timeout=0.2",
        "get_receive_status",
        "get_report_age",
        "__serial_write",
        "def __getattr__",
    ):
        assert contract in source


def test_patch_baseline_checksum_is_documented():
    readme = (PATCH_ROOT / "README.md").read_text()
    assert "b776b71f8bc260c67fa1f804522b305ffbc1415ce40c6fb1bc64f48b518db6fa" in readme
    assert "Nothing in this directory is automatically deployed" in readme


def test_workspace_patch_preserves_official_motion_graph():
    source = (PATCH_ROOT / "yahboomcar-workspace-hardening.patch").read_text()
    assert '"motion_command_topic": "/cmd_vel"' in source
    assert source.count('create_publisher(Twist, "/cmd_vel", 10)') == 2
    assert 'create_publisher(Twist,"/cmd_vel", 10)' in source
    assert "create_publisher(Twist, '/cmd_vel', 10)" in source
    assert "/cmd_vel_safe" not in source
    assert "/cmd_vel_teleop" not in source
