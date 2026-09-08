from pathlib import Path


PATCH = (
    Path(__file__).parents[1]
    / "reference"
    / "vendor-patches"
    / "2026-09-08"
    / "slam-gmapping-map-update-throttle.patch"
)


def test_gmapping_patch_loads_only_timing_parameters() -> None:
    text = PATCH.read_text(encoding="utf-8")

    assert 'declare_parameter<double>("map_update_interval", 0.5)' in text
    assert 'declare_parameter<double>("transform_publish_period", 0.05)' in text


def test_gmapping_patch_preserves_fractional_tf_delay() -> None:
    text = PATCH.read_text(encoding="utf-8")

    assert "+    rclcpp::Time tf_expiration" in text
    assert "rclcpp::Duration::from_seconds(tf_delay_)" in text
    assert "+            static_cast<int32_t>" not in text
