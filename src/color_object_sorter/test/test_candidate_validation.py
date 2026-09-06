import pytest

from color_object_sorter.candidate_validation import (
    CandidateColorValidator,
    FrameEvidence,
    associate_visual_detections,
    inventory_matches,
    visual_association_window,
)


def test_unobserved_candidate_is_not_rejected() -> None:
    validator = CandidateColorValidator(
        minimum_color_observations=2,
        minimum_visible_observations=2,
    )
    result = None
    for _ in range(5):
        result = validator.update(
            [FrameEvidence(1, "unobserved", {})], {1}, ["blue", "red"]
        )
    assert result is not None
    assert result[0].state == "unobserved"


def test_visible_background_candidate_is_rejected() -> None:
    validator = CandidateColorValidator(
        minimum_color_observations=2,
        minimum_visible_observations=2,
    )
    validator.update([FrameEvidence(1, "visible", {})], {1}, ["blue"])
    result = validator.update([FrameEvidence(1, "visible", {})], {1}, ["blue"])
    assert result[0].state == "rejected"


def test_multiframe_color_evidence_validates_arbitrary_color() -> None:
    validator = CandidateColorValidator(
        minimum_color_observations=3,
        minimum_visible_observations=3,
        minimum_color_confidence=0.7,
    )
    result = None
    for _ in range(3):
        result = validator.update(
            [FrameEvidence(7, "visible", {"yellow": 0.9, "blue": 0.1})],
            {7},
            ["blue", "yellow"],
        )
    assert result is not None
    assert result[0].state == "validated"
    assert result[0].color == "yellow"
    assert result[0].color_confidence == pytest.approx(0.9)


def test_inventory_matching_uses_configured_names_and_counts() -> None:
    validator = CandidateColorValidator(minimum_color_observations=1)
    result = validator.update(
        [
            FrameEvidence(1, "visible", {"cyan": 1.0}),
            FrameEvidence(2, "visible", {"cyan": 1.0}),
            FrameEvidence(3, "visible", {"orange": 1.0}),
        ],
        {1, 2, 3},
        ["cyan", "orange"],
    )
    assert inventory_matches(result, ["cyan", "orange"], [2, 1])
    assert not inventory_matches(result, ["cyan", "orange"], [1, 2])


def test_candidate_disappearance_removes_old_evidence() -> None:
    validator = CandidateColorValidator(minimum_color_observations=1)
    validator.update([FrameEvidence(1, "visible", {"blue": 1.0})], {1}, ["blue"])
    result = validator.update([], {2}, ["blue"])
    assert [item.candidate_id for item in result] == [2]


def test_one_visual_detection_cannot_validate_two_candidates() -> None:
    result = associate_visual_detections(
        predicted_x=[0.10, 0.11],
        detection_x=[0.105],
        detection_windows=[0.10],
        ambiguity_margin=0.02,
    )
    assert all(item.detection_index is None for item in result)
    assert all(item.status == "ambiguous" for item in result)


def test_distinct_visual_detections_are_assigned_once() -> None:
    result = associate_visual_detections(
        predicted_x=[-0.4, 0.4],
        detection_x=[-0.38, 0.39],
        detection_windows=[0.10, 0.10],
        ambiguity_margin=0.01,
    )
    assert [item.detection_index for item in result] == [0, 1]


def test_visual_window_stays_narrow_in_image_center() -> None:
    assert visual_association_window(0.2, 0.04, 0.08, 0.03, 0.75, 0.14) == 0.08


def test_visual_window_grows_near_image_edge() -> None:
    window = visual_association_window(0.8766, 0.0453, 0.08, 0.03, 0.75, 0.14)
    assert window == 0.14
    assert visual_association_window(1.0, 0.04, 0.08, 0.03, 0.75, 0.14) == 0.14
