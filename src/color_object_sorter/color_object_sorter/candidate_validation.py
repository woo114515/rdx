"""Pure multi-frame color evidence for LiDAR-first candidates."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping, Sequence


@dataclass(frozen=True)
class FrameEvidence:
    candidate_id: int
    visibility: str
    color_scores: Mapping[str, float]


@dataclass(frozen=True)
class Validation:
    candidate_id: int
    color: str
    color_confidence: float
    color_scores: tuple[tuple[str, float], ...]
    visible_observations: int
    color_observations: int
    state: str


@dataclass(frozen=True)
class VisualAssociation:
    detection_index: int | None
    status: str


def visual_association_window(
    normalized_x: float,
    normalized_width: float,
    minimum_window: float,
    padding: float,
    edge_start: float,
    maximum_edge_window: float,
) -> float:
    """Return a wider association window near distorted image edges."""

    if normalized_width < 0.0 or minimum_window <= 0.0 or padding < 0.0:
        raise ValueError("invalid visual window dimensions")
    if not 0.0 <= edge_start < 1.0:
        raise ValueError("edge_start must be in [0, 1)")
    if maximum_edge_window < minimum_window:
        raise ValueError("maximum edge window must not be smaller than minimum")
    edge_window = (
        maximum_edge_window if abs(normalized_x) >= edge_start else minimum_window
    )
    return max(minimum_window, normalized_width / 2.0 + padding, edge_window)


@dataclass
class _EvidenceTrack:
    visible_observations: int = 0
    color_observations: int = 0
    total_scores: dict[str, float] = field(default_factory=dict)
    ambiguous_observations: int = 0


class CandidateColorValidator:
    """Accumulate positive color evidence without rejecting unseen objects."""

    def __init__(
        self,
        minimum_color_observations: int = 5,
        minimum_visible_observations: int = 8,
        minimum_color_confidence: float = 0.65,
    ) -> None:
        if minimum_color_observations < 1 or minimum_visible_observations < 1:
            raise ValueError("observation limits must be positive")
        if not 0.0 <= minimum_color_confidence <= 1.0:
            raise ValueError("minimum_color_confidence must be in [0, 1]")
        self.minimum_color_observations = minimum_color_observations
        self.minimum_visible_observations = minimum_visible_observations
        self.minimum_color_confidence = minimum_color_confidence
        self._tracks: dict[int, _EvidenceTrack] = {}

    def update(
        self,
        evidence: Sequence[FrameEvidence],
        active_ids: set[int],
        color_names: Sequence[str],
    ) -> tuple[Validation, ...]:
        self._tracks = {
            candidate_id: track
            for candidate_id, track in self._tracks.items()
            if candidate_id in active_ids
        }
        by_id = {item.candidate_id: item for item in evidence}
        for candidate_id in active_ids:
            track = self._tracks.setdefault(candidate_id, _EvidenceTrack())
            item = by_id.get(candidate_id)
            if item is None or item.visibility == "unobserved":
                continue
            if item.visibility == "ambiguous":
                track.ambiguous_observations += 1
                continue
            if item.visibility not in ("visible", "edge"):
                raise ValueError(f"invalid visibility state: {item.visibility}")
            if item.visibility == "visible":
                track.visible_observations += 1
            usable_scores = {
                color: max(0.0, float(item.color_scores.get(color, 0.0)))
                for color in color_names
            }
            if any(score > 0.0 for score in usable_scores.values()):
                track.color_observations += 1
                for color, score in usable_scores.items():
                    track.total_scores[color] = track.total_scores.get(color, 0.0) + score
        return tuple(
            self._summarize(candidate_id, color_names)
            for candidate_id in sorted(active_ids)
        )

    def clear(self) -> None:
        self._tracks.clear()

    def _summarize(
        self, candidate_id: int, color_names: Sequence[str]
    ) -> Validation:
        track = self._tracks.get(candidate_id, _EvidenceTrack())
        ordered_scores = tuple(
            (color, track.total_scores.get(color, 0.0)) for color in color_names
        )
        total = sum(score for _color, score in ordered_scores)
        if total > 0.0:
            color, winning_score = max(ordered_scores, key=lambda item: item[1])
            confidence = winning_score / total
        else:
            color, confidence = "", 0.0
        if (
            track.color_observations >= self.minimum_color_observations
            and confidence >= self.minimum_color_confidence
        ):
            state = "validated"
        elif (
            track.visible_observations >= self.minimum_visible_observations
            and track.color_observations == 0
        ):
            state = "rejected"
        elif track.visible_observations == 0 and track.ambiguous_observations == 0:
            state = "unobserved"
        else:
            state = "uncertain"
        normalized_scores = tuple(
            (name, score / total if total > 0.0 else 0.0)
            for name, score in ordered_scores
        )
        return Validation(
            candidate_id=candidate_id,
            color=color,
            color_confidence=confidence,
            color_scores=normalized_scores,
            visible_observations=track.visible_observations,
            color_observations=track.color_observations,
            state=state,
        )


def inventory_matches(
    validations: Sequence[Validation],
    expected_colors: Sequence[str],
    expected_counts: Sequence[int],
) -> bool:
    """Check the configured inventory without assuming names or quantities."""

    if len(expected_colors) != len(expected_counts):
        raise ValueError("inventory colors and counts must have equal lengths")
    validated = [item for item in validations if item.state == "validated"]
    if not expected_colors:
        return bool(validated)
    actual = {
        color: sum(item.color == color for item in validated)
        for color in expected_colors
    }
    return all(
        actual[color] == int(count)
        for color, count in zip(expected_colors, expected_counts)
    ) and len(validated) == sum(int(count) for count in expected_counts)


def associate_visual_detections(
    predicted_x: Sequence[float],
    detection_x: Sequence[float],
    detection_windows: Sequence[float],
    ambiguity_margin: float,
) -> tuple[VisualAssociation, ...]:
    """Assign each visual detection to at most one spatial candidate."""

    if len(detection_x) != len(detection_windows):
        raise ValueError("each detection needs an association window")
    if ambiguity_margin < 0.0 or any(window <= 0.0 for window in detection_windows):
        raise ValueError("invalid visual association limits")
    candidates = []
    for projected in predicted_x:
        matches = []
        for index, (center, window) in enumerate(zip(detection_x, detection_windows)):
            error = abs(projected - center)
            if error <= window:
                matches.append((index, error))
        candidates.append(tuple(sorted(matches, key=lambda item: (item[1], item[0]))))

    best: tuple[int, float, tuple[int | None, ...]] | None = None

    def search(
        position: int,
        used: set[int],
        assignment: tuple[int | None, ...],
        matched: int,
        cost: float,
    ) -> None:
        nonlocal best
        if position == len(candidates):
            option = (matched, cost, assignment)
            if best is None or (-matched, cost, _assignment_key(assignment)) < (
                -best[0],
                best[1],
                _assignment_key(best[2]),
            ):
                best = option
            return
        search(position + 1, used, assignment + (None,), matched, cost)
        for detection_index, error in candidates[position]:
            if detection_index in used:
                continue
            used.add(detection_index)
            search(
                position + 1,
                used,
                assignment + (detection_index,),
                matched + 1,
                cost + error,
            )
            used.remove(detection_index)

    search(0, set(), (), 0, 0.0)
    assert best is not None
    assignment = best[2]
    output = []
    for candidate_index, detection_index in enumerate(assignment):
        own = dict(candidates[candidate_index])
        if detection_index is None:
            output.append(
                VisualAssociation(None, "ambiguous" if own else "unmatched")
            )
            continue
        error = own[detection_index]
        ambiguous = any(
            alternative != detection_index
            and abs(alternative_error - error) <= ambiguity_margin
            for alternative, alternative_error in candidates[candidate_index]
        )
        for other_index, other_candidates in enumerate(candidates):
            if other_index == candidate_index:
                continue
            competing = dict(other_candidates).get(detection_index)
            if competing is not None and abs(competing - error) <= ambiguity_margin:
                ambiguous = True
                break
        output.append(
            VisualAssociation(
                None if ambiguous else detection_index,
                "ambiguous" if ambiguous else "matched",
            )
        )
    return tuple(output)


def _assignment_key(items: tuple[int | None, ...]) -> tuple[int, ...]:
    return tuple(10**9 if item is None else item for item in items)
