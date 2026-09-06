"""Hardware-independent LiDAR cylinder candidate extraction and accumulation."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from statistics import median
from typing import Sequence


@dataclass(frozen=True)
class ScanCandidate:
    """One cylinder-like cluster in the laser frame."""

    x: float
    y: float
    radius: float
    confidence: float
    point_count: int


@dataclass(frozen=True)
class StableCandidate:
    """One candidate accumulated in the fixed frame."""

    candidate_id: int
    x: float
    y: float
    radius: float
    confidence: float
    observation_count: int
    last_seen: float


@dataclass
class _Track:
    candidate_id: int
    observations: list[tuple[float, float, float, float, float]] = field(
        default_factory=list
    )
    missed_updates: int = 0


def extract_candidates(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    *,
    maximum_point_gap: float = 0.08,
    minimum_points: int = 2,
    minimum_diameter: float = 0.015,
    maximum_diameter: float = 0.16,
    nominal_radius: float = 0.035,
) -> tuple[ScanCandidate, ...]:
    """Extract compact, cylinder-sized clusters from one laser scan."""

    if angle_increment <= 0.0:
        raise ValueError("angle_increment must be positive")
    if maximum_point_gap <= 0.0 or minimum_points < 2:
        raise ValueError("invalid clustering limits")
    if not 0.0 <= minimum_diameter < maximum_diameter:
        raise ValueError("invalid diameter limits")
    if nominal_radius <= 0.0:
        raise ValueError("nominal_radius must be positive")

    groups: list[list[tuple[float, float, float]]] = []
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance) or not range_min <= distance <= range_max:
            groups.append([])
            continue
        angle = angle_min + index * angle_increment
        point = (distance * math.cos(angle), distance * math.sin(angle), distance)
        if not groups or not groups[-1]:
            groups.append([point])
            continue
        previous = groups[-1][-1]
        gap = math.hypot(point[0] - previous[0], point[1] - previous[1])
        if gap <= maximum_point_gap:
            groups[-1].append(point)
        else:
            groups.append([point])

    candidates = []
    for group in groups:
        if len(group) < minimum_points:
            continue
        chord = math.hypot(group[-1][0] - group[0][0], group[-1][1] - group[0][1])
        if not minimum_diameter <= chord <= maximum_diameter:
            continue
        bearing = math.atan2(
            sum(point[1] for point in group),
            sum(point[0] for point in group),
        )
        surface_distance = median(point[2] for point in group)
        center_distance = surface_distance + nominal_radius
        diameter_midpoint = (minimum_diameter + maximum_diameter) / 2.0
        diameter_half_range = (maximum_diameter - minimum_diameter) / 2.0
        size_score = max(
            0.0,
            1.0 - abs(chord - diameter_midpoint) / diameter_half_range,
        )
        point_score = min(1.0, len(group) / max(4.0, float(minimum_points)))
        candidates.append(
            ScanCandidate(
                x=center_distance * math.cos(bearing),
                y=center_distance * math.sin(bearing),
                radius=nominal_radius,
                confidence=0.5 * size_score + 0.5 * point_score,
                point_count=len(group),
            )
        )
    return tuple(candidates)


class CandidateAccumulator:
    """Accumulate one explicitly bounded observation session."""

    def __init__(
        self,
        association_distance: float = 0.12,
        history_size: int = 30,
        maximum_missed_updates: int = 5,
    ) -> None:
        if (
            association_distance <= 0.0
            or history_size < 1
            or maximum_missed_updates < 0
        ):
            raise ValueError("invalid accumulation limits")
        self.association_distance = association_distance
        self.history_size = history_size
        self.maximum_missed_updates = maximum_missed_updates
        self._tracks: list[_Track] = []
        self._next_id = 1

    def update(
        self,
        observations: Sequence[tuple[float, float, float, float]],
        stamp: float,
    ) -> tuple[StableCandidate, ...]:
        pairs = sorted(
            (
                (math.hypot(x - self._x(track), y - self._y(track)), oi, ti)
                for oi, (x, y, _radius, _confidence) in enumerate(observations)
                for ti, track in enumerate(self._tracks)
            ),
            key=lambda item: item[0],
        )
        used_observations: set[int] = set()
        used_tracks: set[int] = set()
        for distance, observation_index, track_index in pairs:
            if distance > self.association_distance:
                break
            if observation_index in used_observations or track_index in used_tracks:
                continue
            self._append(self._tracks[track_index], observations[observation_index], stamp)
            self._tracks[track_index].missed_updates = 0
            used_observations.add(observation_index)
            used_tracks.add(track_index)
        for track_index, track in enumerate(self._tracks):
            if track_index not in used_tracks:
                track.missed_updates += 1
        for index, observation in enumerate(observations):
            if index in used_observations:
                continue
            track = _Track(self._next_id)
            self._next_id += 1
            self._append(track, observation, stamp)
            self._tracks.append(track)
        self._tracks = [
            track
            for track in self._tracks
            if track.missed_updates <= self.maximum_missed_updates
        ]
        return tuple(self._summary(track) for track in self._tracks)

    def stable(self, minimum_observations: int) -> tuple[StableCandidate, ...]:
        if minimum_observations < 1:
            raise ValueError("minimum_observations must be positive")
        return tuple(
            item
            for item in (self._summary(track) for track in self._tracks)
            if item.observation_count >= minimum_observations
        )

    def clear(self) -> None:
        self._tracks.clear()
        self._next_id = 1

    def _append(
        self,
        track: _Track,
        observation: tuple[float, float, float, float],
        stamp: float,
    ) -> None:
        track.observations.append((*observation, stamp))
        del track.observations[:-self.history_size]

    @staticmethod
    def _x(track: _Track) -> float:
        return median(item[0] for item in track.observations)

    @staticmethod
    def _y(track: _Track) -> float:
        return median(item[1] for item in track.observations)

    def _summary(self, track: _Track) -> StableCandidate:
        observations = track.observations
        return StableCandidate(
            candidate_id=track.candidate_id,
            x=self._x(track),
            y=self._y(track),
            radius=median(item[2] for item in observations),
            confidence=median(item[3] for item in observations),
            observation_count=len(observations),
            last_seen=max(item[4] for item in observations),
        )


def merge_nearby_observations(
    observations: Sequence[tuple[float, float, float, float]],
    minimum_separation: float,
) -> tuple[tuple[float, float, float, float], ...]:
    """Merge fragments too close to represent distinct physical objects."""

    if minimum_separation <= 0.0:
        raise ValueError("minimum_separation must be positive")
    remaining = set(range(len(observations)))
    groups: list[list[int]] = []
    while remaining:
        seed = remaining.pop()
        group = [seed]
        frontier = [seed]
        while frontier:
            current = frontier.pop()
            x, y, _radius, _confidence = observations[current]
            neighbors = {
                index
                for index in remaining
                if math.hypot(
                    observations[index][0] - x,
                    observations[index][1] - y,
                )
                < minimum_separation
            }
            remaining.difference_update(neighbors)
            group.extend(neighbors)
            frontier.extend(neighbors)
        groups.append(group)
    merged = []
    for group in groups:
        items = [observations[index] for index in group]
        merged.append(
            (
                median(item[0] for item in items),
                median(item[1] for item in items),
                median(item[2] for item in items),
                max(item[3] for item in items),
            )
        )
    return tuple(merged)


def select_primary_spatial_group(
    candidates: Sequence[StableCandidate],
    maximum_neighbor_distance: float,
) -> tuple[StableCandidate, ...]:
    """Select the largest spatially connected candidate group."""

    if maximum_neighbor_distance <= 0.0:
        raise ValueError("maximum_neighbor_distance must be positive")
    if not candidates:
        return ()
    remaining = set(range(len(candidates)))
    groups: list[list[int]] = []
    while remaining:
        seed = remaining.pop()
        group = [seed]
        frontier = [seed]
        while frontier:
            current = frontier.pop()
            neighbors = {
                index
                for index in remaining
                if math.hypot(
                    candidates[index].x - candidates[current].x,
                    candidates[index].y - candidates[current].y,
                )
                <= maximum_neighbor_distance
            }
            remaining.difference_update(neighbors)
            group.extend(neighbors)
            frontier.extend(neighbors)
        groups.append(group)

    def rank(group: list[int]) -> tuple[int, float, float]:
        items = [candidates[index] for index in group]
        mean_confidence = sum(item.confidence for item in items) / len(items)
        diameter = max(
            (
                math.hypot(left.x - right.x, left.y - right.y)
                for left in items
                for right in items
            ),
            default=0.0,
        )
        return len(items), mean_confidence, -diameter

    selected = max(groups, key=rank)
    return tuple(sorted((candidates[index] for index in selected), key=lambda x: x.candidate_id))
