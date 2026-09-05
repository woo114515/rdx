"""Small deterministic image-space tracker for color detections."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from .vision import Detection


@dataclass(frozen=True)
class TrackedDetection:
    track_id: int
    detection: Detection
    age: int


@dataclass
class _Track:
    track_id: int
    detection: Detection
    age: int = 1
    missed: int = 0


class ObjectTracker:
    def __init__(self, maximum_distance: float = 0.25, maximum_missed: int = 3):
        if maximum_distance <= 0.0 or maximum_missed < 0:
            raise ValueError("invalid tracker limits")
        self.maximum_distance = maximum_distance
        self.maximum_missed = maximum_missed
        self._next_id = 1
        self._tracks: dict[int, _Track] = {}

    def update(self, detections: Iterable[Detection]) -> tuple[TrackedDetection, ...]:
        unmatched_tracks = set(self._tracks)
        results: list[TrackedDetection] = []
        ordered = sorted(detections, key=lambda item: (-item.area_ratio, item.color))
        for detection in ordered:
            candidates = [
                track
                for track in self._tracks.values()
                if track.track_id in unmatched_tracks
                and track.detection.color == detection.color
            ]
            match = min(
                candidates,
                key=lambda track: self._distance(track.detection, detection),
                default=None,
            )
            if match is None or self._distance(match.detection, detection) > self.maximum_distance:
                match = _Track(self._next_id, detection)
                self._tracks[match.track_id] = match
                self._next_id += 1
            else:
                unmatched_tracks.remove(match.track_id)
                match.detection = detection
                match.age += 1
                match.missed = 0
            results.append(TrackedDetection(match.track_id, detection, match.age))

        for track_id in tuple(unmatched_tracks):
            track = self._tracks[track_id]
            track.missed += 1
            if track.missed > self.maximum_missed:
                del self._tracks[track_id]
        return tuple(sorted(results, key=lambda item: item.track_id))

    @staticmethod
    def _distance(first: Detection, second: Detection) -> float:
        return math.hypot(
            first.normalized_x - second.normalized_x,
            first.normalized_y - second.normalized_y,
        )
