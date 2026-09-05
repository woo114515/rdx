"""Pure temporal confirmation logic for localized color observations."""

from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass, field
import math
from statistics import median, pstdev


@dataclass(frozen=True)
class Observation:
    frame: int
    stamp: float
    color: str
    shape: str
    confidence: float
    bearing: float
    distance: float
    matched: bool


@dataclass(frozen=True)
class Confirmation:
    track_id: int
    color: str
    shape: str
    confidence: float
    bearing: float
    distance: float
    observation_count: int
    detection_rate: float
    color_consistency: float
    lidar_match_rate: float
    bearing_stddev: float
    distance_stddev: float
    confirmed: bool


@dataclass
class _Track:
    track_id: int
    observations: deque[Observation] = field(default_factory=deque)


class TemporalConfirmer:
    """Associate observations spatially and confirm only stable histories."""

    def __init__(
        self,
        window_seconds: float = 1.0,
        minimum_observations: int = 6,
        minimum_detection_rate: float = 0.6,
        minimum_color_consistency: float = 0.8,
        minimum_lidar_match_rate: float = 0.5,
        maximum_bearing_stddev: float = math.radians(3.0),
        maximum_distance_stddev: float = 0.10,
        association_bearing: float = math.radians(6.0),
        association_distance: float = 0.30,
    ) -> None:
        if window_seconds <= 0.0 or minimum_observations < 1:
            raise ValueError("window and minimum observations must be positive")
        self.window_seconds = window_seconds
        self.minimum_observations = minimum_observations
        self.minimum_detection_rate = minimum_detection_rate
        self.minimum_color_consistency = minimum_color_consistency
        self.minimum_lidar_match_rate = minimum_lidar_match_rate
        self.maximum_bearing_stddev = maximum_bearing_stddev
        self.maximum_distance_stddev = maximum_distance_stddev
        self.association_bearing = association_bearing
        self.association_distance = association_distance
        self._tracks: list[_Track] = []
        self._frame_stamps: deque[tuple[int, float]] = deque()
        self._next_track_id = 1
        self._frame = 0

    def update(self, stamp: float, observations: list[Observation]) -> list[Confirmation]:
        self._frame += 1
        self._frame_stamps.append((self._frame, stamp))
        cutoff = stamp - self.window_seconds
        while self._frame_stamps and self._frame_stamps[0][1] < cutoff:
            self._frame_stamps.popleft()
        for track in self._tracks:
            while track.observations and track.observations[0].stamp < cutoff:
                track.observations.popleft()
        self._tracks = [track for track in self._tracks if track.observations]

        available = set(range(len(self._tracks)))
        for source in observations:
            candidate = self._best_track(source, available)
            if candidate is None:
                track = _Track(self._next_track_id)
                self._next_track_id += 1
                self._tracks.append(track)
            else:
                track = self._tracks[candidate]
                available.remove(candidate)
            track.observations.append(
                Observation(
                    frame=self._frame,
                    stamp=stamp,
                    color=source.color,
                    shape=source.shape,
                    confidence=source.confidence,
                    bearing=source.bearing,
                    distance=source.distance,
                    matched=source.matched,
                )
            )
        return [self._summarize(track) for track in self._tracks]

    def _best_track(self, observation: Observation, available: set[int]) -> int | None:
        scored: list[tuple[float, int]] = []
        for index in available:
            latest = self._tracks[index].observations[-1]
            bearing_error = abs(_angle_difference(observation.bearing, latest.bearing))
            if bearing_error > self.association_bearing:
                continue
            distance_error = 0.0
            if observation.matched and latest.matched:
                distance_error = abs(observation.distance - latest.distance)
                if distance_error > self.association_distance:
                    continue
            scored.append((bearing_error + distance_error, index))
        return min(scored)[1] if scored else None

    def _summarize(self, track: _Track) -> Confirmation:
        items = list(track.observations)
        counts = Counter(item.color for item in items)
        color, color_count = counts.most_common(1)[0]
        colored = [item for item in items if item.color == color]
        matched = [item for item in colored if item.matched]
        bearings = [item.bearing for item in matched]
        distances = [item.distance for item in matched]
        bearing_stddev = _circular_stddev(bearings)
        distance_stddev = pstdev(distances) if len(distances) > 1 else 0.0
        frame_count = max(1, len(self._frame_stamps))
        detection_rate = len({item.frame for item in items}) / frame_count
        color_consistency = color_count / len(items)
        lidar_match_rate = len(matched) / len(colored)
        confirmed = (
            len(items) >= self.minimum_observations
            and detection_rate >= self.minimum_detection_rate
            and color_consistency >= self.minimum_color_consistency
            and lidar_match_rate >= self.minimum_lidar_match_rate
            and bool(matched)
            and bearing_stddev <= self.maximum_bearing_stddev
            and distance_stddev <= self.maximum_distance_stddev
        )
        return Confirmation(
            track_id=track.track_id,
            color=color,
            shape=Counter(item.shape for item in colored).most_common(1)[0][0],
            confidence=median(item.confidence for item in colored),
            bearing=_circular_mean(bearings) if bearings else math.nan,
            distance=median(distances) if distances else math.nan,
            observation_count=len(items),
            detection_rate=detection_rate,
            color_consistency=color_consistency,
            lidar_match_rate=lidar_match_rate,
            bearing_stddev=bearing_stddev,
            distance_stddev=distance_stddev,
            confirmed=confirmed,
        )


def _angle_difference(left: float, right: float) -> float:
    return math.atan2(math.sin(left - right), math.cos(left - right))


def _circular_mean(values: list[float]) -> float:
    return math.atan2(
        sum(math.sin(value) for value in values),
        sum(math.cos(value) for value in values),
    )


def _circular_stddev(values: list[float]) -> float:
    if len(values) < 2:
        return 0.0
    center = _circular_mean(values)
    return pstdev(_angle_difference(value, center) for value in values)
