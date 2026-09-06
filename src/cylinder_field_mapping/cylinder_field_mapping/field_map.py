"""Pure map-level tracking and enclosing-circle geometry."""

from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass, field
import math
from statistics import median


@dataclass(frozen=True)
class Observation:
    x: float
    y: float
    color: str
    confidence: float
    stamp: float


@dataclass(frozen=True)
class Cylinder:
    cylinder_id: int
    x: float
    y: float
    color: str
    confidence: float
    observation_count: int
    last_seen: float


@dataclass(frozen=True)
class Circle:
    x: float
    y: float
    radius: float


@dataclass
class _Track:
    cylinder_id: int
    observations: deque[Observation] = field(default_factory=deque)


class CylinderFieldMap:
    """Associate observations by map position, independent of camera IDs."""

    def __init__(
        self,
        association_distance: float = 0.15,
        history_size: int = 25,
        relocation_distance: float = 0.75,
        relocation_max_age: float = 3.0,
    ) -> None:
        if (
            association_distance <= 0.0
            or history_size < 1
            or relocation_distance < association_distance
            or relocation_max_age <= 0.0
        ):
            raise ValueError("mapping limits must be positive")
        self.association_distance = association_distance
        self.history_size = history_size
        self.relocation_distance = relocation_distance
        self.relocation_max_age = relocation_max_age
        self._tracks: list[_Track] = []
        self._next_id = 1
        self._latest_stamp: float | None = None

    def update(
        self,
        observations: list[Observation],
        stamp: float | None = None,
    ) -> tuple[Cylinder, ...]:
        if observations or stamp is not None:
            newest = (
                max(item.stamp for item in observations)
                if stamp is None
                else stamp
            )
            if observations and any(item.stamp > newest for item in observations):
                raise ValueError("update stamp precedes an observation")
            if self._latest_stamp is not None and newest < self._latest_stamp:
                raise ValueError("observation time must not move backwards")
            self._latest_stamp = newest
        pairs = sorted(
            (
                (
                    math.hypot(obs.x - self._x(track), obs.y - self._y(track)),
                    oi,
                    ti,
                )
                for oi, obs in enumerate(observations)
                for ti, track in enumerate(self._tracks)
            ),
            key=lambda item: item[0],
        )
        assigned_observations: set[int] = set()
        assigned_tracks: set[int] = set()
        for distance, observation_index, track_index in pairs:
            if distance > self.association_distance:
                break
            if (
                observation_index in assigned_observations
                or track_index in assigned_tracks
            ):
                continue
            self._append(self._tracks[track_index], observations[observation_index])
            assigned_observations.add(observation_index)
            assigned_tracks.add(track_index)

        relocation_pairs = sorted(
            (
                (
                    math.hypot(obs.x - self._x(track), obs.y - self._y(track)),
                    oi,
                    ti,
                )
                for oi, obs in enumerate(observations)
                for ti, track in enumerate(self._tracks)
                if oi not in assigned_observations
                and ti not in assigned_tracks
                and obs.color == self._summary(track).color
                and obs.stamp - self._last_seen(track) <= self.relocation_max_age
            ),
            key=lambda item: item[0],
        )
        for distance, observation_index, track_index in relocation_pairs:
            if distance > self.relocation_distance:
                break
            if (
                observation_index in assigned_observations
                or track_index in assigned_tracks
            ):
                continue
            self._replace_history(
                self._tracks[track_index], observations[observation_index]
            )
            assigned_observations.add(observation_index)
            assigned_tracks.add(track_index)
        for index, observation in enumerate(observations):
            if index in assigned_observations:
                continue
            track = _Track(self._next_id)
            self._next_id += 1
            self._append(track, observation)
            self._tracks.append(track)
        return tuple(self._summary(track) for track in self._tracks)

    def active(self, maximum_age: float) -> tuple[Cylinder, ...]:
        """Return tracks observed recently relative to the newest input."""
        if maximum_age <= 0.0:
            raise ValueError("maximum_age must be positive")
        if self._latest_stamp is None:
            return ()
        return tuple(
            item
            for item in (self._summary(track) for track in self._tracks)
            if self._latest_stamp - item.last_seen <= maximum_age
        )

    def envelope(
        self,
        minimum_observations: int = 1,
        maximum_age: float | None = None,
    ) -> Circle | None:
        cylinders = (
            self.active(maximum_age)
            if maximum_age is not None
            else tuple(self._summary(track) for track in self._tracks)
        )
        points = [
            (item.x, item.y)
            for item in cylinders
            if item.observation_count >= minimum_observations
        ]
        return minimum_enclosing_circle(points)

    def clear(self) -> None:
        """Remove all accumulated tracks and restart deterministic IDs."""
        self._tracks.clear()
        self._next_id = 1
        self._latest_stamp = None

    def _append(self, track: _Track, observation: Observation) -> None:
        track.observations.append(observation)
        while len(track.observations) > self.history_size:
            track.observations.popleft()

    def _replace_history(self, track: _Track, observation: Observation) -> None:
        """Move a briefly missing same-color track without averaging positions."""
        track.observations.clear()
        self._append(track, observation)

    @staticmethod
    def _x(track: _Track) -> float:
        return median(item.x for item in track.observations)

    @staticmethod
    def _y(track: _Track) -> float:
        return median(item.y for item in track.observations)

    @staticmethod
    def _last_seen(track: _Track) -> float:
        return max(item.stamp for item in track.observations)

    def _summary(self, track: _Track) -> Cylinder:
        items = list(track.observations)
        color = Counter(item.color for item in items).most_common(1)[0][0]
        colored = [item for item in items if item.color == color]
        return Cylinder(
            cylinder_id=track.cylinder_id,
            x=self._x(track),
            y=self._y(track),
            color=color,
            confidence=median(item.confidence for item in colored),
            observation_count=len(items),
            last_seen=max(item.stamp for item in items),
        )


def minimum_enclosing_circle(points: list[tuple[float, float]]) -> Circle | None:
    """Return the deterministic minimum circle for a small point set."""
    if not points:
        return None
    best: Circle | None = None
    candidates = [Circle(x, y, 0.0) for x, y in points]
    for index, left in enumerate(points):
        for right in points[index + 1 :]:
            candidates.append(_diameter_circle(left, right))
    for first in range(len(points)):
        for second in range(first + 1, len(points)):
            for third in range(second + 1, len(points)):
                circle = _circumcircle(points[first], points[second], points[third])
                if circle is not None:
                    candidates.append(circle)
    for circle in candidates:
        contains_all = all(
            math.hypot(x - circle.x, y - circle.y) <= circle.radius + 1e-9
            for x, y in points
        )
        if contains_all:
            if best is None or circle.radius < best.radius:
                best = circle
    return best


def _diameter_circle(left: tuple[float, float], right: tuple[float, float]) -> Circle:
    x = (left[0] + right[0]) / 2.0
    y = (left[1] + right[1]) / 2.0
    return Circle(x, y, math.hypot(left[0] - right[0], left[1] - right[1]) / 2.0)


def _circumcircle(
    a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]
) -> Circle | None:
    determinant = 2.0 * (
        a[0] * (b[1] - c[1])
        + b[0] * (c[1] - a[1])
        + c[0] * (a[1] - b[1])
    )
    if abs(determinant) < 1e-12:
        return None
    aa = a[0] ** 2 + a[1] ** 2
    bb = b[0] ** 2 + b[1] ** 2
    cc = c[0] ** 2 + c[1] ** 2
    x = (aa * (b[1] - c[1]) + bb * (c[1] - a[1]) + cc * (a[1] - b[1])) / determinant
    y = (aa * (c[0] - b[0]) + bb * (a[0] - c[0]) + cc * (b[0] - a[0])) / determinant
    return Circle(x, y, math.hypot(x - a[0], y - a[1]))
