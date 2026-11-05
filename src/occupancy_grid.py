"""Build a sparse 2D occupancy grid from the latest LiDAR scan."""

import math
from typing import Protocol, Sequence


class LaserScanData(Protocol):
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: Sequence[float]


class OccupancyGrid:
    """Store occupied map cells in odometry (world) coordinates."""

    def __init__(self, resolution: float = 0.05):
        if not math.isfinite(resolution) or resolution <= 0.0:
            raise ValueError("resolution must be finite and greater than zero")

        self.resolution = resolution
        self.occupied_cells: dict[tuple[int, int], tuple[float, float]] = {}
        self.has_scan = False

    def update_from_scan(
        self,
        scan: LaserScanData,
        position: tuple[float, float],
        orientation: float,
    ) -> bool:
        """Replace the grid with valid obstacle returns from one scan."""
        self.occupied_cells.clear()
        self.has_scan = False

        values = (position[0], position[1], orientation, scan.angle_min)
        if not all(math.isfinite(value) for value in values):
            print(
                "\n[debug] Rejected LiDAR scan\n"
                "\treason: non-finite robot pose or angle_min\n"
                f"\tposition: {position}\n"
                f"\torientation: {orientation}\n"
                f"\tangle_min: {scan.angle_min}"
            )
            return False
        if not math.isfinite(scan.angle_increment):
            print(
                "\n[debug] Rejected LiDAR scan\n"
                "\treason: non-finite angle_increment\n"
                f"\tangle_increment: {scan.angle_increment}"
            )
            return False
        if len(scan.ranges) > 1 and scan.angle_increment == 0.0:
            print(
                "\n[debug] Rejected LiDAR scan\n"
                "\treason: angle_increment is zero\n"
                f"\tranges: {len(scan.ranges)}"
            )
            return False

        for index, distance in enumerate(scan.ranges):
            if (
                not math.isfinite(distance)
                or distance < scan.range_min
                or distance > scan.range_max
            ):
                continue

            angle = scan.angle_min + index * scan.angle_increment
            world_x = position[0] + distance * math.cos(orientation + angle)
            world_y = position[1] + distance * math.sin(orientation + angle)
            cell = self._world_to_cell(world_x, world_y)
            self.occupied_cells[cell] = (world_x, world_y)

        self.has_scan = True
        return True

    def closest_obstacle_ahead(
        self,
        position: tuple[float, float],
        heading: float,
        corridor_half_width: float,
    ) -> tuple[float, float] | None:
        """Return the nearest mapped point in a forward-facing corridor."""
        forward = (math.cos(heading), math.sin(heading))
        nearest_point = None
        nearest_distance = math.inf

        for point in self.occupied_cells.values():
            relative = (point[0] - position[0], point[1] - position[1])
            distance_ahead = self._dot(relative, forward)
            distance_to_side = abs(
                relative[0] * -forward[1] + relative[1] * forward[0]
            )
            if (
                0.0 < distance_ahead < nearest_distance
                and distance_to_side <= corridor_half_width
            ):
                nearest_point = point
                nearest_distance = distance_ahead

        return nearest_point

    def segment_clearance(
        self,
        start: tuple[float, float],
        end: tuple[float, float],
    ) -> float:
        """Return the closest mapped obstacle distance from a line segment."""
        if not self.has_scan:
            return 0.0
        if not self.occupied_cells:
            return math.inf

        return min(
            self._distance_to_segment(point, start, end)
            for point in self.occupied_cells.values()
        )

    def _world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (
            math.floor(x / self.resolution),
            math.floor(y / self.resolution),
        )

    @staticmethod
    def _dot(
        first: tuple[float, float],
        second: tuple[float, float],
    ) -> float:
        return first[0] * second[0] + first[1] * second[1]

    @staticmethod
    def _distance_to_segment(
        point: tuple[float, float],
        start: tuple[float, float],
        end: tuple[float, float],
    ) -> float:
        segment = (end[0] - start[0], end[1] - start[1])
        segment_length_squared = segment[0] ** 2 + segment[1] ** 2
        if segment_length_squared == 0.0:
            return math.dist(point, start)

        point_from_start = (point[0] - start[0], point[1] - start[1])
        projection = OccupancyGrid._dot(point_from_start, segment)
        fraction = min(1.0, max(0.0, projection / segment_length_squared))
        nearest_point = (
            start[0] + fraction * segment[0],
            start[1] + fraction * segment[1],
        )
        return math.dist(point, nearest_point)
