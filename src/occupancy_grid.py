"""Build a sparse 2D occupancy grid from the latest LiDAR scan."""

import math
from typing import NamedTuple, Protocol, Sequence


class LaserScanData(Protocol):
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: Sequence[float]


class ObstacleExtent(NamedTuple):
    """Lateral span of the obstacle blocking the path, in path-frame meters."""

    along: float
    along_max: float
    lat_min: float
    lat_max: float


class OccupancyGrid:
    """Store occupied map cells in odometry (world) coordinates."""

    def __init__(self, resolution: float = 0.05):
        """Create a grid with the given cell width in meters (default 0.05 m)."""
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
        """Replace the grid with valid obstacle returns from one scan.

        Only the current scan is kept: routes are checked against what the
        LiDAR sees right now, with no memory of past scans.
        """
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
        view_half_angle: float,
        corridor_half_width: float,
    ) -> tuple[float, float] | None:
        """Return the nearest mapped point inside the forward view cone
        and within corridor_half_width of the path line."""
        forward = (math.cos(heading), math.sin(heading))
        nearest_point = None
        nearest_distance = math.inf

        for point in self.occupied_cells.values():
            relative = (point[0] - position[0], point[1] - position[1])
            distance = math.hypot(*relative)
            if distance == 0.0 or distance >= nearest_distance:
                continue

            bearing = math.atan2(relative[1], relative[0])
            angle_offset = math.atan2(
                math.sin(bearing - heading),
                math.cos(bearing - heading),
            )
            distance_to_side = abs(
                relative[0] * -forward[1] + relative[1] * forward[0]
            )
            if (
                abs(angle_offset) <= view_half_angle
                and distance_to_side <= corridor_half_width
            ):
                nearest_point = point
                nearest_distance = distance

        return nearest_point

    def blocking_obstacle(
        self,
        position: tuple[float, float],
        heading: float,
        corridor_half_width: float,
        lookahead: float,
        depth: float = 0.35,
        width_margin: float = 0.6,
        lateral_origin: tuple[float, float] | None = None,
    ) -> ObstacleExtent | None:
        """Return the lateral extent of the obstacle blocking the path corridor.

        Candidate cells must sit inside the forward corridor. The returned
        extent covers every mapped cell within ``depth`` meters past the
        nearest candidate, reaching ``width_margin`` beyond the corridor so
        wide obstacles are measured fully.
        """
        origin = position if lateral_origin is None else lateral_origin
        forward = (math.cos(heading), math.sin(heading))
        left = (-forward[1], forward[0])

        cells = []
        for point in self.occupied_cells.values():
            relative = (point[0] - position[0], point[1] - position[1])
            along = self._dot(relative, forward)
            lateral = self._dot(
                (point[0] - origin[0], point[1] - origin[1]),
                left,
            )
            cells.append((along, lateral))

        corridor = [
            (along, lateral)
            for along, lateral in cells
            if 0.0 < along <= lookahead
            and abs(lateral) <= corridor_half_width
        ]
        if not corridor:
            return None

        nearest = min(along for along, _ in corridor)
        lateral_limit = corridor_half_width + width_margin
        band = [
            (along, lateral)
            for along, lateral in cells
            if nearest - 0.05 <= along <= nearest + depth
            and abs(lateral) <= lateral_limit
        ]
        if not band:
            return None

        return ObstacleExtent(
            along=nearest,
            along_max=max(along for along, _ in band),
            lat_min=min(lateral for _, lateral in band),
            lat_max=max(lateral for _, lateral in band),
        )

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
