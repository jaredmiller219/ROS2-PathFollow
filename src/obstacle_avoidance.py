"""Measure a blocking obstacle and plan a turn-drive maneuver around it."""

import math

from occupancy_grid import ObstacleExtent, OccupancyGrid


class ObstacleAvoidance:
    """Plan a sideways bypass using the measured width of the obstacle."""

    # Run States
    GO_TO_GOAL = "GO_TO_GOAL"
    BLOCKED = "BLOCKED"

    # Maneuver leg phases executed one at a time by the controller.
    TURN_ASIDE = "TURN_ASIDE"
    DRIVE_ASIDE = "DRIVE_ASIDE"
    TURN_FORWARD = "TURN_FORWARD"
    PASS_OBSTACLE = "PASS_OBSTACLE"
    TURN_TO_PATH = "TURN_TO_PATH"
    RETURN_TO_PATH = "RETURN_TO_PATH"

    # Distance ahead of the robot at which an obstacle begins bypass planning.
    OBSTACLE_TRIGGER_DISTANCE = 1.5

    # Half-width of the planned forward path used when evaluating obstacle clearance.
    PATH_HALF_WIDTH = 0.40

    # Lateral distance the robot needs beside an obstacle edge.
    ROBOT_RADIUS = 0.36

    # Extra clearance beyond the robot distance (~4 inches).
    EDGE_MARGIN = 0.10

    # Minimum sideways travel so a bypass maneuver always happens.
    MIN_ASIDE_DISTANCE = 0.15

    # Depth of the along-track band used when measuring obstacle width.
    OBSTACLE_DEPTH = 0.35

    # Extra lateral reach beyond the path corridor when measuring obstacle width.
    WIDTH_MARGIN = 0.6

    # Approximate half-foot robot width plus a three-inch safety margin.
    ROBOT_CLEARANCE = 0.4

    # Extra clearance demanded at planning time so scan noise does not
    # push the executed route below ROBOT_CLEARANCE.
    PLANNING_CLEARANCE_MARGIN = 0.05

    def __init__(
        self,
        occupancy_grid: OccupancyGrid,
        start_position: tuple[float, float],
        goal_position: tuple[float, float],
    ):
        self.occupancy_grid = occupancy_grid
        self.start_position = start_position
        self.goal_position = goal_position
        self.goal_heading = math.atan2(
            goal_position[1] - start_position[1],
            goal_position[0] - start_position[0],
        )
        self.goal_distance = math.dist(start_position, goal_position)
        self.state = self.GO_TO_GOAL
        self._legs: list[tuple[str, float, float]] = []
        self._current_leg: tuple[str, float, float] | None = None
        self.leg_index = 0
        self._last_reported_state: str | None = None
        self._set_state(self.GO_TO_GOAL)

    @property
    def current_leg(self) -> tuple[str, float, float] | None:
        """Return the maneuver leg currently being executed, if any."""
        return self._current_leg

    def update(self, current_position: tuple[float, float]) -> str:
        """Find a blocking obstacle and plan the bypass maneuver."""
        if self.state == self.GO_TO_GOAL:
            self._start_bypass_if_needed(current_position)
        return self.state

    def finish_leg(self) -> None:
        """Advance to the next maneuver leg once the current one completes."""
        self._advance_leg()

    def _start_bypass_if_needed(
        self,
        current_position: tuple[float, float],
    ) -> None:
        extent = self.occupancy_grid.blocking_obstacle(
            current_position,
            self.goal_heading,
            self.PATH_HALF_WIDTH,
            self.OBSTACLE_TRIGGER_DISTANCE,
            depth=self.OBSTACLE_DEPTH,
            width_margin=self.WIDTH_MARGIN,
            lateral_origin=self.start_position,
        )
        if extent is None:
            return

        forward = self._forward
        obstacle_point = (
            current_position[0] + forward[0] * extent.along,
            current_position[1] + forward[1] * extent.along,
        )
        print(
            "\n[debug] Obstacle detected\n"
            f"\tapprox_position: {obstacle_point}\n"
            f"\tdistance_ahead: {extent.along:.3f} m\n"
            f"\twidth: {extent.lat_max - extent.lat_min:.3f} m\n"
            f"\tlat_min: {extent.lat_min:.3f} m\n"
            f"\tlat_max: {extent.lat_max:.3f} m\n"
            f"\tgoal_distance: {self.goal_distance:.3f} m"
        )

        if extent.along > self.goal_distance:
            goal_clearance = math.dist(obstacle_point, self.goal_position)
            if goal_clearance >= self.ROBOT_CLEARANCE:
                print(
                    "\n[debug] Ignoring obstacle beyond the goal\n"
                    f"\tgoal_clearance: {goal_clearance:.3f} m\n"
                    f"\trequired_clearance: {self.ROBOT_CLEARANCE:.3f} m"
                )
                return
            print(
                "\n[debug] Obstacle is beyond the goal but within required clearance\n"
                f"\tgoal_clearance: {goal_clearance:.3f} m\n"
                f"\trequired_clearance: {self.ROBOT_CLEARANCE:.3f} m"
            )

        if not self._choose_bypass(current_position, extent):
            print(
                "\n[debug] Both bypass routes failed\n"
                f"\trequired_clearance: {self.ROBOT_CLEARANCE:.3f} m"
            )
            self._set_state(self.BLOCKED)
            return

        self._advance_leg()

    def _choose_bypass(
        self,
        current_position: tuple[float, float],
        extent: ObstacleExtent,
    ) -> bool:
        forward = self._forward
        left = (-forward[1], forward[0])
        robot_lateral = self._dot(
            (
                current_position[0] - self.start_position[0],
                current_position[1] - self.start_position[1],
            ),
            left,
        )
        current_progress = self._dot(
            (
                current_position[0] - self.start_position[0],
                current_position[1] - self.start_position[1],
            ),
            forward,
        )
        required_clearance = self.ROBOT_CLEARANCE + self.PLANNING_CLEARANCE_MARGIN
        remaining_distance = max(0.0, self.goal_distance - current_progress)
        pass_distance = min(
            extent.along_max + required_clearance + 0.05,
            remaining_distance,
        )
        return_progress = current_progress + pass_distance
        return_point = (
            self.start_position[0] + forward[0] * return_progress,
            self.start_position[1] + forward[1] * return_progress,
        )

        best_route = None
        best_clearance = -1.0
        for side in (1.0, -1.0):
            edge = extent.lat_max if side > 0 else extent.lat_min
            target_lat = edge + side * (self.ROBOT_RADIUS + self.EDGE_MARGIN)
            aside_distance = max(
                abs(target_lat - robot_lateral),
                self.MIN_ASIDE_DISTANCE,
            )
            aside_latitude = robot_lateral + side * aside_distance
            return_distance = max(abs(aside_latitude), self.MIN_ASIDE_DISTANCE)

            aside_point = (
                current_position[0] + side * left[0] * aside_distance,
                current_position[1] + side * left[1] * aside_distance,
            )
            pass_point = (
                aside_point[0] + forward[0] * pass_distance,
                aside_point[1] + forward[1] * pass_distance,
            )
            route_points = [
                current_position,
                aside_point,
                pass_point,
                return_point,
            ]
            clearances = [
                self.occupancy_grid.segment_clearance(start, end)
                for start, end in zip(route_points, route_points[1:])
            ]
            route_clearance = min(clearances)
            print(
                f"\n[debug] Bypass candidate\n"
                f"\tside: {side:+.0f}\n"
                f"\ttarget_lat: {target_lat:.3f} m\n"
                f"\taside_distance: {aside_distance:.3f} m\n"
                f"\tpass_distance: {pass_distance:.3f} m\n"
                f"\twaypoints: {[point for point in route_points]}\n"
                f"\tclearance: {route_clearance:.3f} m"
            )
            if route_clearance < required_clearance:
                print(
                    f"\n[debug] Rejected bypass candidate\n"
                    f"\tside: {side:+.0f}\n"
                    f"\tclearance: {route_clearance:.3f} m\n"
                    f"\trequired_clearance: {required_clearance:.3f} m"
                )
                continue

            if route_clearance > best_clearance:
                best_route = (side, aside_distance, return_distance)
                best_clearance = route_clearance

        if best_route is None:
            return False

        side, aside_distance, return_distance = best_route
        aside_heading = self.goal_heading + side * math.pi / 2
        return_heading = self.goal_heading - side * math.pi / 2
        self._legs = [
            (self.TURN_ASIDE, aside_heading, 0.0),
            (self.DRIVE_ASIDE, aside_heading, aside_distance),
            (self.TURN_FORWARD, self.goal_heading, 0.0),
            (self.PASS_OBSTACLE, self.goal_heading, pass_distance),
            (self.TURN_TO_PATH, return_heading, 0.0),
            (self.RETURN_TO_PATH, return_heading, return_distance),
            (self.TURN_FORWARD, self.goal_heading, 0.0),
        ]
        return True

    def _set_state(self, state: str) -> None:
        """Announce a state transition once when that action begins."""
        if self.state == state and self._last_reported_state == state:
            return

        self.state = state
        self._last_reported_state = state

        messages = {
            self.GO_TO_GOAL: "Driving toward the goal.",
            self.TURN_ASIDE: "Obstacle detected; turning toward the clearer side.",
            self.DRIVE_ASIDE: "Moving sideways to clear the obstacle edge.",
            self.TURN_FORWARD: "Turning to face forward again.",
            self.PASS_OBSTACLE: "Driving forward alongside the obstacle.",
            self.TURN_TO_PATH: "Turning back toward the original path.",
            self.RETURN_TO_PATH: "Returning to the original path line.",
            self.BLOCKED: "Route is blocked; stopping navigation.",
        }
        print(
            "\n[debug] Obstacle avoidance\n"
            f"\t{messages.get(state, f'Entering state {state}.')}"
        )

    def _advance_leg(self) -> None:
        if not self._legs:
            self._current_leg = None
            self._set_state(self.GO_TO_GOAL)
            return

        self._current_leg = self._legs.pop(0)
        self.leg_index += 1
        self._set_state(self._current_leg[0])

    @property
    def _forward(self) -> tuple[float, float]:
        return math.cos(self.goal_heading), math.sin(self.goal_heading)

    @staticmethod
    def _dot(
        first: tuple[float, float],
        second: tuple[float, float],
    ) -> float:
        return first[0] * second[0] + first[1] * second[1]
