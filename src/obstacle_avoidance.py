"""Choose a clear route around an obstacle and back to the goal line."""

import math

from occupancy_grid import OccupancyGrid


class ObstacleAvoidance:
    """Plan a three-waypoint bypass using the current occupancy grid."""

    # Run States
    GO_TO_GOAL = "GO_TO_GOAL"
    MOVE_ASIDE = "MOVE_ASIDE"
    PASS_OBSTACLE = "PASS_OBSTACLE"
    RETURN_TO_PATH = "RETURN_TO_PATH"
    BLOCKED = "BLOCKED"

    # Distance ahead of the robot at which an obstacle begins bypass planning.
    OBSTACLE_TRIGGER_DISTANCE = 1.5

    # Half-width of the planned forward path used when evaluating obstacle clearance.
    PATH_HALF_WIDTH = 0.40

    # Lateral offsets tried when choosing a bypass route, smallest first.
    # The first offset that keeps ROBOT_CLEARANCE from every mapped point wins.
    SIDE_STEP_DISTANCES = (0.3, 0.5, 0.7, 0.9, 1.1)

    # Forward distance used for the temporary pass waypoint.
    PASS_DISTANCE = 0.9

    # Forward distance used to return to the goal line after passing the obstacle.
    RETURN_DISTANCE = 0.3

    # Approximate half-foot robot width plus a three-inch safety margin.
    ROBOT_CLEARANCE = 0.4

    # Extra clearance demanded at planning time so waypoint tolerance and
    # scan noise do not push the executed route below ROBOT_CLEARANCE.
    PLANNING_CLEARANCE_MARGIN = 0.15

    # Distance tolerance for deciding whether the current bypass waypoint has been reached.
    WAYPOINT_TOLERANCE = 0.10

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
        self.waypoint: tuple[float, float] | None = None
        self._waypoints: list[tuple[str, tuple[float, float]]] = []
        self._last_reported_state: str | None = None
        self._set_state(self.GO_TO_GOAL)

    def update(self, current_position: tuple[float, float]) -> str:
        """Find a nearby obstacle or advance to the next bypass waypoint."""
        if self.state == self.GO_TO_GOAL:
            self._start_bypass_if_needed(current_position)
        elif self.state == self.BLOCKED:
            return self.state
        elif self.waypoint is not None and math.dist(
            current_position, self.waypoint
        ) <= self.WAYPOINT_TOLERANCE:
            self._advance_waypoint()

        return self.state

    def _start_bypass_if_needed(
        self,
        current_position: tuple[float, float],
    ) -> None:
        obstacle = self.occupancy_grid.closest_obstacle_ahead(
            current_position,
            self.goal_heading,
            self.PATH_HALF_WIDTH,
        )
        if obstacle is None:
            return

        along_path = self._dot(
            (obstacle[0] - current_position[0], obstacle[1] - current_position[1]),
            self._forward,
        )
        distance_to_obstacle = math.dist(current_position, obstacle)
        print(
            "\n[debug] Obstacle candidate\n"
            f"\tposition: {obstacle}\n"
            f"\tdistance_to_obstacle: {distance_to_obstacle:.3f} m\n"
            f"\talong_path: {along_path:.3f} m\n"
            f"\tgoal_distance: {self.goal_distance:.3f} m"
        )
        if along_path > self.OBSTACLE_TRIGGER_DISTANCE:
            print(
                "\n[debug] Ignoring obstacle\n"
                f"\talong_path: {along_path:.3f} m\n"
                f"\ttrigger_distance: "
                f"{self.OBSTACLE_TRIGGER_DISTANCE:.3f} m"
            )
            return

        if along_path > self.goal_distance:
            goal_clearance = math.dist(obstacle, self.goal_position)
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

        if not self._choose_bypass(current_position, obstacle):
            print(
                "\n[debug] Both bypass routes failed\n"
                f"\trequired_clearance: {self.ROBOT_CLEARANCE:.3f} m"
            )
            self._set_state(self.BLOCKED)
            return

        self._advance_waypoint()

    def _choose_bypass(
        self,
        current_position: tuple[float, float],
        obstacle: tuple[float, float],
    ) -> bool:
        forward = self._forward
        left = (-forward[1], forward[0])
        current_progress = self._dot(
            (
                current_position[0] - self.start_position[0],
                current_position[1] - self.start_position[1],
            ),
            forward,
        )
        obstacle_progress = self._dot(
            (
                obstacle[0] - self.start_position[0],
                obstacle[1] - self.start_position[1],
            ),
            forward,
        )
        pass_progress = min(
            self.goal_distance,
            max(current_progress, obstacle_progress) + self.PASS_DISTANCE,
        )
        return_progress = min(
            self.goal_distance,
            max(pass_progress, obstacle_progress + self.RETURN_DISTANCE),
        )

        required_clearance = self.ROBOT_CLEARANCE + self.PLANNING_CLEARANCE_MARGIN
        side_routes = []
        for side in (1.0, -1.0):
            for offset in self.SIDE_STEP_DISTANCES:
                side_vector = (side * left[0] * offset, side * left[1] * offset)
                side_step = (
                    current_position[0] + side_vector[0],
                    current_position[1] + side_vector[1],
                )
                pass_point = (
                    self.start_position[0] + forward[0] * pass_progress + side_vector[0],
                    self.start_position[1] + forward[1] * pass_progress + side_vector[1],
                )
                return_point = (
                    self.start_position[0] + forward[0] * return_progress,
                    self.start_position[1] + forward[1] * return_progress,
                )
                route = [
                    (self.MOVE_ASIDE, side_step),
                    (self.PASS_OBSTACLE, pass_point),
                    (self.RETURN_TO_PATH, return_point),
                ]
                route_points = [current_position, *(point for _, point in route)]
                clearances = [
                    self.occupancy_grid.segment_clearance(start, end)
                    for start, end in zip(route_points, route_points[1:])
                ]
                route_clearance = min(clearances)
                print(
                    f"\n[debug] Bypass candidate\n"
                    f"\tside: {side:+.0f}\n"
                    f"\toffset: {offset:.2f}\n"
                    f"\twaypoints: {[point for _, point in route]}\n"
                    f"\tclearance: {route_clearance:.3f} m"
                )
                if route_clearance < required_clearance:
                    print(
                        f"\n[debug] Rejected bypass candidate\n"
                        f"\tside: {side:+.0f}\n"
                        f"\toffset: {offset:.2f}\n"
                        f"\tclearance: {route_clearance:.3f} m\n"
                        f"\trequired_clearance: {required_clearance:.3f} m"
                    )
                    continue

                side_routes.append((offset, route_clearance, route))
                break

        if not side_routes:
            return False

        _, _, best_route = min(
            side_routes,
            key=lambda candidate: (candidate[0], -candidate[1]),
        )
        self._waypoints = best_route
        return True

    def _set_state(self, state: str) -> None:
        """Announce a state transition once when that action begins."""
        if self.state == state and self._last_reported_state == state:
            return

        self.state = state
        self._last_reported_state = state

        messages = {
            self.GO_TO_GOAL: "Starting normal path following toward the goal.",
            self.MOVE_ASIDE: "Obstacle detected; moving aside to clear the path.",
            self.PASS_OBSTACLE: "Passing the obstacle and continuing around it.",
            self.RETURN_TO_PATH: "Returning to the original path toward the goal.",
            self.BLOCKED: "Route is blocked; stopping navigation.",
        }
        print(
            "\n[debug] Obstacle avoidance\n"
            f"\t{messages.get(state, f'Entering state {state}.')}"
        )

    def _advance_waypoint(self) -> None:
        if not self._waypoints:
            self._set_state(self.GO_TO_GOAL)
            self.waypoint = None
            return

        next_state, next_waypoint = self._waypoints.pop(0)
        self._set_state(next_state)
        self.waypoint = next_waypoint

    @property
    def _forward(self) -> tuple[float, float]:
        return math.cos(self.goal_heading), math.sin(self.goal_heading)

    @staticmethod
    def _dot(
        first: tuple[float, float],
        second: tuple[float, float],
    ) -> float:
        return first[0] * second[0] + first[1] * second[1]
