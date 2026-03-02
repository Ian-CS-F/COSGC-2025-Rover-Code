import heapq

from heightmap import Heightmap

# Maps a cardinal move direction to the matching DirectionalSlopes property name
_DIR_SLOPE: dict[tuple[int, int], str] = {
    (-1, 0): "yNeg",
    (1, 0):  "yPos",
    (0, 1):  "xPos",
    (0, -1): "xNeg",
}

# State: (row, col, dr, dc) — position plus the direction we arrived from
State = tuple[int, int, int, int]


class AStar:
    """
    A* pathfinding on a Heightmap.

    Movement cost = |slope in the travel direction| + turn_penalty (if direction changes).
    Cells are passable when their height value is 0.
    Supports 4-directional or 8-directional movement.
    """

    def __init__(
        self,
        heightmap: Heightmap,
        diagonal: bool = False,
        turn_penalty: float = 1.0,
        max_height_diff: int = 1,
    ):
        self.heightmap = heightmap
        self.rows = heightmap.rows
        self.cols = heightmap.cols
        self.diagonal = diagonal
        self.turn_penalty = turn_penalty
        self.max_height_diff = max_height_diff

    def find_path(
        self, start: tuple[int, int], goal: tuple[int, int]
    ) -> list[tuple[int, int]] | None:
        """
        Find the lowest-cost path from start to goal.

        Args:
            start: (row, col) of the starting cell.
            goal:  (row, col) of the destination cell.

        Returns:
            Ordered list of (row, col) tuples from start to goal,
            or None if no path exists.
        """
        if not self._passable(start) or not self._passable(goal):
            return None

        # Sentinel direction (0, 0) means "no prior direction" (start cell)
        start_state: State = (start[0], start[1], 0, 0)

        open_heap: list[tuple[float, State]] = []
        heapq.heappush(open_heap, (0.0, start_state))

        came_from: dict[State, State | None] = {start_state: None}
        g_score: dict[State, float] = {start_state: 0.0}

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        if self.diagonal:
            directions += [(-1, -1), (-1, 1), (1, -1), (1, 1)]

        while open_heap:
            _, current = heapq.heappop(open_heap)
            r, c, prev_dr, prev_dc = current

            if (r, c) == goal:
                return self._reconstruct(came_from, current)

            for dr, dc in directions:
                nr, nc = r + dr, c + dc
                if not self._passable((nr, nc), from_pos=(r, c)):
                    continue

                neighbor: State = (nr, nc, dr, dc)
                step_cost = self._cost((r, c), (dr, dc), (prev_dr, prev_dc))
                tentative_g = g_score[current] + step_cost

                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self._heuristic((nr, nc), goal)
                    heapq.heappush(open_heap, (f, neighbor))

        return None  # No path found

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _passable(self, pos: tuple[int, int], from_pos: tuple[int, int] | None = None) -> bool:
        r, c = pos
        if not (0 <= r < self.rows and 0 <= c < self.cols):
            return False
        if from_pos is not None:
            diff = abs(self.heightmap.heights[r][c] - self.heightmap.heights[from_pos[0]][from_pos[1]])
            if diff > self.max_height_diff:
                return False
        return True

    def _cost(
        self,
        pos: tuple[int, int],
        move_dir: tuple[int, int],
        prev_dir: tuple[int, int],
    ) -> float:
        slope = self._slope_at(pos, move_dir)
        turning = prev_dir != (0, 0) and prev_dir != move_dir
        return abs(slope) + (self.turn_penalty if turning else 0.0)

    def _slope_at(self, pos: tuple[int, int], direction: tuple[int, int]) -> float:
        slopes = self.heightmap.slopes[pos[0]][pos[1]]
        if slopes is None:
            return 0.0
        attr = _DIR_SLOPE.get(direction)
        return getattr(slopes, attr) if attr else 0.0

    def _heuristic(self, a: tuple[int, int], b: tuple[int, int]) -> float:
        # Octile distance — admissible for both 4- and 8-directional movement
        dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
        return 1.0 * abs(dr - dc) + 1.4142 * min(dr, dc)

    def _reconstruct(
        self,
        came_from: dict[State, State | None],
        current: State,
    ) -> list[tuple[int, int]]:
        path = []
        while current is not None:
            path.append((current[0], current[1]))  # strip direction, keep position
            current = came_from[current]
        path.reverse()
        return path
