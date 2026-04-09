"""
Test Protocol: Navigation
=========================
Verifies A* pathfinding, slope calculations, and heightmap logic produce
correct results.  These are purely computational — no hardware needed.

Run on any machine:
    python3 "test_nav.py"
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "Python", "Control", "Nav"))

from astar import AStar            # type: ignore
from slope import DirectionalSlopes  # type: ignore
from heightmap import Heightmap    # type: ignore


# ── Helpers ───────────────────────────────────────────────────────────────────

def _flat_map(rows: int, cols: int) -> Heightmap:
    """Return a Heightmap with all heights = 0 and slopes computed."""
    hmap = Heightmap(rows, cols)
    hmap.compute_slopes()
    return hmap


def _wall_map(rows: int, cols: int, wall_row: int,
              gap_col: int | None = None) -> Heightmap:
    """
    Return a Heightmap with a horizontal wall at wall_row (height=100 cm).
    If gap_col is given, that column is left at height 0 (a passage through).
    """
    hmap = Heightmap(rows, cols)
    for c in range(cols):
        if c != gap_col:
            hmap.heights[wall_row][c] = 100
    hmap.compute_slopes()
    return hmap


def _steps_adjacent(path: list[tuple[int, int]]) -> bool:
    """Return True when every consecutive pair in path is exactly one cell apart."""
    for (r1, c1), (r2, c2) in zip(path, path[1:]):
        dr, dc = abs(r2 - r1), abs(c2 - c1)
        # 4-directional: exactly one of dr/dc is 1, the other 0
        # 8-directional: both may be 1 (diagonal), but never jump 2+
        if dr > 1 or dc > 1 or (dr == 0 and dc == 0):
            return False
    return True


# ═════════════════════════════════════════════════════════════════════════════
# A* — basic path-finding
# ═════════════════════════════════════════════════════════════════════════════

def test_astar_finds_path() -> bool:
    """
    5×5 open flat grid, start=(0,0), goal=(4,4).
    A* must return a non-None path that starts at (0,0) and ends at (4,4).
    """
    print("TEST: A* finds a path on a 5×5 flat grid…")
    hmap    = _flat_map(5, 5)
    planner = AStar(hmap, diagonal=False, turn_penalty=1.5, max_height_diff=3)
    path    = planner.find_path((0, 0), (4, 4))
    passed  = (
        path is not None
        and len(path) > 0
        and path[0]  == (0, 0)
        and path[-1] == (4, 4)
    )
    print(f"  Path length: {len(path) if path else 'None'}  "
          f"start={path[0] if path else '?'}  end={path[-1] if path else '?'}  "
          f"→  {'PASS' if passed else 'FAIL'}")
    return passed


def test_astar_path_all_steps_adjacent() -> bool:
    """
    Every consecutive pair of cells in the path must be exactly one cell apart
    (no diagonal jumps in 4-directional mode, no skips of 2+).
    """
    print("TEST: A* path — every step is exactly one cell apart…")
    hmap    = _flat_map(6, 6)
    planner = AStar(hmap, diagonal=False, turn_penalty=0.0, max_height_diff=3)
    path    = planner.find_path((0, 0), (5, 5))
    if path is None:
        print("  FAIL — no path returned")
        return False
    passed = _steps_adjacent(path)
    print(f"  Path length: {len(path)}  All steps adjacent: {passed}  "
          f"→  {'PASS' if passed else 'FAIL (illegal jump detected)'}")
    return passed


def test_astar_path_length_straight_line() -> bool:
    """
    On a flat 1×10 grid from (0,0) to (0,9), the shortest path has exactly
    10 cells (start inclusive).  This verifies A* isn't adding redundant steps.
    """
    print("TEST: A* path length on a flat 1×10 straight line…")
    hmap    = _flat_map(1, 10)
    planner = AStar(hmap, diagonal=False, turn_penalty=0.0, max_height_diff=3)
    path    = planner.find_path((0, 0), (0, 9))
    expected = 10  # (0,0) … (0,9) inclusive
    passed  = path is not None and len(path) == expected
    print(f"  Length: {len(path) if path else 'None'}  Expected: {expected}  "
          f"→  {'PASS' if passed else 'FAIL'}")
    return passed


def test_astar_avoids_obstacles() -> bool:
    """
    5×5 grid with a horizontal wall at row 2 (height=100 cm) except col 4.
    Start=(0,0), goal=(4,0).  A* must route through the gap at col 4.
    Verified by checking no path cell sits inside the solid wall.
    """
    print("TEST: A* routes around a wall with a single gap…")
    # Wall at row 2, cols 0–3; gap at col 4
    hmap    = _wall_map(5, 5, wall_row=2, gap_col=4)
    planner = AStar(hmap, diagonal=False, turn_penalty=0.0, max_height_diff=3)
    path    = planner.find_path((0, 0), (4, 0))
    if path is None:
        print("  FAIL — no path returned")
        return False
    wall_cells = {(2, c) for c in range(4)}   # solid part of the wall
    hits_wall  = any(cell in wall_cells for cell in path)
    passed = not hits_wall
    print(f"  Path length: {len(path)}  Hits wall: {hits_wall}  "
          f"→  {'PASS' if passed else 'FAIL (path went through wall)'}")
    return passed


def test_astar_returns_none_when_blocked() -> bool:
    """
    3×3 grid with a full solid wall at row 1 (all three columns, height=100 cm).
    Start=(0,1) above, goal=(2,1) below.  No path should exist.
    """
    print("TEST: A* returns None when goal is completely blocked…")
    hmap    = _wall_map(3, 3, wall_row=1, gap_col=None)  # no gap
    planner = AStar(hmap, diagonal=False, turn_penalty=1.5, max_height_diff=3)
    path    = planner.find_path((0, 1), (2, 1))
    passed  = path is None
    print(f"  Result: {'None (correct)' if path is None else f'path of length {len(path)} (wrong)'}  "
          f"→  {'PASS' if passed else 'FAIL'}")
    return passed


def test_astar_start_equals_goal() -> bool:
    """
    When start and goal are the same cell, A* should return a single-element
    path containing just that cell.
    """
    print("TEST: A* start == goal returns single-cell path…")
    hmap    = _flat_map(5, 5)
    planner = AStar(hmap, diagonal=False, turn_penalty=1.5, max_height_diff=3)
    path    = planner.find_path((2, 2), (2, 2))
    passed  = path is not None and len(path) == 1 and path[0] == (2, 2)
    print(f"  Path: {path}  →  {'PASS' if passed else 'FAIL'}")
    return passed


def test_astar_diagonal_shorter() -> bool:
    """
    With diagonal=True, A* finds a shorter path than the 4-directional L-shaped
    route on a flat 5×5 grid.  Diagonal path to (4,4) should be 5 cells; the
    4-directional minimum is 9 cells.
    """
    print("TEST: A* diagonal mode finds shorter path than 4-directional…")
    hmap      = _flat_map(5, 5)
    plan_4dir = AStar(hmap, diagonal=False, turn_penalty=0.0, max_height_diff=3)
    plan_8dir = AStar(hmap, diagonal=True,  turn_penalty=0.0, max_height_diff=3)
    path_4    = plan_4dir.find_path((0, 0), (4, 4))
    path_8    = plan_8dir.find_path((0, 0), (4, 4))
    passed    = (
        path_4 is not None
        and path_8 is not None
        and len(path_8) < len(path_4)
        and path_8[0]  == (0, 0)
        and path_8[-1] == (4, 4)
    )
    len4 = len(path_4) if path_4 else "None"
    len8 = len(path_8) if path_8 else "None"
    print(f"  4-dir length: {len4}  8-dir length: {len8}  "
          f"→  {'PASS' if passed else 'FAIL'}")
    return passed


def test_astar_respects_max_height_diff() -> bool:
    """
    A 1×3 grid with a spike at the middle cell (height=100 cm, max_height_diff=3).
    The transition (0,0)→(0,1) has diff=100 > 3, so (0,2) is unreachable.

    A second sub-test uses a gentle slope (diff=2 each step) and confirms the
    path IS found.
    """
    print("TEST: A* blocks moves that exceed max_height_diff…")

    # Sub-test A: spike blocks passage
    hmap_spike = Heightmap(1, 3)
    hmap_spike.heights[0][0] = 0
    hmap_spike.heights[0][1] = 100   # diff=100 from (0,0) → blocked
    hmap_spike.heights[0][2] = 0
    hmap_spike.compute_slopes()
    planner_spike = AStar(hmap_spike, diagonal=False, max_height_diff=3)
    path_blocked = planner_spike.find_path((0, 0), (0, 2))

    # Sub-test B: gentle slope is traversable
    hmap_slope = Heightmap(1, 3)
    hmap_slope.heights[0][0] = 0
    hmap_slope.heights[0][1] = 2    # diff=2 ≤ 3 → OK
    hmap_slope.heights[0][2] = 4    # diff from prev=2 ≤ 3 → OK
    hmap_slope.compute_slopes()
    planner_slope = AStar(hmap_slope, diagonal=False, max_height_diff=3)
    path_open = planner_slope.find_path((0, 0), (0, 2))

    passed = path_blocked is None and path_open is not None
    print(f"  Spike (diff=100): {'None ✓' if path_blocked is None else 'found (wrong)'}  "
          f"Slope (diff=2): {'found ✓' if path_open is not None else 'None (wrong)'}  "
          f"→  {'PASS' if passed else 'FAIL'}")
    return passed


def test_astar_out_of_bounds_goal() -> bool:
    """
    Goal outside the grid bounds — A* must return None without raising an exception.
    """
    print("TEST: A* returns None for out-of-bounds goal…")
    hmap    = _flat_map(5, 5)
    planner = AStar(hmap, diagonal=False, turn_penalty=1.5, max_height_diff=3)
    try:
        path   = planner.find_path((0, 0), (99, 99))
        passed = path is None
    except Exception as e:
        print(f"  FAIL — raised exception: {e}")
        return False
    print(f"  Result: {'None (correct)' if path is None else 'path returned (wrong)'}  "
          f"→  {'PASS' if passed else 'FAIL'}")
    return passed


# ═════════════════════════════════════════════════════════════════════════════
# Heightmap — slope calculations
# ═════════════════════════════════════════════════════════════════════════════

def test_slope_flat_surface() -> bool:
    """
    A completely flat heightmap (all heights = 0) must have slope = 0.0
    in every direction for every cell after compute_slopes().
    """
    print("TEST: All slopes are 0.0 on a flat heightmap…")
    hmap = Heightmap(5, 5)   # defaults to all-zero heights
    hmap.compute_slopes()
    all_zero = True
    for r in range(hmap.rows):
        for c in range(hmap.cols):
            s = hmap.slopes[r][c]
            if s is None:
                all_zero = False
                break
            if any(abs(v) > 1e-9 for v in (s.yPos, s.xPos, s.yNeg, s.xNeg)):
                all_zero = False
                break
    print(f"  All slopes zero: {all_zero}  →  {'PASS' if all_zero else 'FAIL'}")
    return all_zero


def test_slope_known_incline() -> bool:
    """
    A 3×1 column where heights are [0, 10, 20] cm (10 cm/cell incline).

    compute_slopes() should give:
        Cell (0,0): yPos = +10  (rising toward row 1)
        Cell (1,0): yPos = +10  (rising toward row 2)
                    yNeg = -10  (falling back toward row 0)
        Cell (2,0): yNeg = -10  (falling back toward row 1)

    Slope formula: (h_neighbour - h_current) / 1  [cells are 1 unit apart]
    """
    print("TEST: Slope values match a known 10 cm/cell incline…")
    hmap = Heightmap(3, 1)
    hmap.heights[0][0] = 0
    hmap.heights[1][0] = 10
    hmap.heights[2][0] = 20
    hmap.compute_slopes()

    s0 = hmap.slopes[0][0]
    s1 = hmap.slopes[1][0]
    s2 = hmap.slopes[2][0]

    checks = [
        ("(0,0).yPos == +10", s0 is not None and abs(s0.yPos - 10.0)  < 1e-9),
        ("(1,0).yPos == +10", s1 is not None and abs(s1.yPos - 10.0)  < 1e-9),
        ("(1,0).yNeg == -10", s1 is not None and abs(s1.yNeg - (-10.0)) < 1e-9),
        ("(2,0).yNeg == -10", s2 is not None and abs(s2.yNeg - (-10.0)) < 1e-9),
    ]
    for label, ok in checks:
        print(f"  {label}: {'✓' if ok else '✗'}")
    passed = all(ok for _, ok in checks)
    print(f"  →  {'PASS' if passed else 'FAIL'}")
    return passed


def test_slope_boundary_cells_use_self() -> bool:
    """
    Boundary cells have no neighbour on one or more sides.  compute_slopes()
    must not raise an IndexError and must produce slope = 0.0 for the missing
    direction (neighbour height defaults to the cell's own height).
    """
    print("TEST: Boundary cell slopes use self-height for missing neighbours (no IndexError)…")
    hmap = Heightmap(3, 3)
    hmap.heights[0][0] = 5   # top-left corner
    hmap.heights[2][2] = 5   # bottom-right corner
    try:
        hmap.compute_slopes()
    except Exception as e:
        print(f"  FAIL — compute_slopes raised: {e}")
        return False

    # Top-left (0,0): yNeg and xNeg directions have no real neighbour.
    # They should both be 0.0 (self - self = 0).
    s = hmap.slopes[0][0]
    passed = s is not None and abs(s.yNeg) < 1e-9 and abs(s.xNeg) < 1e-9
    print(f"  (0,0).yNeg={s.yNeg if s else '?'}  (0,0).xNeg={s.xNeg if s else '?'}  "
          f"→  {'PASS' if passed else 'FAIL'}")
    return passed


def test_slope_directional_asymmetry() -> bool:
    """
    With a step-up at the next cell, yPos should be positive and yNeg should
    be negative at the step cell — the slope direction is asymmetric.

    Grid (3×1):  heights = [0, 10, 10]

        Cell (1,0): yPos = 0   (flat ahead to row 2)
                    yNeg = -10 (drop back to row 0)
    """
    print("TEST: DirectionalSlopes are asymmetric (yPos ≠ yNeg at a step)…")
    hmap = Heightmap(3, 1)
    hmap.heights[0][0] = 0
    hmap.heights[1][0] = 10
    hmap.heights[2][0] = 10
    hmap.compute_slopes()

    s = hmap.slopes[1][0]
    passed = (
        s is not None
        and abs(s.yPos  -   0.0)  < 1e-9   # flat from (1,0) to (2,0)
        and abs(s.yNeg - (-10.0)) < 1e-9   # drop from (1,0) to (0,0)
    )
    print(f"  (1,0).yPos={s.yPos if s else '?'}  (1,0).yNeg={s.yNeg if s else '?'}  "
          f"→  {'PASS' if passed else 'FAIL'}")
    return passed


# ═════════════════════════════════════════════════════════════════════════════
# DirectionalSlopes — unit tests
# ═════════════════════════════════════════════════════════════════════════════

def test_directional_slopes_zero() -> bool:
    """
    Two identical heights → all four slope properties should be 0.0.
    """
    print("TEST: DirectionalSlopes all zero when heights are equal…")
    s = DirectionalSlopes(
        y_pos=((0, 5), (1, 5)),
        x_pos=((0, 5), (1, 5)),
        y_neg=((0, 5), (1, 5)),
        x_neg=((0, 5), (1, 5)),
    )
    passed = all(abs(v) < 1e-9 for v in (s.yPos, s.xPos, s.yNeg, s.xNeg))
    print(f"  yPos={s.yPos}  xPos={s.xPos}  yNeg={s.yNeg}  xNeg={s.xNeg}  "
          f"→  {'PASS' if passed else 'FAIL'}")
    return passed


def test_directional_slopes_rise() -> bool:
    """
    Neighbours 3 cm higher than current cell → slope = +3.0 in that direction.
    """
    print("TEST: DirectionalSlopes = +3.0 for a 3 cm rise…")
    s = DirectionalSlopes(
        y_pos=((0, 0), (1, 3)),    # 3 cm up in +y direction
        x_pos=((0, 0), (1, 3)),
        y_neg=((0, 0), (1, 0)),    # flat
        x_neg=((0, 0), (1, 0)),
    )
    passed = abs(s.yPos - 3.0) < 1e-9 and abs(s.xPos - 3.0) < 1e-9
    print(f"  yPos={s.yPos}  xPos={s.xPos}  →  {'PASS' if passed else 'FAIL'}")
    return passed


def test_directional_slopes_drop() -> bool:
    """
    Neighbours 5 cm lower → slope = -5.0 in that direction.
    """
    print("TEST: DirectionalSlopes = -5.0 for a 5 cm drop…")
    s = DirectionalSlopes(
        y_pos=((0, 10), (1, 5)),   # 5 cm drop in +y
        x_pos=((0, 10), (1, 5)),
        y_neg=((0, 10), (1, 10)),
        x_neg=((0, 10), (1, 10)),
    )
    passed = abs(s.yPos - (-5.0)) < 1e-9 and abs(s.xPos - (-5.0)) < 1e-9
    print(f"  yPos={s.yPos}  xPos={s.xPos}  →  {'PASS' if passed else 'FAIL'}")
    return passed


# ═════════════════════════════════════════════════════════════════════════════
# Heightmap — grid structure
# ═════════════════════════════════════════════════════════════════════════════

def test_heightmap_dimensions() -> bool:
    """
    Heightmap(rows, cols) must create a grid of exactly that size with all
    heights initialised to 0 and slopes initialised to None.
    """
    print("TEST: Heightmap initialises to correct dimensions and zero heights…")
    hmap = Heightmap(7, 11)
    rows_ok   = hmap.rows == 7 and len(hmap.heights) == 7
    cols_ok   = hmap.cols == 11 and all(len(row) == 11 for row in hmap.heights)
    zeros_ok  = all(hmap.heights[r][c] == 0
                    for r in range(7) for c in range(11))
    slopes_ok = all(hmap.slopes[r][c] is None
                    for r in range(7) for c in range(11))
    passed = rows_ok and cols_ok and zeros_ok and slopes_ok
    print(f"  rows={hmap.rows}  cols={hmap.cols}  "
          f"zeros={zeros_ok}  slopes_None={slopes_ok}  "
          f"→  {'PASS' if passed else 'FAIL'}")
    return passed


def test_heightmap_slopes_populated_after_compute() -> bool:
    """
    After compute_slopes(), every cell's slope entry must be a
    DirectionalSlopes instance (no more None values).
    """
    print("TEST: compute_slopes() populates every cell…")
    hmap = Heightmap(4, 4)
    hmap.compute_slopes()
    all_set = all(
        isinstance(hmap.slopes[r][c], DirectionalSlopes)
        for r in range(hmap.rows) for c in range(hmap.cols)
    )
    print(f"  All slopes set: {all_set}  →  {'PASS' if all_set else 'FAIL'}")
    return all_set


# ═════════════════════════════════════════════════════════════════════════════
# Integration — A* + heightmap together
# ═════════════════════════════════════════════════════════════════════════════

def test_astar_prefers_flat_path() -> bool:
    """
    Two routes connect start to goal: one over a ridge (height=3 cm) and one
    flat.  With max_height_diff=10 (both routes passable), A* should prefer
    the flat route because |slope| adds to movement cost.

    Layout (5×3 grid):
        Col 0: flat (height 0)  — direct route
        Col 1: ridge (height 3) — costlier

    Start=(0,0), Goal=(4,0), alternative via col 1 is longer and steeper.
    Since this is a straight grid, A* on the flat column is shorter anyway.
    We verify the path stays on the flat column.
    """
    print("TEST: A* prefers flat column over a ridge when both are passable…")
    hmap = Heightmap(5, 3)
    for r in range(5):
        hmap.heights[r][1] = 3   # ridge in middle column
    hmap.compute_slopes()
    planner = AStar(hmap, diagonal=False, turn_penalty=10.0, max_height_diff=10)
    path = planner.find_path((0, 0), (4, 0))
    if path is None:
        print("  FAIL — no path found")
        return False
    # The direct path down col 0 should be preferred (no slope cost, no turns)
    uses_ridge = any(c == 1 for _, c in path)
    passed = not uses_ridge
    print(f"  Path: {path}  Uses ridge col: {uses_ridge}  "
          f"→  {'PASS' if passed else 'FAIL (detoured through ridge)'}")
    return passed


def test_astar_replan_on_updated_map() -> bool:
    """
    Simulate a replan: first find a path on a clear map, then add a wall
    blocking that path, recompute slopes, and replan.  The second path must
    avoid the new obstacle.
    """
    print("TEST: A* replans correctly after heightmap is updated…")
    hmap    = _flat_map(5, 5)
    planner = AStar(hmap, diagonal=False, turn_penalty=0.0, max_height_diff=3)
    path1   = planner.find_path((0, 2), (4, 2))
    if path1 is None:
        print("  FAIL — first plan returned None")
        return False

    # Add a wall at row 2 across the middle — blocks the straight path
    for c in range(5):
        hmap.heights[2][c] = 100
    hmap.compute_slopes()

    path2 = planner.find_path((0, 2), (4, 2))
    # Wall is solid (no gap) → no path should exist in 4-directional mode
    passed = path1 is not None and path2 is None
    print(f"  Plan 1 length: {len(path1)}  "
          f"Plan 2 after wall: {'None (correct)' if path2 is None else f'{len(path2)} cells (wrong)'}  "
          f"→  {'PASS' if passed else 'FAIL'}")
    return passed


# ═════════════════════════════════════════════════════════════════════════════
# Entry point
# ═════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("=" * 60)
    print("  COSGC Rover — Navigation Unit Tests")
    print("  (no hardware required)")
    print("=" * 60)

    test_groups = [
        ("A* — path finding", [
            test_astar_finds_path,
            test_astar_path_all_steps_adjacent,
            test_astar_path_length_straight_line,
            test_astar_avoids_obstacles,
            test_astar_returns_none_when_blocked,
            test_astar_start_equals_goal,
            test_astar_diagonal_shorter,
            test_astar_respects_max_height_diff,
            test_astar_out_of_bounds_goal,
        ]),
        ("Heightmap — slope calculation", [
            test_slope_flat_surface,
            test_slope_known_incline,
            test_slope_boundary_cells_use_self,
            test_slope_directional_asymmetry,
        ]),
        ("DirectionalSlopes — unit", [
            test_directional_slopes_zero,
            test_directional_slopes_rise,
            test_directional_slopes_drop,
        ]),
        ("Heightmap — grid structure", [
            test_heightmap_dimensions,
            test_heightmap_slopes_populated_after_compute,
        ]),
        ("Integration — A* + heightmap", [
            test_astar_prefers_flat_path,
            test_astar_replan_on_updated_map,
        ]),
    ]

    results: list[tuple[str, bool]] = []

    for group_name, tests in test_groups:
        print(f"\n=== {group_name} ===\n")
        for t in tests:
            try:
                passed = t()
            except Exception as e:
                print(f"  ERROR — {e}")
                passed = False
            results.append((t.__name__, passed))

    print("\n" + "=" * 60)
    print("NAVIGATION TEST RESULTS")
    print("=" * 60)
    passed_n = sum(1 for _, p in results if p)
    failed_n = sum(1 for _, p in results if not p)
    for name, passed in results:
        print(f"  {'PASS' if passed else 'FAIL'}  {name}")
    print(f"\n  {passed_n} passed  |  {failed_n} failed  |  {len(results)} total")
