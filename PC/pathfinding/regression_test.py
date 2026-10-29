from pathfinding.route_solver import MazeSolver
from pathfinding.settings import Direction
from pathfinding.motion_commands import generateCommands


def _assert_legal_transitions(solver: MazeSolver, path, label: str):

    for i in range(1, len(path)):
        cur = path[i - 1]
        nxt = path[i]
        neighbors = solver.getNeighbors(cur.x, cur.y, cur.direction)
        legal = {
            (nx, ny, int(nd))
            for nx, ny, nd, _safe in neighbors
        }
        key = (nxt.x, nxt.y, int(nxt.direction))
        if key not in legal:
            raise AssertionError(
                f"{label}: illegal transition at index {i}: "
                f"({cur.x}, {cur.y}, {int(cur.direction)}) -> "
                f"({nxt.x}, {nxt.y}, {int(nxt.direction)}); "
                f"legal={sorted(legal)}"
            )


def _assert_no_duplicate_screenshots(path, label: str):
    seen = []
    for state in path:
        sid = state.screenshot_id
        if sid == -1:
            continue
        if sid in seen:
            raise AssertionError(
                f"{label}: obstacle id {sid} snapshotted more than once"
            )
        seen.append(sid)
    return seen


def _run_case(name: str, obstacles, robot_x=1, robot_y=1, robot_dir=Direction.NORTH):
    solver = MazeSolver(20, 20, robot_x, robot_y, robot_dir, big_turn=None, allow_45=False)
    for obstacle in obstacles:
        solver.addObstacle(obstacle["x"], obstacle["y"], obstacle["d"], obstacle["id"])

    path, cost = solver.getOptimalOrderExact(retrying=False)
    assert path, f"{name}: empty path"
    assert path[0].x == robot_x and path[0].y == robot_y

    _assert_legal_transitions(solver, path, name)
    snapped = _assert_no_duplicate_screenshots(path, name)

    commands, _times = generateCommands(path, obstacles)
    assert commands, f"{name}: generateCommands returned empty list"
    assert any(c.startswith("FIN") for c in commands), f"{name}: missing FIN"

    print(
        f"PASS {name}: cost={cost:.3f}, visited={len(snapped)}, "
        f"commands={len(commands)}, path_len={len(path)}"
    )
    return path, cost, commands, snapped


def test_sample_multi_obstacle():
    obstacles = [
        {"x": 7, "y": 2, "d": Direction.NORTH, "id": 1},
        {"x": 4, "y": 14, "d": Direction.SOUTH, "id": 2},
        {"x": 8, "y": 9, "d": Direction.EAST, "id": 3},
        {"x": 3, "y": 7, "d": Direction.EAST, "id": 4},
        {"x": 12, "y": 16, "d": Direction.SOUTH, "id": 5},
        {"x": 13, "y": 5, "d": Direction.NORTH, "id": 6},
        {"x": 16, "y": 11, "d": Direction.WEST, "id": 7},
    ]
    return _run_case("sample", obstacles)


def test_reverse_path_regression():

    obstacles = [
        {"x": 7, "y": 6, "d": Direction.WEST, "id": 2},
        {"x": 6, "y": 5, "d": Direction.NORTH, "id": 3},
        {"x": 14, "y": 4, "d": Direction.NORTH, "id": 4},
        {"x": 3, "y": 14, "d": Direction.NORTH, "id": 1},
    ]
    return _run_case("reverse_regression", obstacles)


def main():
    test_sample_multi_obstacle()
    test_reverse_path_regression()
    print("All regression tests passed.")


if __name__ == "__main__":
    main()
