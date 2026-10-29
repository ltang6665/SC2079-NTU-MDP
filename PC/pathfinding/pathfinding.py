from pathfinding.route_solver import MazeSolver
import time
from pathfinding.motion_commands import generateCommands
from pathfinding.settings import CAMERA_DIRECTION, ALLOW_45

def pathfinding(obstacles, robot_x = 1, robot_y = 1, robot_direction = 0, big_turn = None, retrying = False, mode = 0):


    maze_solver = MazeSolver(20, 20, robot_x, robot_y, robot_direction, big_turn=big_turn, allow_45=ALLOW_45)


    for obstacle in obstacles:
        maze_solver.addObstacle(obstacle['x'], obstacle['y'], obstacle['d'], obstacle['id'])

    start = time.time()


    optimal_path, distance = maze_solver.getOptimalOrderExact(retrying=retrying)

    visited_obstacles = {
        state.screenshot_id
        for state in optimal_path
        if state.screenshot_id != -1
    }
    print(CAMERA_DIRECTION)
    print(f"Obstacles visited: {len(visited_obstacles)} / {len(obstacles)}")
    print(f"Visited obstacle IDs: {sorted(visited_obstacles)}")

    commands,time_list = generateCommands(optimal_path, obstacles)


    path_results = [optimal_path[0].getDict()]

    i = 0
    for command in commands:
        if command.startswith("SNAP"):
            continue
        if command.startswith("FIN"):
            continue
        elif command.startswith("FW") or command.startswith("FS"):
            i += int(command[2:]) // 10
        elif command.startswith("BW") or command.startswith("BS"):
            i += int(command[2:]) // 10
        else:
            i += 1


        path_results.append(optimal_path[i].getDict())
    print(f"Time taken to find shortest path using A* search: {time.time() - start}s")
    print(f"Distance to travel: {distance} units")
    return {
            'distance': distance,
            'path': path_results,
            'commands': commands,
            'time': time_list
        }
