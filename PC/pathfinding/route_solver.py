import heapq
import math
from typing import List, Optional, Tuple
import numpy as np
from pathfinding.entities.robot_model import Robot
from pathfinding.entities.arena import Obstacle, CellState, Grid
from pathfinding.settings import Direction, MOVE_DIRECTIONS, TURN_FACTOR, MAX_ITERATIONS, TURN_RADIUS, SAFE_TURN_COST, TURN_BIG, TURN_SMALL
from pathfinding.motion_commands import generateCommands

turn_wrt_big_turns = [
    [TURN_BIG,     TURN_SMALL],
    [TURN_BIG + 1, TURN_SMALL]
]


class MazeSolver:
    def __init__(
            self,
            size_x: int,
            size_y: int,
            robot_x: int,
            robot_y: int,
            robot_direction: Direction,
            big_turn=None,
            allow_45 = True
    ):

        self.grid = Grid(size_x, size_y)

        self.robot = Robot(robot_x, robot_y, robot_direction)

        self.path_table = dict()
        self.cost_table = dict()
        if big_turn is None:
            self.big_turn = 0
        else:
            self.big_turn = int(big_turn)
        self.allow_45 = allow_45
        self._safe_cost_cache = None

    def addObstacle(self, x: int, y: int, direction: Direction, obstacle_id: int):

        obstacle = Obstacle(x, y, direction, obstacle_id)
        self.grid.addObstacle(obstacle)
        self._safe_cost_cache = None

    def resetObstacles(self):
        self.grid.reset_obstacles()
        self._safe_cost_cache = None

    @staticmethod
    def computeCoordDistance(x1: int, y1: int, x2: int, y2: int, level=1):

        horizontal_distance = x1 - x2
        vertical_distance = y1 - y2

        if level == 2:
            return math.sqrt(horizontal_distance ** 2 + vertical_distance ** 2)

        return abs(horizontal_distance) + abs(vertical_distance)

    @staticmethod
    def computeStateDistance(start_state: CellState, end_state: CellState, level=1):

        return MazeSolver.computeCoordDistance(start_state.x, start_state.y, end_state.x, end_state.y, level)

    @staticmethod
    def getVisitOptions(n):

        s = []
        l = bin(2 ** n - 1).count('1')

        for i in range(2 ** n):
            s.append(bin(i)[2:].zfill(l))

        s.sort(key=lambda x: x.count('1'), reverse=True)
        return s

    def getOptimalOrderDp(self, retrying) -> List[CellState]:


        from python_tsp.exact import solve_tsp_dynamic_programming

        distance = 1e9
        optimal_path = []
        all_view_positions = self.grid.get_view_obstacle_positions(retrying)

        for op in self.getVisitOptions(len(all_view_positions)):


            items = [self.robot.get_start_state()]

            cur_view_positions = []


            for idx in range(len(all_view_positions)):

                if op[idx] == '1':

                    items = items + all_view_positions[idx]

                    cur_view_positions.append(all_view_positions[idx])


            self.pathCostGenerator(items)
            combination = []
            self.generateCombination(cur_view_positions, 0, [], combination, [MAX_ITERATIONS])

            for c in combination:
                visited_candidates = [0]

                cur_index = 1
                fixed_cost = 0
                for index, view_position in enumerate(cur_view_positions):
                    visited_candidates.append(cur_index + c[index])
                    fixed_cost += view_position[c[index]].penalty
                    cur_index += len(view_position)

                cost_np = np.zeros((len(visited_candidates), len(visited_candidates)))

                for s in range(len(visited_candidates) - 1):
                    for e in range(s + 1, len(visited_candidates)):
                        u = items[visited_candidates[s]]
                        v = items[visited_candidates[e]]
                        if (u, v) in self.cost_table.keys():
                            cost_np[s][e] = self.cost_table[(u, v)]
                        else:
                            cost_np[s][e] = 1e9
                        cost_np[e][s] = cost_np[s][e]
                cost_np[:, 0] = 0
                _permutation, _distance = solve_tsp_dynamic_programming(cost_np)


                if _distance + fixed_cost >= distance:
                    continue

                optimal_path = [items[0]]
                distance = _distance + fixed_cost

                for i in range(len(_permutation) - 1):
                    from_item = items[visited_candidates[_permutation[i]]]
                    to_item = items[visited_candidates[_permutation[i + 1]]]

                    cur_path = self.path_table[(from_item, to_item)]
                    for j in range(1, len(cur_path)):
                        optimal_path.append(CellState(cur_path[j][0], cur_path[j][1], cur_path[j][2]))

                    optimal_path[-1].set_screenshot(to_item.screenshot_id)

            if optimal_path:

                break

        return optimal_path, distance

    def getOptimalOrderExact(self, retrying=False):

        all_view_positions = self.grid.get_view_obstacle_positions(retrying)
        start_state = self.robot.get_start_state()
        n = len(all_view_positions)

        if n == 0:
            return [start_state], 0.0


        edge_cache = {}

        def state_key(state: CellState) -> Tuple[int, int, int]:
            return (state.x, state.y, int(state.direction))

        def directed_astar(start: CellState, end: CellState) -> Optional[Tuple[float, list]]:
            key = (state_key(start), state_key(end))
            if key in edge_cache:
                return edge_cache[key]

            if start.is_eq(end.x, end.y, end.direction):
                result = (0.0, [state_key(start)])
                edge_cache[key] = result
                return result

            start_tuple = state_key(start)
            g_distance = {start_tuple: 0.0}

            heap = [(
                self.computeStateDistance(start, end, level=2),
                start.x,
                start.y,
                int(start.direction),
            )]
            parent = {}
            visited = set()

            while heap:
                _, current_x, current_y, cur_dir_int = heapq.heappop(heap)
                current_heading = Direction(cur_dir_int)
                cur_tuple = (current_x, current_y, cur_dir_int)

                if cur_tuple in visited:
                    continue

                if end.is_eq(current_x, current_y, current_heading):
                    path = []
                    cursor = cur_tuple
                    while cursor in parent:
                        path.append(cursor)
                        cursor = parent[cursor]
                    path.append(cursor)
                    path.reverse()
                    result = (g_distance[cur_tuple], path)
                    edge_cache[key] = result
                    return result

                visited.add(cur_tuple)
                cur_distance = g_distance[cur_tuple]

                for neighbor_x, neighbor_y, neighbor_heading, safe_cost in self.getNeighbors(
                    current_x, current_y, current_heading
                ):
                    next_tuple = (neighbor_x, neighbor_y, int(neighbor_heading))
                    if next_tuple in visited:
                        continue

                    step_cost = math.hypot(neighbor_x - current_x, neighbor_y - current_y)
                    move_cost = (
                        Direction.rotation_cost(neighbor_heading, current_heading) * TURN_FACTOR
                        + step_cost
                        + safe_cost
                    )
                    tentative_g = cur_distance + move_cost

                    if next_tuple not in g_distance or tentative_g < g_distance[next_tuple]:
                        g_distance[next_tuple] = tentative_g
                        parent[next_tuple] = cur_tuple
                        f_cost = tentative_g + self.computeCoordDistance(
                            neighbor_x, neighbor_y, end.x, end.y, level=2
                        )
                        heapq.heappush(
                            heap,
                            (f_cost, neighbor_x, neighbor_y, int(neighbor_heading)),
                        )

            edge_cache[key] = None
            return None


        dp = {}
        parent = {}

        for obs_idx in range(n):
            for view_idx, viewpoint in enumerate(all_view_positions[obs_idx]):
                edge = directed_astar(start_state, viewpoint)
                if edge is None:
                    continue
                edge_cost, _ = edge
                total = edge_cost + viewpoint.penalty
                state = (1 << obs_idx, obs_idx, view_idx)
                if state not in dp or total < dp[state]:
                    dp[state] = total
                    parent[state] = None


        for mask in range(1, 1 << n):
            for last_obs in range(n):
                if not (mask & (1 << last_obs)):
                    continue
                views_last = all_view_positions[last_obs]
                for last_view in range(len(views_last)):
                    state = (mask, last_obs, last_view)
                    if state not in dp:
                        continue
                    cur_cost = dp[state]
                    cur_vp = views_last[last_view]

                    for next_obs in range(n):
                        if mask & (1 << next_obs):
                            continue
                        for next_view, next_vp in enumerate(all_view_positions[next_obs]):
                            edge = directed_astar(cur_vp, next_vp)
                            if edge is None:
                                continue
                            edge_cost, _ = edge
                            new_cost = cur_cost + edge_cost + next_vp.penalty
                            new_state = (mask | (1 << next_obs), next_obs, next_view)
                            if new_state not in dp or new_cost < dp[new_state]:
                                dp[new_state] = new_cost
                                parent[new_state] = state

        if not dp:

            return [start_state], 0.0


        max_visited = max(bin(mask).count("1") for mask, _, _ in dp.keys())
        best_state = None
        best_cost = float("inf")
        for state, cost in dp.items():
            mask, _, _ = state
            if bin(mask).count("1") != max_visited:
                continue
            if cost < best_cost:
                best_cost = cost
                best_state = state


        sequence = []
        state = best_state
        while state is not None:
            _, obs_idx, view_idx = state
            sequence.append((obs_idx, view_idx))
            state = parent[state]
        sequence.reverse()


        optimal_path = [CellState(start_state.x, start_state.y, start_state.direction)]
        current = start_state
        for obs_idx, view_idx in sequence:
            viewpoint = all_view_positions[obs_idx][view_idx]
            edge = directed_astar(current, viewpoint)
            if edge is None:

                return [start_state], 0.0
            _, path_tuples = edge
            for j in range(1, len(path_tuples)):
                x, y, d = path_tuples[j]
                optimal_path.append(CellState(x, y, Direction(d)))
            optimal_path[-1].set_screenshot(viewpoint.screenshot_id)
            current = viewpoint

        return optimal_path, best_cost

    @staticmethod
    def generateCombination(view_positions, index, current, result, iteration_left):
        if index == len(view_positions):
            result.append(current[:])
            return

        if iteration_left[0] == 0:
            return

        iteration_left[0] -= 1
        for j in range(len(view_positions[index])):
            current.append(j)
            MazeSolver.generateCombination(view_positions, index + 1, current, result, iteration_left)
            current.pop()

    def _buildSafeCostCache(self):
        cache = {}
        for x in range(self.grid.size_x):
            for y in range(self.grid.size_y):
                cost = 0
                for ob in self.grid.obstacles:
                    if ((abs(ob.x - x) == 2 and abs(ob.y - y) == 2) or
                            (abs(ob.x - x) == 1 and abs(ob.y - y) == 2) or
                            (abs(ob.x - x) == 2 and abs(ob.y - y) == 1) or
                            (abs(ob.x - x) <= 1 and abs(ob.y - y) <= 1)):
                        cost = SAFE_TURN_COST
                        break
                cache[(x, y)] = cost
        self._safe_cost_cache = cache

    def getSafeCost(self, x, y):

        if self._safe_cost_cache is None:
            self._buildSafeCostCache()
        return self._safe_cost_cache.get((x, y), 0)


    def _hasForwardClearance(self, x, y, direction, steps=2):

        fwd = {Direction.NORTH: (0, 1), Direction.EAST: (1, 0),
               Direction.SOUTH: (0, -1), Direction.WEST: (-1, 0)}
        if direction not in fwd:
            return True
        fdx, fdy = fwd[direction]
        return all(self.grid.reachable(x + fdx * s, y + fdy * s) for s in range(1, steps + 1))

    def _hasBackwardClearance(self, x, y, direction, steps=2):

        bwd = {Direction.NORTH: (0, -1), Direction.EAST: (-1, 0),
               Direction.SOUTH: (0, 1), Direction.WEST: (1, 0)}
        if direction not in bwd:
            return True
        bdx, bdy = bwd[direction]
        return all(self.grid.reachable(x + bdx * s, y + bdy * s) for s in range(1, steps + 1))

    def getNeighbors(self, x, y, direction):
        neighbors = []

        for dx, dy, md in MOVE_DIRECTIONS:
            if md == direction:
                if self.grid.reachable(x + dx, y + dy):
                    safe_cost = self.getSafeCost(x + dx, y + dy)
                    neighbors.append((x + dx, y + dy, md, safe_cost))
                if self.grid.reachable(x - dx, y - dy):
                    safe_cost = self.getSafeCost(x - dx, y - dy)
                    neighbors.append((x - dx, y - dy, md, safe_cost))

        if self.allow_45:
            for dx, dy, md in MOVE_DIRECTIONS:
                diff = (int(md) - int(direction)) % 8
                if diff in [1, 7]:
                    if self.grid.reachable(x + dx, y + dy, turn=True) and self.grid.reachable(x, y, preTurn=True):
                        safe_cost = self.getSafeCost(x + dx, y + dy)
                        neighbors.append((x + dx, y + dy, md, safe_cost + 5))

        bigger_change = turn_wrt_big_turns[self.big_turn][0]
        smaller_change = turn_wrt_big_turns[self.big_turn][1]
        _fvec = {Direction.NORTH: (0, 1), Direction.EAST: (1, 0),
                 Direction.SOUTH: (0, -1), Direction.WEST: (-1, 0)}
        fwd_penalty = 0 if self._hasForwardClearance(x, y, direction) else SAFE_TURN_COST
        bwd_clear = self._hasBackwardClearance(x, y, direction)
        fvec = _fvec.get(direction, (0, 0))

        if direction == Direction.NORTH:
            if self.grid.reachable(x + bigger_change, y + smaller_change, turn=True) and self.grid.reachable(x, y, preTurn=True):
                safe_cost = self.getSafeCost(x + bigger_change, y + smaller_change)
                neighbors.append((x + bigger_change, y + smaller_change, Direction.EAST, safe_cost + 10 + fwd_penalty))
            if bwd_clear and self.grid.reachable(x - smaller_change, y - bigger_change, turn=True) and self.grid.reachable(x, y, preTurn=True):
                safe_cost = self.getSafeCost(x - smaller_change, y - bigger_change)
                neighbors.append((x - smaller_change, y - bigger_change, Direction.EAST, safe_cost + 10))
            if self.grid.reachable(x - bigger_change, y + smaller_change, turn=True) and self.grid.reachable(x, y, preTurn=True):
                safe_cost = self.getSafeCost(x - bigger_change, y + smaller_change)
                neighbors.append((x - bigger_change, y + smaller_change, Direction.WEST, safe_cost + 10 + fwd_penalty))
            if bwd_clear and self.grid.reachable(x + smaller_change, y - bigger_change, turn=True) and self.grid.reachable(x, y, preTurn=True):
                safe_cost = self.getSafeCost(x + smaller_change, y - bigger_change)
                neighbors.append((x + smaller_change, y - bigger_change, Direction.WEST, safe_cost + 10))

        elif direction == Direction.EAST:
            if self.grid.reachable(x + smaller_change, y + bigger_change, turn=True) and self.grid.reachable(x, y, preTurn=True):
                safe_cost = self.getSafeCost(x + smaller_change, y + bigger_change)
                neighbors.append((x + smaller_change, y + bigger_change, Direction.NORTH, safe_cost + 10 + fwd_penalty))
            if bwd_clear and self.grid.reachable(x - bigger_change, y - smaller_change, turn=True) and self.grid.reachable(x, y, preTurn=True):
                safe_cost = self.getSafeCost(x - bigger_change, y - smaller_change)
                neighbors.append((x - bigger_change, y - smaller_change, Direction.NORTH, safe_cost + 10))
            if self.grid.reachable(x + smaller_change, y - bigger_change, turn=True) and self.grid.reachable(x, y, preTurn=True):
                safe_cost = self.getSafeCost(x + smaller_change, y - bigger_change)
                neighbors.append((x + smaller_change, y - bigger_change, Direction.SOUTH, safe_cost + 10 + fwd_penalty))
            if bwd_clear and self.grid.reachable(x - bigger_change, y + smaller_change, turn=True) and self.grid.reachable(x, y, preTurn=True):
                safe_cost = self.getSafeCost(x - bigger_change, y + smaller_change)
                neighbors.append((x - bigger_change, y + smaller_change, Direction.SOUTH, safe_cost + 10))

        elif direction == Direction.SOUTH:
            if self.grid.reachable(x + bigger_change, y - smaller_change, turn=True) and self.grid.reachable(x, y, preTurn=True):
                safe_cost = self.getSafeCost(x + bigger_change, y - smaller_change)
                neighbors.append((x + bigger_change, y - smaller_change, Direction.EAST, safe_cost + 10 + fwd_penalty))
            if bwd_clear and self.grid.reachable(x - smaller_change, y + bigger_change, turn=True) and self.grid.reachable(x, y, preTurn=True):
                safe_cost = self.getSafeCost(x - smaller_change, y + bigger_change)
                neighbors.append((x - smaller_change, y + bigger_change, Direction.EAST, safe_cost + 10))
            if self.grid.reachable(x - bigger_change, y - smaller_change, turn=True) and self.grid.reachable(x, y, preTurn=True):
                safe_cost = self.getSafeCost(x - bigger_change, y - smaller_change)
                neighbors.append((x - bigger_change, y - smaller_change, Direction.WEST, safe_cost + 10 + fwd_penalty))
            if bwd_clear and self.grid.reachable(x + smaller_change, y + bigger_change, turn=True) and self.grid.reachable(x, y, preTurn=True):
                safe_cost = self.getSafeCost(x + smaller_change, y + bigger_change)
                neighbors.append((x + smaller_change, y + bigger_change, Direction.WEST, safe_cost + 10))

        elif direction == Direction.WEST:
            if self.grid.reachable(x - smaller_change, y - bigger_change, turn=True) and self.grid.reachable(x, y, preTurn=True):
                safe_cost = self.getSafeCost(x - smaller_change, y - bigger_change)
                neighbors.append((x - smaller_change, y - bigger_change, Direction.SOUTH, safe_cost + 10 + fwd_penalty))
            if bwd_clear and self.grid.reachable(x + bigger_change, y + smaller_change, turn=True) and self.grid.reachable(x, y, preTurn=True):
                safe_cost = self.getSafeCost(x + bigger_change, y + smaller_change)
                neighbors.append((x + bigger_change, y + smaller_change, Direction.SOUTH, safe_cost + 10))
            if self.grid.reachable(x - smaller_change, y + bigger_change, turn=True) and self.grid.reachable(x, y, preTurn=True):
                safe_cost = self.getSafeCost(x - smaller_change, y + bigger_change)
                neighbors.append((x - smaller_change, y + bigger_change, Direction.NORTH, safe_cost + 10 + fwd_penalty))
            if bwd_clear and self.grid.reachable(x + bigger_change, y - smaller_change, turn=True) and self.grid.reachable(x, y, preTurn=True):
                safe_cost = self.getSafeCost(x + bigger_change, y - smaller_change)
                neighbors.append((x + bigger_change, y - smaller_change, Direction.NORTH, safe_cost + 10))

        return neighbors

    def pathCostGenerator(self, states: List[CellState]):

        def record_path(start, end, parent: dict, cost: int):

            self.cost_table[(start, end)] = cost
            self.cost_table[(end, start)] = cost

            path = []
            cursor = (end.x, end.y, end.direction)

            while cursor in parent:
                path.append(cursor)
                cursor = parent[cursor]

            path.append(cursor)

            self.path_table[(start, end)] = path[::-1]
            self.path_table[(end, start)] = path

        def astar_search(start: CellState, end: CellState):
            if (start, end) in self.path_table:
                return

            g_distance = {(start.x, start.y, start.direction): 0}
            heap = [(self.computeStateDistance(start, end), start.x, start.y, start.direction)]
            parent = dict()
            visited = set()

            while heap:
                _, current_x, current_y, current_heading = heapq.heappop(heap)

                if (current_x, current_y, current_heading) in visited:
                    continue

                if end.is_eq(current_x, current_y, current_heading):
                    record_path(start, end, parent, g_distance[(current_x, current_y, current_heading)])
                    return

                visited.add((current_x, current_y, current_heading))
                cur_distance = g_distance[(current_x, current_y, current_heading)]

                for neighbor_x, neighbor_y, neighbor_heading, safe_cost in self.getNeighbors(current_x, current_y, current_heading):
                    if (neighbor_x, neighbor_y, neighbor_heading) in visited:
                        continue

                    step_cost = math.sqrt((neighbor_x - current_x) ** 2 + (neighbor_y - current_y) ** 2)
                    move_cost = Direction.rotation_cost(neighbor_heading, current_heading) * TURN_FACTOR + step_cost + safe_cost

                    tentative_g = cur_distance + move_cost
                    next_cost = tentative_g + self.computeCoordDistance(neighbor_x, neighbor_y, end.x, end.y, level=2)

                    if (neighbor_x, neighbor_y, neighbor_heading) not in g_distance or \
                            g_distance[(neighbor_x, neighbor_y, neighbor_heading)] > tentative_g:
                        g_distance[(neighbor_x, neighbor_y, neighbor_heading)] = tentative_g
                        parent[(neighbor_x, neighbor_y, neighbor_heading)] = (current_x, current_y, current_heading)

                        heapq.heappush(heap, (next_cost, neighbor_x, neighbor_y, neighbor_heading))

        for i in range(len(states) - 1):
            for j in range(i + 1, len(states)):
                astar_search(states[i], states[j])

if __name__ == "__main__":
    solver = MazeSolver(
        20, 20,
        1, 1,
        Direction.NORTH,
        big_turn=None,
        allow_45=False
    )


    obstacles = [
            (7, 2, Direction.NORTH, 1),
            (4, 14, Direction.SOUTH, 2),
            (8, 9, Direction.EAST, 3),
            (3, 7, Direction.EAST, 4),
            (12, 16, Direction.SOUTH, 5),
            (13, 5, Direction.NORTH, 6),
            (16, 11, Direction.WEST, 7),
        ]
    obstacles.sort(key=lambda obstacle: obstacle[3])


    for x, y, direction, obstacle_id in obstacles:
        solver.addObstacle(x, y, direction, obstacle_id)

    print("Number of obstacles:", len(solver.grid.obstacles))


    best_path, total_distance = solver.getOptimalOrderDp(retrying=False)

    print("\nTotal distance:", total_distance)


    obstacle_dicts = [
        {
            "x": x,
            "y": y,
            "d": direction,
            "id": obstacle_id
        }
        for x, y, direction, obstacle_id in obstacles
    ]


    commands, command_times = generateCommands(
        best_path,
        obstacle_dicts
    )

    print("\nBEST PATH:")
    for state in best_path:
        print(state.x, state.y, state.direction, state.screenshot_id)

    print("\nCOMMANDS:")
    print(commands)

    print("\nDISTANCE:")
    print(total_distance)
