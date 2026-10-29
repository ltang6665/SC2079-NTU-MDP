from pathfinding.settings import GRID_WIDTH, GRID_HEIGHT, BORDER_MARGIN, Direction, MOVE_DIRECTIONS, CAMERA_DIRECTION, ALLOW_45


def withinBounds(center_x: int, center_y: int):

    return (BORDER_MARGIN <= center_x < GRID_WIDTH - BORDER_MARGIN
            and BORDER_MARGIN <= center_y < GRID_HEIGHT - BORDER_MARGIN)


def generateCommands(states, obstacles):


    obstacles_dict = {obstacle['id']: obstacle for obstacle in obstacles}
    commands = []

    def add_snap(curr):


        if curr.screenshot_id == -1:
            return

        obstacle = obstacles_dict[curr.screenshot_id]
        rob = curr

        ob_dir = Direction(obstacle['d'])
        rob_dir = Direction(rob.direction)


        if CAMERA_DIRECTION == 'FRONT':

            if ob_dir == Direction.WEST and rob_dir == Direction.EAST:
                if obstacle['x'] > rob.x:
                    commands.append(f"SNAP{curr.screenshot_id}_R")
                elif obstacle['x'] == rob.x:
                    commands.append(f"SNAP{curr.screenshot_id}_C")
                else:
                    commands.append(f"SNAP{curr.screenshot_id}_L")

            elif ob_dir == Direction.EAST and rob_dir == Direction.WEST:
                if obstacle['x'] > rob.x:
                    commands.append(f"SNAP{curr.screenshot_id}_L")
                elif obstacle['x'] == rob.x:
                    commands.append(f"SNAP{curr.screenshot_id}_C")
                else:
                    commands.append(f"SNAP{curr.screenshot_id}_R")

            elif ob_dir == Direction.NORTH and rob_dir == Direction.SOUTH:
                if obstacle['y'] > rob.y:
                    commands.append(f"SNAP{curr.screenshot_id}_L")
                elif obstacle['y'] == rob.y:
                    commands.append(f"SNAP{curr.screenshot_id}_C")
                else:
                    commands.append(f"SNAP{curr.screenshot_id}_R")

            elif ob_dir == Direction.SOUTH and rob_dir == Direction.NORTH:
                if obstacle['y'] > rob.y:
                    commands.append(f"SNAP{curr.screenshot_id}_R")
                elif obstacle['y'] == rob.y:
                    commands.append(f"SNAP{curr.screenshot_id}_C")
                else:
                    commands.append(f"SNAP{curr.screenshot_id}_L")


        elif CAMERA_DIRECTION == 'LEFT':

            if ob_dir == Direction.NORTH and rob_dir == Direction.WEST:
                if obstacle['y'] > rob.y:
                    commands.append(f"SNAP{curr.screenshot_id}_R")
                elif obstacle['y'] == rob.y:
                    commands.append(f"SNAP{curr.screenshot_id}_C")
                else:
                    commands.append(f"SNAP{curr.screenshot_id}_L")

            elif ob_dir == Direction.EAST and rob_dir == Direction.NORTH:
                if obstacle['x'] > rob.x:
                    commands.append(f"SNAP{curr.screenshot_id}_L")
                elif obstacle['x'] == rob.x:
                    commands.append(f"SNAP{curr.screenshot_id}_C")
                else:
                    commands.append(f"SNAP{curr.screenshot_id}_R")

            elif ob_dir == Direction.SOUTH and rob_dir == Direction.EAST:
                if obstacle['y'] > rob.y:
                    commands.append(f"SNAP{curr.screenshot_id}_L")
                elif obstacle['y'] == rob.y:
                    commands.append(f"SNAP{curr.screenshot_id}_C")
                else:
                    commands.append(f"SNAP{curr.screenshot_id}_R")

            elif ob_dir == Direction.WEST and rob_dir == Direction.SOUTH:
                if obstacle['x'] > rob.x:
                    commands.append(f"SNAP{curr.screenshot_id}_R")
                elif obstacle['x'] == rob.x:
                    commands.append(f"SNAP{curr.screenshot_id}_C")
                else:
                    commands.append(f"SNAP{curr.screenshot_id}_L")


        elif CAMERA_DIRECTION == 'RIGHT':


            north_headings = {Direction.EAST}

            if ALLOW_45:
                north_headings.update({
                    Direction.NORTHEAST,
                    Direction.SOUTHEAST
                })

            if ob_dir == Direction.NORTH and rob_dir in north_headings:
                if obstacle['y'] > rob.y:
                    commands.append(f"SNAP{curr.screenshot_id}_R")
                elif obstacle['y'] == rob.y:
                    commands.append(f"SNAP{curr.screenshot_id}_C")
                else:
                    commands.append(f"SNAP{curr.screenshot_id}_L")


            east_headings = {Direction.SOUTH}

            if ALLOW_45:
                east_headings.update({
                    Direction.SOUTHEAST,
                    Direction.SOUTHWEST
                })

            if ob_dir == Direction.EAST and rob_dir in east_headings:
                if obstacle['x'] > rob.x:
                    commands.append(f"SNAP{curr.screenshot_id}_L")
                elif obstacle['x'] == rob.x:
                    commands.append(f"SNAP{curr.screenshot_id}_C")
                else:
                    commands.append(f"SNAP{curr.screenshot_id}_R")


            south_headings = {Direction.WEST}

            if ALLOW_45:
                south_headings.update({
                    Direction.SOUTHWEST,
                    Direction.NORTHWEST
                })

            if ob_dir == Direction.SOUTH and rob_dir in south_headings:
                if obstacle['y'] > rob.y:
                    commands.append(f"SNAP{curr.screenshot_id}_L")
                elif obstacle['y'] == rob.y:
                    commands.append(f"SNAP{curr.screenshot_id}_C")
                else:
                    commands.append(f"SNAP{curr.screenshot_id}_R")


            west_headings = {Direction.NORTH}

            if ALLOW_45:
                west_headings.update({
                    Direction.NORTHWEST,
                    Direction.NORTHEAST
                })

            if ob_dir == Direction.WEST and rob_dir in west_headings:
                if obstacle['x'] > rob.x:
                    commands.append(f"SNAP{curr.screenshot_id}_R")
                elif obstacle['x'] == rob.x:
                    commands.append(f"SNAP{curr.screenshot_id}_C")
                else:
                    commands.append(f"SNAP{curr.screenshot_id}_L")


    for i in range(1, len(states)):
        previous_state = states[i - 1]
        curr = states[i]

        dx = curr.x - previous_state.x
        dy = curr.y - previous_state.y

        old_dir = int(previous_state.direction)
        new_dir = int(curr.direction)

        diff = (new_dir - old_dir) % 8


        if curr.direction == previous_state.direction:

            expected_dx, expected_dy, _ = MOVE_DIRECTIONS[int(curr.direction)]

            if (dx, dy) == (expected_dx, expected_dy):
                commands.append("FW10")

            elif (dx, dy) == (-expected_dx, -expected_dy):
                commands.append("BW10")

            else:
                raise Exception(
                    f"Unexpected straight movement: "
                    f"dir={curr.direction}, "
                    f"expected ({expected_dx},{expected_dy}) "
                    f"or opposite, got ({dx},{dy})"
                )


        elif diff == 1:

            expected_dx, expected_dy, _ = MOVE_DIRECTIONS[int(curr.direction)]

            new_dx = expected_dx * dx
            new_dy = expected_dy * dy

            if new_dx > 0 or new_dy > 0:
                commands.append("FR45")
            else:
                commands.append("BL45")


        elif diff == 7:

            expected_dx, expected_dy, _ = MOVE_DIRECTIONS[int(curr.direction)]

            new_dx = expected_dx * dx
            new_dy = expected_dy * dy

            if new_dx > 0 or new_dy > 0:
                commands.append("FL45")
            else:
                commands.append("BR45")


        elif diff == 2:

            expected_dx, expected_dy, _ = MOVE_DIRECTIONS[int(curr.direction)]

            new_dx = expected_dx * dx
            new_dy = expected_dy * dy

            if new_dx > 0 or new_dy > 0:
                commands.append("FR90")
            else:
                commands.append("BL90")


        elif diff == 6:

            expected_dx, expected_dy, _ = MOVE_DIRECTIONS[int(curr.direction)]

            new_dx = expected_dx * dx
            new_dy = expected_dy * dy

            if new_dx > 0 or new_dy > 0:
                commands.append("FL90")
            else:
                commands.append("BR90")


        elif diff == 4:
            commands.extend(["FR90", "FR90"])

        else:
            raise Exception(
                f"Unsupported direction change: "
                f"{previous_state.direction} -> {curr.direction}"
            )


        add_snap(curr)

    commands.append("FIN")


    compressed = [commands[0]]

    for i in range(1, len(commands)):

        if commands[i].startswith("FW") and compressed[-1].startswith("FW"):

            steps = int(compressed[-1][2:])

            if steps != 180:
                compressed[-1] = f"FW{steps + 10}"
                continue

        elif commands[i].startswith("BW") and compressed[-1].startswith("BW"):

            steps = int(compressed[-1][2:])

            if steps != 180:
                compressed[-1] = f"BW{steps + 10}"
                continue

        compressed.append(commands[i])

    time = generateCommandTimes(compressed)

    return compressed, time


def generateCommandTimes(compressed_commands: list):

    time = []

    for command in compressed_commands:

        if command.startswith("FW") or command.startswith("BW"):
            steps = int(command[2:])

            time.append(steps / 10 * 3)


        elif command.startswith("FR45") or command.startswith("FL45") \
             or command.startswith("BR45") or command.startswith("BL45"):
            time.append(4)


        elif command.startswith("FR90") or command.startswith("FL90") \
             or command.startswith("BR90") or command.startswith("BL90"):
            time.append(8)


        elif command.startswith("SNAP"):
            time.append(0)


        elif command.startswith("FIN"):
            time.append(0)

        else:

            raise Exception(f"Unknown command in time_generator: {command}")

    return time
