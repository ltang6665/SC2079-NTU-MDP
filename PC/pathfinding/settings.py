from enum import Enum

class Direction(int, Enum):
    NORTH = 0
    NORTHEAST = 1
    EAST = 2
    SOUTHEAST = 3
    SOUTH = 4
    SOUTHWEST = 5
    WEST = 6
    NORTHWEST = 7
    SKIP = 8

    def __int__(self):
        return self.value

    @staticmethod
    def rotation_cost(d1, d2):
        diff = abs(d1 - d2)
        return min(diff, 8 - diff)

MOVE_DIRECTIONS = [
    (0, 1, Direction.NORTH),
    (1, 1, Direction.NORTHEAST),
    (1, 0, Direction.EAST),
    (1, -1, Direction.SOUTHEAST),
    (0, -1, Direction.SOUTH),
    (-1, -1, Direction.SOUTHWEST),
    (-1, 0, Direction.WEST),
    (-1, 1, Direction.NORTHWEST),
]

TURN_FACTOR = 13

EXPANDED_CELL = 1


GRID_WIDTH = 20
GRID_HEIGHT = 20


ALLOW_EDGE = True
BORDER_MARGIN = 0 if ALLOW_EDGE else 1

MAX_ITERATIONS = 4000
TURN_RADIUS = 2
TURN_BIG   = 3
TURN_SMALL = 2

SAFE_TURN_COST = 700
SNAPSHOT_COST = 50
NUM_VIEWPOINTS = 4
CAMERA_DIRECTION = 'FRONT'

CLEARANCE = 2
ALLOW_45 = False
