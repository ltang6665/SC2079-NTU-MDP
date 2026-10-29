from typing import List
from pathfinding.settings import Direction, BORDER_MARGIN, EXPANDED_CELL, SNAPSHOT_COST, NUM_VIEWPOINTS, CAMERA_DIRECTION, CLEARANCE, ALLOW_45
from pathfinding.motion_commands import withinBounds

class CellState:


    def __init__(self, x, y, direction: Direction = Direction.NORTH, screenshot_id=-1, penalty=0):
        self.x = x
        self.y = y
        self.pos_x = x
        self.pos_y = y
        self.direction = direction

        self.screenshot_id = screenshot_id
        self.penalty = penalty

    def cmp_position(self, x, y) -> bool:

        return self.x == x and self.y == y

    def is_eq(self, x, y, direction):

        return self.x == x and self.y == y and self.direction == direction

    def __repr__(self):
        return "x: {}, y: {}, d: {}, screenshot: {}".format(self.x, self.y, self.direction, self.screenshot_id)

    def set_screenshot(self, screenshot_id):

        self.screenshot_id = screenshot_id

    def getDict(self):

        return {'x': self.x, 'y': self.y, 'd': self.direction, 's': self.screenshot_id}


class Obstacle(CellState):


    def __init__(self, x: int, y: int, direction: Direction, obstacle_id: int):
        super().__init__(x, y, direction)
        self.obstacle_id = obstacle_id

    def __eq__(self, other):

        return self.x == other.x and self.y == other.y and self.direction == other.direction

    def _getViewStateFront4(self):
        view_states = []

        E = EXPANDED_CELL

        if self.direction == Direction.NORTH:

            view_states = [
                CellState(
                    self.x - 1,
                    self.y + 2 + E,
                    Direction.SOUTH,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),

                CellState(
                    self.x,
                    self.y + 1 + E,
                    Direction.SOUTH,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),

                CellState(
                    self.x + 1,
                    self.y + 2 + E,
                    Direction.SOUTH,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),

                CellState(
                    self.x,
                    self.y + 2 + E,
                    Direction.SOUTH,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),
            ]

        elif self.direction == Direction.SOUTH:

            view_states = [
                CellState(
                    self.x + 1,
                    self.y - 2 - E,
                    Direction.NORTH,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),

                CellState(
                    self.x,
                    self.y - 1 - E,
                    Direction.NORTH,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),

                CellState(
                    self.x - 1,
                    self.y - 2 - E,
                    Direction.NORTH,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),

                CellState(
                    self.x,
                    self.y - 2 - E,
                    Direction.NORTH,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),
            ]

        elif self.direction == Direction.EAST:

            view_states = [
                CellState(
                    self.x + 2 + E,
                    self.y + 1,
                    Direction.WEST,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),

                CellState(
                    self.x + 1 + E,
                    self.y,
                    Direction.WEST,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),

                CellState(
                    self.x + 2 + E,
                    self.y - 1,
                    Direction.WEST,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),

                CellState(
                    self.x + 2 + E,
                    self.y,
                    Direction.WEST,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),
            ]

        elif self.direction == Direction.WEST:

            view_states = [
                CellState(
                    self.x - 2 - E,
                    self.y - 1,
                    Direction.EAST,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),

                CellState(
                    self.x - 1 - E,
                    self.y,
                    Direction.EAST,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),

                CellState(
                    self.x - 2 - E,
                    self.y + 1,
                    Direction.EAST,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),

                CellState(
                    self.x - 2 - E,
                    self.y,
                    Direction.EAST,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),
            ]

        return [
            state for state in view_states
            if withinBounds(state.x, state.y)
        ]

    def _getViewStateLeft4(self):
            view_states = []

            E = EXPANDED_CELL

            if self.direction == Direction.NORTH:

                view_states = [
                    CellState(
                        self.x - 1,
                        self.y + 2 + E,
                        Direction.WEST,
                        screenshot_id=self.obstacle_id,
                        penalty=SNAPSHOT_COST
                    ),

                    CellState(
                        self.x,
                        self.y + 1 + E,
                        Direction.WEST,
                        screenshot_id=self.obstacle_id,
                        penalty=0
                    ),

                    CellState(
                        self.x + 1,
                        self.y + 2 + E,
                        Direction.WEST,
                        screenshot_id=self.obstacle_id,
                        penalty=SNAPSHOT_COST
                    ),

                    CellState(
                        self.x,
                        self.y + 2 + E,
                        Direction.WEST,
                        screenshot_id=self.obstacle_id,
                        penalty=0
                    ),
                ]

            elif self.direction == Direction.SOUTH:

                view_states = [
                    CellState(
                        self.x + 1,
                        self.y - 2 - E,
                        Direction.EAST,
                        screenshot_id=self.obstacle_id,
                        penalty=SNAPSHOT_COST
                    ),

                    CellState(
                        self.x,
                        self.y - 1 - E,
                        Direction.EAST,
                        screenshot_id=self.obstacle_id,
                        penalty=0
                    ),

                    CellState(
                        self.x - 1,
                        self.y - 2 - E,
                        Direction.EAST,
                        screenshot_id=self.obstacle_id,
                        penalty=SNAPSHOT_COST
                    ),

                    CellState(
                        self.x,
                        self.y - 2 - E,
                        Direction.EAST,
                        screenshot_id=self.obstacle_id,
                        penalty=0
                    ),
                ]

            elif self.direction == Direction.EAST:

                view_states = [
                    CellState(
                        self.x + 2 + E,
                        self.y + 1,
                        Direction.NORTH,
                        screenshot_id=self.obstacle_id,
                        penalty=SNAPSHOT_COST
                    ),

                    CellState(
                        self.x + 1 + E,
                        self.y,
                        Direction.NORTH,
                        screenshot_id=self.obstacle_id,
                        penalty=0
                    ),

                    CellState(
                        self.x + 2 + E,
                        self.y - 1,
                        Direction.NORTH,
                        screenshot_id=self.obstacle_id,
                        penalty=SNAPSHOT_COST
                    ),

                    CellState(
                        self.x + 2 + E,
                        self.y,
                        Direction.NORTH,
                        screenshot_id=self.obstacle_id,
                        penalty=0
                    ),
                ]

            elif self.direction == Direction.WEST:

                view_states = [
                    CellState(
                        self.x - 2 - E,
                        self.y - 1,
                        Direction.SOUTH,
                        screenshot_id=self.obstacle_id,
                        penalty=SNAPSHOT_COST
                    ),

                    CellState(
                        self.x - 1 - E,
                        self.y,
                        Direction.SOUTH,
                        screenshot_id=self.obstacle_id,
                        penalty=0
                    ),

                    CellState(
                        self.x - 2 - E,
                        self.y + 1,
                        Direction.SOUTH,
                        screenshot_id=self.obstacle_id,
                        penalty=SNAPSHOT_COST
                    ),

                    CellState(
                        self.x - 2 - E,
                        self.y,
                        Direction.SOUTH,
                        screenshot_id=self.obstacle_id,
                        penalty=0
                    ),
                ]

            return [
                state for state in view_states
                if withinBounds(state.x, state.y)
            ]

    def _getViewStateRight2(self):
        view_states = []

        E = EXPANDED_CELL

        if self.direction == Direction.NORTH:


            view_states = [

                CellState(
                    self.x,
                    self.y + 1 + E,
                    Direction.EAST,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),

                CellState(
                    self.x,
                    self.y + 2 + E,
                    Direction.EAST,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),
            ]
            print(view_states)

        elif self.direction == Direction.SOUTH:

            view_states = [
                CellState(
                    self.x,
                    self.y - 1 - E,
                    Direction.WEST,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),

                CellState(
                    self.x,
                    self.y - 2 - E,
                    Direction.WEST,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),
            ]

        elif self.direction == Direction.EAST:

            view_states = [

                CellState(
                    self.x + 1 + E,
                    self.y,
                    Direction.SOUTH,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),

                CellState(
                    self.x + 2 + E,
                    self.y,
                    Direction.SOUTH,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),
            ]

        elif self.direction == Direction.WEST:

            view_states = [

                CellState(
                    self.x - 1 - E,
                    self.y,
                    Direction.NORTH,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),

                CellState(
                    self.x - 2 - E,
                    self.y,
                    Direction.NORTH,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),
            ]

        return [
            state for state in view_states
            if withinBounds(state.x, state.y)
        ]

    def _getViewStateRight5(self):

        view_states = []

        E = EXPANDED_CELL

        if self.direction == Direction.NORTH:


            view_states = [
                CellState(
                    self.x,
                    self.y + 1 + E,
                    Direction.EAST,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),
                CellState(
                    self.x,
                    self.y + 2 + E,
                    Direction.EAST,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),
                CellState(
                    self.x - 1,
                    self.y + 3 + E,
                    Direction.EAST,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),
                CellState(
                    self.x,
                    self.y + 3 + E,
                    Direction.EAST,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),
                CellState(
                    self.x + 1,
                    self.y + 3 + E,
                    Direction.EAST,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),
            ]

        elif self.direction == Direction.SOUTH:

            view_states = [
                CellState(
                    self.x,
                    self.y - 1 - E,
                    Direction.WEST,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),
                CellState(
                    self.x,
                    self.y - 2 - E,
                    Direction.WEST,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),
                CellState(
                    self.x + 1,
                    self.y - 3 - E,
                    Direction.WEST,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),
                CellState(
                    self.x,
                    self.y - 3 - E,
                    Direction.WEST,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),
                CellState(
                    self.x - 1,
                    self.y - 3 - E,
                    Direction.WEST,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),
            ]

        elif self.direction == Direction.EAST:

            view_states = [
                CellState(
                    self.x + 1 + E,
                    self.y,
                    Direction.SOUTH,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),
                CellState(
                    self.x + 2 + E,
                    self.y,
                    Direction.SOUTH,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),
                CellState(
                    self.x + 3 + E,
                    self.y + 1,
                    Direction.SOUTH,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),
                CellState(
                    self.x + 3 + E,
                    self.y,
                    Direction.SOUTH,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),
                CellState(
                    self.x + 3 + E,
                    self.y - 1,
                    Direction.SOUTH,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),
            ]

        elif self.direction == Direction.WEST:

            view_states = [
                CellState(
                    self.x - 1 - E,
                    self.y,
                    Direction.NORTH,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),
                CellState(
                    self.x - 2 - E,
                    self.y,
                    Direction.NORTH,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),
                CellState(
                    self.x - 3 - E,
                    self.y - 1,
                    Direction.NORTH,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),
                CellState(
                    self.x - 3 - E,
                    self.y,
                    Direction.NORTH,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),
                CellState(
                    self.x - 3 - E,
                    self.y + 1,
                    Direction.NORTH,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),
            ]

        return [
            state for state in view_states
            if withinBounds(state.x, state.y)
        ]

    def _getViewStateRight2Allow45(self):
        view_states = []

        E = EXPANDED_CELL

        if self.direction == Direction.NORTH:


            view_states = [

                CellState(
                    self.x,
                    self.y + 1 + E,
                    Direction.EAST,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),

                CellState(
                    self.x-1,
                    self.y + 2 + E,
                    Direction.NORTHEAST,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),

                CellState(
                    self.x+1,
                    self.y + 2 + E,
                    Direction.SOUTHEAST,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),

                CellState(
                    self.x,
                    self.y + 2 + E,
                    Direction.EAST,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),
            ]
            print(view_states)

        elif self.direction == Direction.SOUTH:

            view_states = [
                CellState(
                    self.x,
                    self.y - 1 - E,
                    Direction.WEST,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),

                CellState(
                    self.x + 1,
                    self.y - 2 - E,
                    Direction.SOUTHWEST,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),

                CellState(
                    self.x - 1,
                    self.y - 2 - E,
                    Direction.NORTHWEST,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),

                CellState(
                    self.x,
                    self.y - 2 - E,
                    Direction.WEST,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),
            ]

        elif self.direction == Direction.EAST:

            view_states = [

                CellState(
                    self.x + 1 + E,
                    self.y,
                    Direction.SOUTH,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),

                CellState(
                    self.x + 2 + E,
                    self.y,
                    Direction.SOUTH,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),

                CellState(
                    self.x + 2 + E,
                    self.y + 1,
                    Direction.SOUTHEAST,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),

                CellState(
                    self.x + 2 + E,
                    self.y - 1,
                    Direction.SOUTHWEST,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),
            ]

        elif self.direction == Direction.WEST:

            view_states = [

                CellState(
                    self.x - 1 - E,
                    self.y,
                    Direction.NORTH,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),

                CellState(
                    self.x - 2 - E,
                    self.y,
                    Direction.NORTH,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),
                CellState(
                    self.x - 2 - E,
                    self.y + 1,
                    Direction.NORTHEAST,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),
                CellState(
                    self.x - 2 - E,
                    self.y - 1,
                    Direction.NORTHWEST,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),
            ]

        return [
            state for state in view_states
            if withinBounds(state.x, state.y)
        ]

    def _getViewStateRight4(self):
                view_states = []

                E = EXPANDED_CELL

                if self.direction == Direction.NORTH:

                    view_states = [
                        CellState(
                            self.x - 1,
                            self.y + 2 + E,
                            Direction.EAST,
                            screenshot_id=self.obstacle_id,
                            penalty=SNAPSHOT_COST
                        ),

                        CellState(
                            self.x,
                            self.y + 1 + E,
                            Direction.EAST,
                            screenshot_id=self.obstacle_id,
                            penalty=0
                        ),

                        CellState(
                            self.x + 1,
                            self.y + 2 + E,
                            Direction.EAST,
                            screenshot_id=self.obstacle_id,
                            penalty=SNAPSHOT_COST
                        ),

                        CellState(
                            self.x,
                            self.y + 2 + E,
                            Direction.EAST,
                            screenshot_id=self.obstacle_id,
                            penalty=0
                        ),
                    ]

                elif self.direction == Direction.SOUTH:

                    view_states = [
                        CellState(
                            self.x + 1,
                            self.y - 2 - E,
                            Direction.WEST,
                            screenshot_id=self.obstacle_id,
                            penalty=SNAPSHOT_COST
                        ),

                        CellState(
                            self.x,
                            self.y - 1 - E,
                            Direction.WEST,
                            screenshot_id=self.obstacle_id,
                            penalty=0
                        ),

                        CellState(
                            self.x - 1,
                            self.y - 2 - E,
                            Direction.WEST,
                            screenshot_id=self.obstacle_id,
                            penalty=SNAPSHOT_COST
                        ),

                        CellState(
                            self.x,
                            self.y - 2 - E,
                            Direction.WEST,
                            screenshot_id=self.obstacle_id,
                            penalty=0
                        ),
                    ]

                elif self.direction == Direction.EAST:

                    view_states = [
                        CellState(
                            self.x + 2 + E,
                            self.y + 1,
                            Direction.SOUTH,
                            screenshot_id=self.obstacle_id,
                            penalty=SNAPSHOT_COST
                        ),

                        CellState(
                            self.x + 1 + E,
                            self.y,
                            Direction.SOUTH,
                            screenshot_id=self.obstacle_id,
                            penalty=0
                        ),

                        CellState(
                            self.x + 2 + E,
                            self.y - 1,
                            Direction.SOUTH,
                            screenshot_id=self.obstacle_id,
                            penalty=SNAPSHOT_COST
                        ),

                        CellState(
                            self.x + 2 + E,
                            self.y,
                            Direction.SOUTH,
                            screenshot_id=self.obstacle_id,
                            penalty=0
                        ),
                    ]

                elif self.direction == Direction.WEST:

                    view_states = [
                        CellState(
                            self.x - 2 - E,
                            self.y - 1,
                            Direction.NORTH,
                            screenshot_id=self.obstacle_id,
                            penalty=SNAPSHOT_COST
                        ),

                        CellState(
                            self.x - 1 - E,
                            self.y,
                            Direction.NORTH,
                            screenshot_id=self.obstacle_id,
                            penalty=0
                        ),

                        CellState(
                            self.x - 2 - E,
                            self.y + 1,
                            Direction.NORTH,
                            screenshot_id=self.obstacle_id,
                            penalty=SNAPSHOT_COST
                        ),

                        CellState(
                            self.x - 2 - E,
                            self.y,
                            Direction.NORTH,
                            screenshot_id=self.obstacle_id,
                            penalty=0
                        ),
                    ]

                return [
                    state for state in view_states
                    if withinBounds(state.x, state.y)
                ]

    def _getViewStateFront6(self):
        view_states = []

        E = EXPANDED_CELL

        if self.direction == Direction.NORTH:


            view_states = [
                CellState(
                    self.x - 1,
                    self.y + 2 + E,
                    Direction.SOUTH,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),

                CellState(
                    self.x,
                    self.y + 2 + E,
                    Direction.SOUTH,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),

                CellState(
                    self.x + 1,
                    self.y + 2 + E,
                    Direction.SOUTH,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),

                CellState(
                    self.x - 1,
                    self.y + 1 + E,
                    Direction.SOUTH,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),

                CellState(
                    self.x,
                    self.y + 1 + E,
                    Direction.SOUTH,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),

                CellState(
                    self.x + 1,
                    self.y + 1 + E,
                    Direction.SOUTH,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),
            ]

        elif self.direction == Direction.SOUTH:


            view_states = [
                CellState(
                    self.x + 1,
                    self.y - 2 - E,
                    Direction.NORTH,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),

                CellState(
                    self.x,
                    self.y - 2 - E,
                    Direction.NORTH,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),

                CellState(
                    self.x - 1,
                    self.y - 2 - E,
                    Direction.NORTH,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),

                CellState(
                    self.x + 1,
                    self.y - 1 - E,
                    Direction.NORTH,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),

                CellState(
                    self.x,
                    self.y - 1 - E,
                    Direction.NORTH,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),

                CellState(
                    self.x - 1,
                    self.y - 1 - E,
                    Direction.NORTH,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),
            ]

        elif self.direction == Direction.EAST:


            view_states = [
                CellState(
                    self.x + 2 + E,
                    self.y + 1,
                    Direction.WEST,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),

                CellState(
                    self.x + 2 + E,
                    self.y,
                    Direction.WEST,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),

                CellState(
                    self.x + 2 + E,
                    self.y - 1,
                    Direction.WEST,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),

                CellState(
                    self.x + 1 + E,
                    self.y + 1,
                    Direction.WEST,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),

                CellState(
                    self.x + 1 + E,
                    self.y,
                    Direction.WEST,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),

                CellState(
                    self.x + 1 + E,
                    self.y - 1,
                    Direction.WEST,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),
            ]

        elif self.direction == Direction.WEST:


            view_states = [
                CellState(
                    self.x - 2 - E,
                    self.y - 1,
                    Direction.EAST,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),

                CellState(
                    self.x - 2 - E,
                    self.y,
                    Direction.EAST,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),

                CellState(
                    self.x - 2 - E,
                    self.y + 1,
                    Direction.EAST,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),

                CellState(
                    self.x - 1 - E,
                    self.y - 1,
                    Direction.EAST,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),

                CellState(
                    self.x - 1 - E,
                    self.y,
                    Direction.EAST,
                    screenshot_id=self.obstacle_id,
                    penalty=0
                ),

                CellState(
                    self.x - 1 - E,
                    self.y + 1,
                    Direction.EAST,
                    screenshot_id=self.obstacle_id,
                    penalty=SNAPSHOT_COST
                ),
            ]

        return [
            state for state in view_states
            if withinBounds(state.x, state.y)
        ]

    def _getViewStateLeft6(self):
            view_states = []

            E = EXPANDED_CELL

            if self.direction == Direction.NORTH:


                view_states = [
                    CellState(
                        self.x - 1,
                        self.y + 2 + E,
                        Direction.WEST,
                        screenshot_id=self.obstacle_id,
                        penalty=SNAPSHOT_COST
                    ),

                    CellState(
                        self.x,
                        self.y + 2 + E,
                        Direction.WEST,
                        screenshot_id=self.obstacle_id,
                        penalty=0
                    ),

                    CellState(
                        self.x + 1,
                        self.y + 2 + E,
                        Direction.WEST,
                        screenshot_id=self.obstacle_id,
                        penalty=SNAPSHOT_COST

                    ),

                    CellState(
                        self.x - 1,
                        self.y + 1 + E,
                        Direction.WEST,
                        screenshot_id=self.obstacle_id,
                        penalty=SNAPSHOT_COST
                    ),

                    CellState(
                        self.x,
                        self.y + 1 + E,
                        Direction.WEST,
                        screenshot_id=self.obstacle_id,
                        penalty=0
                    ),

                    CellState(
                        self.x + 1,
                        self.y + 1 + E,
                        Direction.WEST,
                        screenshot_id=self.obstacle_id,
                        penalty=SNAPSHOT_COST
                    ),
                ]

            elif self.direction == Direction.SOUTH:


                view_states = [
                    CellState(
                        self.x + 1,
                        self.y - 2 - E,
                        Direction.EAST,
                        screenshot_id=self.obstacle_id,
                        penalty=SNAPSHOT_COST
                    ),

                    CellState(
                        self.x,
                        self.y - 2 - E,
                        Direction.EAST,
                        screenshot_id=self.obstacle_id,
                        penalty=0
                    ),

                    CellState(
                        self.x - 1,
                        self.y - 2 - E,
                        Direction.EAST,
                        screenshot_id=self.obstacle_id,
                        penalty=SNAPSHOT_COST
                    ),

                    CellState(
                        self.x + 1,
                        self.y - 1 - E,
                        Direction.EAST,
                        screenshot_id=self.obstacle_id,
                        penalty=SNAPSHOT_COST
                    ),

                    CellState(
                        self.x,
                        self.y - 1 - E,
                        Direction.EAST,
                        screenshot_id=self.obstacle_id,
                        penalty=0
                    ),

                    CellState(
                        self.x - 1,
                        self.y - 1 - E,
                        Direction.EAST,
                        screenshot_id=self.obstacle_id,
                        penalty=SNAPSHOT_COST
                    ),
                ]

            elif self.direction == Direction.EAST:


                view_states = [
                    CellState(
                        self.x + 2 + E,
                        self.y + 1,
                        Direction.NORTH,
                        screenshot_id=self.obstacle_id,
                        penalty=SNAPSHOT_COST
                    ),

                    CellState(
                        self.x + 2 + E,
                        self.y,
                        Direction.NORTH,
                        screenshot_id=self.obstacle_id,
                        penalty=0
                    ),

                    CellState(
                        self.x + 2 + E,
                        self.y - 1,
                        Direction.NORTH,
                        screenshot_id=self.obstacle_id,
                        penalty=SNAPSHOT_COST
                    ),

                    CellState(
                        self.x + 1 + E,
                        self.y + 1,
                        Direction.NORTH,
                        screenshot_id=self.obstacle_id,
                        penalty=SNAPSHOT_COST
                    ),

                    CellState(
                        self.x + 1 + E,
                        self.y,
                        Direction.NORTH,
                        screenshot_id=self.obstacle_id,
                        penalty=0
                    ),

                    CellState(
                        self.x + 1 + E,
                        self.y - 1,
                        Direction.NORTH,
                        screenshot_id=self.obstacle_id,
                        penalty=SNAPSHOT_COST
                    ),
                ]

            elif self.direction == Direction.WEST:


                view_states = [
                    CellState(
                        self.x - 2 - E,
                        self.y - 1,
                        Direction.SOUTH,
                        screenshot_id=self.obstacle_id,
                        penalty=SNAPSHOT_COST
                    ),

                    CellState(
                        self.x - 2 - E,
                        self.y,
                        Direction.SOUTH,
                        screenshot_id=self.obstacle_id,
                        penalty=0
                    ),

                    CellState(
                        self.x - 2 - E,
                        self.y + 1,
                        Direction.SOUTH,
                        screenshot_id=self.obstacle_id,
                        penalty=SNAPSHOT_COST
                    ),

                    CellState(
                        self.x - 1 - E,
                        self.y - 1,
                        Direction.SOUTH,
                        screenshot_id=self.obstacle_id,
                        penalty=SNAPSHOT_COST
                    ),

                    CellState(
                        self.x - 1 - E,
                        self.y,
                        Direction.SOUTH,
                        screenshot_id=self.obstacle_id,
                        penalty=0
                    ),

                    CellState(
                        self.x - 1 - E,
                        self.y + 1,
                        Direction.SOUTH,
                        screenshot_id=self.obstacle_id,
                        penalty=SNAPSHOT_COST
                    ),
                ]

            return [
                state for state in view_states
                if withinBounds(state.x, state.y)
            ]

    def _getViewStateRight6(self):
                view_states = []

                E = EXPANDED_CELL

                if self.direction == Direction.NORTH:


                    view_states = [
                        CellState(
                            self.x - 1,
                            self.y + 2 + E,
                            Direction.EAST,
                            screenshot_id=self.obstacle_id,
                            penalty=SNAPSHOT_COST
                        ),

                        CellState(
                            self.x,
                            self.y + 2 + E,
                            Direction.EAST,
                            screenshot_id=self.obstacle_id,
                            penalty=0
                        ),

                        CellState(
                            self.x + 1,
                            self.y + 2 + E,
                            Direction.EAST,
                            screenshot_id=self.obstacle_id,
                            penalty=SNAPSHOT_COST
                        ),

                        CellState(
                            self.x - 1,
                            self.y + 1 + E,
                            Direction.EAST,
                            screenshot_id=self.obstacle_id,
                            penalty=SNAPSHOT_COST
                        ),

                        CellState(
                            self.x,
                            self.y + 1 + E,
                            Direction.EAST,
                            screenshot_id=self.obstacle_id,
                            penalty=0
                        ),

                        CellState(
                            self.x + 1,
                            self.y + 1 + E,
                            Direction.EAST,
                            screenshot_id=self.obstacle_id,
                            penalty=SNAPSHOT_COST
                        ),
                    ]

                elif self.direction == Direction.SOUTH:


                    view_states = [
                        CellState(
                            self.x + 1,
                            self.y - 2 - E,
                            Direction.WEST,
                            screenshot_id=self.obstacle_id,
                            penalty=SNAPSHOT_COST
                        ),

                        CellState(
                            self.x,
                            self.y - 2 - E,
                            Direction.WEST,
                            screenshot_id=self.obstacle_id,
                            penalty=0
                        ),

                        CellState(
                            self.x - 1,
                            self.y - 2 - E,
                            Direction.WEST,
                            screenshot_id=self.obstacle_id,
                            penalty=SNAPSHOT_COST
                        ),

                        CellState(
                            self.x + 1,
                            self.y - 1 - E,
                            Direction.WEST,
                            screenshot_id=self.obstacle_id,
                            penalty=SNAPSHOT_COST
                        ),

                        CellState(
                            self.x,
                            self.y - 1 - E,
                            Direction.WEST,
                            screenshot_id=self.obstacle_id,
                            penalty=0
                        ),

                        CellState(
                            self.x - 1,
                            self.y - 1 - E,
                            Direction.WEST,
                            screenshot_id=self.obstacle_id,
                            penalty=SNAPSHOT_COST
                        ),
                    ]

                elif self.direction == Direction.EAST:


                    view_states = [
                        CellState(
                            self.x + 2 + E,
                            self.y + 1,
                            Direction.SOUTH,
                            screenshot_id=self.obstacle_id,
                            penalty=SNAPSHOT_COST
                        ),

                        CellState(
                            self.x + 2 + E,
                            self.y,
                            Direction.SOUTH,
                            screenshot_id=self.obstacle_id,
                            penalty=0
                        ),

                        CellState(
                            self.x + 2 + E,
                            self.y - 1,
                            Direction.SOUTH,
                            screenshot_id=self.obstacle_id,
                            penalty=SNAPSHOT_COST
                        ),

                        CellState(
                            self.x + 1 + E,
                            self.y + 1,
                            Direction.SOUTH,
                            screenshot_id=self.obstacle_id,
                            penalty=SNAPSHOT_COST
                        ),

                        CellState(
                            self.x + 1 + E,
                            self.y,
                            Direction.SOUTH,
                            screenshot_id=self.obstacle_id,
                            penalty=0
                        ),

                        CellState(
                            self.x + 1 + E,
                            self.y - 1,
                            Direction.SOUTH,
                            screenshot_id=self.obstacle_id,
                            penalty=SNAPSHOT_COST
                        ),
                    ]

                elif self.direction == Direction.WEST:


                    view_states = [
                        CellState(
                            self.x - 2 - E,
                            self.y - 1,
                            Direction.NORTH,
                            screenshot_id=self.obstacle_id,
                            penalty=SNAPSHOT_COST
                        ),

                        CellState(
                            self.x - 2 - E,
                            self.y,
                            Direction.NORTH,
                            screenshot_id=self.obstacle_id,
                            penalty=0
                        ),

                        CellState(
                            self.x - 2 - E,
                            self.y + 1,
                            Direction.NORTH,
                            screenshot_id=self.obstacle_id,
                            penalty=SNAPSHOT_COST
                        ),

                        CellState(
                            self.x - 1 - E,
                            self.y - 1,
                            Direction.NORTH,
                            screenshot_id=self.obstacle_id,
                            penalty=SNAPSHOT_COST
                        ),

                        CellState(
                            self.x - 1 - E,
                            self.y,
                            Direction.NORTH,
                            screenshot_id=self.obstacle_id,
                            penalty=0
                        ),

                        CellState(
                            self.x - 1 - E,
                            self.y + 1,
                            Direction.NORTH,
                            screenshot_id=self.obstacle_id,
                            penalty=SNAPSHOT_COST
                        ),
                    ]

                return [
                    state for state in view_states
                    if withinBounds(state.x, state.y)
                ]

    def getViewState(self, retrying):

        if retrying:
            return []

        if CAMERA_DIRECTION == 'FRONT':
            if NUM_VIEWPOINTS == 4:
                return self._getViewStateFront4()
            elif NUM_VIEWPOINTS == 6:
                return self._getViewStateFront6()

        elif CAMERA_DIRECTION == 'RIGHT':
            if NUM_VIEWPOINTS == 2:
                if ALLOW_45 == True:
                    return self._getViewStateRight2Allow45()
                elif ALLOW_45 == False:
                    return self._getViewStateRight2()
            elif NUM_VIEWPOINTS == 4:
                return self._getViewStateRight4()
            elif NUM_VIEWPOINTS == 5:
                return self._getViewStateRight5()
            elif NUM_VIEWPOINTS == 6:
                return self._getViewStateRight6()

        elif CAMERA_DIRECTION == 'LEFT':
            if NUM_VIEWPOINTS == 4:
                return self._getViewStateLeft4()
            elif NUM_VIEWPOINTS == 6:
                return self._getViewStateLeft6()

        return []

class Grid:

    def __init__(self, size_x: int, size_y: int):

        self.size_x = size_x
        self.size_y = size_y
        self.obstacles: List[Obstacle] = []
        self._reachable_cache = None

    def addObstacle(self, obstacle: Obstacle):


        to_add = True
        for ob in self.obstacles:
            if ob == obstacle:
                to_add = False
                break

        if to_add:
            self.obstacles.append(obstacle)
            self._reachable_cache = None

    def reset_obstacles(self):

        self.obstacles = []
        self._reachable_cache = None

    def get_obstacles(self):

        return self.obstacles

    def _reachable_uncached(self, x: int, y: int, turn=False, preTurn=False) -> bool:
        if not self.is_valid_coord(x, y):
            return False

        for ob in self.obstacles:
            if ob.x == 4 and ob.y <= 4 and x < 4 and y < 4:
                continue
            if abs(ob.x - x) + abs(ob.y - y) >= 4:
                continue
            if turn or preTurn:
                if max(abs(ob.x - x), abs(ob.y - y)) < EXPANDED_CELL * 2 + 1:
                    return False
            else:
                if max(abs(ob.x - x), abs(ob.y - y)) < CLEARANCE:
                    return False

        return True

    def _build_reachable_cache(self):
        cache = {}
        for x in range(self.size_x):
            for y in range(self.size_y):
                for turn in (False, True):
                    for preTurn in (False, True):
                        cache[(x, y, turn, preTurn)] = self._reachable_uncached(x, y, turn, preTurn)
        self._reachable_cache = cache

    def reachable(self, x: int, y: int, turn=False, preTurn=False) -> bool:
        if self._reachable_cache is None:
            self._build_reachable_cache()
        return self._reachable_cache.get((x, y, turn, preTurn), False)

    def is_valid_coord(self, x: int, y: int) -> bool:

        if (x < BORDER_MARGIN or x >= self.size_x - BORDER_MARGIN
                or y < BORDER_MARGIN or y >= self.size_y - BORDER_MARGIN):
            return False
        return True

    def is_valid_cell_state(self, state: CellState) -> bool:

        return self.is_valid_coord(state.x, state.y)

    def get_view_obstacle_positions(self, retrying) -> List[List[CellState]]:
        optimal_positions = []

        for obstacle in sorted(self.obstacles, key=lambda ob: ob.obstacle_id):
            if obstacle.direction == 8:
                continue

            view_states = [
                vs for vs in obstacle.getViewState(retrying)
                if self.reachable(vs.x, vs.y)
            ]

            optimal_positions.append(view_states)

        return optimal_positions
