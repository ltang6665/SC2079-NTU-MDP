from typing import List
from pathfinding.entities.arena import CellState
from pathfinding.settings import Direction


class Robot:
    def __init__(self, center_x: int, center_y: int, start_direction: Direction):

        self.states: List[CellState] = [
            CellState(center_x, center_y, start_direction)]

    def get_start_state(self):

        return self.states[0]
