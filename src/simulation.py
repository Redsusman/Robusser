import numpy as np
grid = [
    [1,2,3,4,5],
    [6,7,8,9,10],
    [11,12,13,14,15],
    [16, 17,18,19,20],
    [21,22,23,24,25],
    [25,26,27,28,29,30],
]

class Node:
    def __init__(self, value, cell_x, cell_y):
        self.value = value
        self.string_val = str(value)

node_grid = [[Node(grid[row][col], row, col) for col in range(len(grid[0]))] for row in range(len(grid))]
