import numpy as np
def generate_gridmap(rows, cols):
    grid = np.random.randint(0, 2, size=(rows, cols))
    grid[0][0] = 0
    grid[rows-1][cols-1] = 0
    return grid
# Initialize the A* path follower
# (0, 0)(1, 1)(2, 1)(3, 2)(3, 3)(4, 3)(5, 4)(5, 5)(5, 6)(6, 5)(7, 6)(7, 7)(7, 8)(8, 9)(9, 9)
row=10
col=10
grid = generate_gridmap(row,col)
print(grid)