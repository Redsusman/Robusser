import heapq
import math
import matplotlib.pyplot as plt
import numpy as np
        
class Point:
    def __init__(self, x: float, y: float, theta: float):
        self.x = x
        self.y = y
        self.theta=theta
        self.point = (x, y,theta)


class Node:
    def __init__(self, value, cell_idx, cell_idy, theta):
        self.x=0.0
        self.y=0.0
        self.theta=0.0
        self.value = value
        self.idx = cell_idx
        self.idy = cell_idy
        self.theta = theta
        self.g = 0
        self.h = 0
        self.f = 0
        self.parent = None
        self.neighbors = []
        visited = False

    def __lt__(self, other):
        return self.f < other.f

    def __eq__(self, other):
        return self.idx == other.idx and self.idy == other.idy


class Kinematic_AStar_Path_Follower:
    def __init__(self, map_grid, max_vel, max_omega, robot_radius):
        self.max_vel = max_vel
        self.max_omega = max_omega
        self.map_grid = map_grid
        self.robot_radius = robot_radius
        self.node_grid = [
            [
                Node(self.map_grid[row][col], row, col,0)
                for col in range(len(self.map_grid[0]))
            ]
            for row in range(len(self.map_grid))
        ]

    def reconstruct_path(self, end_node):
        path = []
        current = end_node
        while current is not None:
            path.append(Point(current.idx, current.idy, current.theta))
            current = current.parent
        path.reverse()
        return path
    
    def calculate_trajectory(self, v, omega, dt, start_x=0.0, start_y=0.0, start_theta=0.0):
        x = start_x
        y = start_y
        theta = start_theta
        trajectory = []

        num_steps = 20
        delta_t = dt / num_steps

        for _ in range(num_steps):
        # Save current position
            trajectory.append(Point(x, y, theta))
        # Update state
            x += v * math.cos(theta) * delta_t
            y += v * math.sin(theta) * delta_t
            theta += omega * delta_t

        return trajectory


    def is_collision(self, x, y):
        grid_rows = len(self.map_grid)
        grid_cols = len(self.map_grid[0]) if grid_rows > 0 else 0

        # Convert to integer grid indices
        i = int(round(y))  # rows correspond to y
        j = int(round(x))  # cols correspond to x

        # Check out-of-bounds
        if i < 0 or j < 0 or i >= grid_rows or j >= grid_cols:
            return True  # Treat out-of-bounds as collision

        # Check if robot is on an obstacle cell
        if self.map_grid[i][j] == 1:
            return True

        # Optional: inflate by robot radius (check 8-connected neighbors)
        radius = int(math.ceil(self.robot_radius))
        for di in range(-radius, radius + 1):
            for dj in range(-radius, radius + 1):
                ni = i + di
                nj = j + dj
                if 0 <= ni < grid_rows and 0 <= nj < grid_cols:
                    if self.map_grid[ni][nj] == 1:
                        return True

        return False
    
    def calculate_distance(self, node1, node2):
        dx = node2.x - node1.x
        dy = node2.y - node1.y
        return math.hypot(dx, dy)
    
    def heuristic(self, node, goal):
        dx = goal.x - node.x
        dy = goal.y - node.y
        dtheta = abs(goal.theta - node.theta)
        dtheta = min(dtheta, 2 * math.pi - dtheta)  # Wrap-around
        return math.hypot(dx, dy) + 0.1 * dtheta  # Small weight on angle
    
    def find_path(self, start, goal):
        open_set = [start]
        heapq.heapify(open_set)
        closed_set = set()

        while open_set:
            current_node = heapq.heappop(open_set)
            current_pos = (round(current_node.x, 2), round(current_node.y, 2), round(current_node.theta, 2))

            if current_node == goal:  # Use tolerance-based check for x, y, theta
                return self.reconstruct_path(current_node)

            closed_set.add(current_pos)

            possible_velocities = np.linspace(0, self.max_vel, num=25)
            possible_omegas = np.linspace(-self.max_omega, self.max_omega, num=25)

            for velocity in possible_velocities:
                for omega in possible_omegas:
                    traj = list(self.calculate_trajectory(current_node.x, current_node.y, current_node.theta, velocity, omega))

                    collision = False
                    for point in traj:
                        if self.is_collision(point.x, point.y):
                            collision = True
                            break
                    if collision:
                        continue

                    final_point = traj[-1]
                    neighbor_pos = (round(final_point.x, 2), round(final_point.y, 2), round(final_point.theta, 2))
                    if neighbor_pos in closed_set:
                        continue

                    neighbor = Node(value=0, cell_idx=final_point.x, cell_idy=final_point.y, theta=final_point.theta)
                    neighbor.x = final_point.x
                    neighbor.y = final_point.y
                    neighbor.theta = final_point.theta

                    tentative_g = current_node.g + self.calculate_distance(current_node, neighbor)

                    if not hasattr(neighbor, 'g') or tentative_g < neighbor.g:
                        neighbor.g = tentative_g
                        neighbor.h = self.heuristic(neighbor, goal)
                        neighbor.f = neighbor.g + neighbor.h
                        neighbor.parent = current_node
                        heapq.heappush(open_set, neighbor)

    def search_neighbor(self, grid, cell_i, cell_j):
        offsets = [(1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1)]
        neighbors = []
        for x, y in offsets:
            neighbor_x, neighbor_y = cell_i + x, cell_j + y
            if 0 <= neighbor_x < len(grid) and 0 <= neighbor_y < len(grid[0]):
                neighbors.append((grid[neighbor_x][neighbor_y]))
        return neighbors

    

grid =  [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
        [0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
        [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
        [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
        [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
        [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
        [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
        [1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1],
        [1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1],
        [1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0],
        [1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1],
        [1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1],
        [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
        [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
        [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
        [1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1],
        [1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1],
        [1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1],
        [1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1],
        [1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1],
        [1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1],
        [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1],
        [1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1],
        [1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1],
        [1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1],
        [1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1],
        [1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1],
        [1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1],
        [1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]]

astar = Kinematic_AStar_Path_Follower(grid,1.0, math.pi/2,0.5)

path = astar.find_path(
    Node(value=0, cell_idx=0, cell_idy=0, theta=0),
    Node(value=0, cell_idx=5, cell_idy=6, theta=0)
)

x_list = []
y_list = []

if path is not None:
    for node in path:
        print(node.x)
        x_list.append(node.x)
        y_list.append(node.y)

plt.plot(x_list, y_list, marker='o', color='blue')



# x = []
# y = []
# colors = []

# for row in astar.node_grid:
#     for node in row:
#         x.append(node.idx)  # Assuming node.x contains the x-coordinate
#         y.append(node.idy)  # Assuming node.y contains the y-coordinate
#         colors.append("red" if node.value == 1 else "blue")  # Color based on node value

# # Plot the grid

# plt.figure(figsize=(10, 10))
# plt.scatter(x, y, c=colors, s=10)  # Ensure 'c' matches the size of 'x' and 'y'
# plt.xlabel("X Coordinate")
# plt.ylabel("Y Coordinate")
# plt.title("Inflated Grid Visualization")
# plt.grid()
plt.show()