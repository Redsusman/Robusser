import heapq
import math
import matplotlib.pyplot as plt
import numpy as np

class Point:
    def __init__(self, x: float, y: float, theta: float):
        self.x = x
        self.y = y
        self.theta = theta

class Node:
    def __init__(self, value, cell_idx, cell_idy, theta):
        self.x = cell_idx  # Changed from 0.0 to cell_idx
        self.y = cell_idy  # Changed from 0.0 to cell_idy
        self.theta = theta
        self.value = value
        self.idx = cell_idx
        self.idy = cell_idy
        self.g = float('inf')
        self.h = 0
        self.f = float('inf')
        self.parent = None
        self.visited = False  # Fixed: made it an instance variable

    def __lt__(self, other):
        return self.f < other.f

    def __eq__(self, other):
        return (self.idx == other.idx and 
                self.idy == other.idy and 
                abs(self.theta - other.theta) < math.pi/4)  # Added theta comparison

class Kinematic_AStar_Path_Follower:
    def __init__(self, map_grid, max_vel, max_omega, robot_radius):
        self.max_vel = max_vel
        self.max_omega = max_omega
        self.map_grid = map_grid
        self.robot_radius = robot_radius
        self.node_grid = [
            [
                Node(self.map_grid[row][col], row, col, 0)
                for col in range(len(self.map_grid[0]))
            for row in range(len(self.map_grid))
        ]]
    
    def calculate_distance(self, node1, node2):
        """Calculate Euclidean distance between two nodes."""
        dx = node2.x - node1.x
        dy = node2.y - node1.y
        return math.sqrt(dx**2 + dy**2)

    def reconstruct_path(self, end_node):
        path = []
        current = end_node
        while current is not None:
            path.append(Point(current.x, current.y, current.theta))
            current = current.parent
        return path[::-1]  # Reverse the path

    def calculate_trajectory(self, v, omega, dt, start_x, start_y, start_theta):
        x = start_x
        y = start_y
        theta = start_theta
        trajectory = []
        num_steps = 20
        delta_t = dt / num_steps

        for _ in range(num_steps):
            trajectory.append(Point(x, y, theta))
            x += v * math.cos(theta) * delta_t
            y += v * math.sin(theta) * delta_t
            theta += omega * delta_t
            theta = theta % (2 * math.pi)  # Normalize angle
        return trajectory

    def is_collision(self, x, y):
        grid_rows = len(self.map_grid)
        grid_cols = len(self.map_grid[0]) if grid_rows > 0 else 0

        # Convert to integer grid indices
        i = int(round(y))
        j = int(round(x))

        # Check out-of-bounds
        if i < 0 or j < 0 or i >= grid_rows or j >= grid_cols:
            return True

        # Check if robot is on an obstacle cell
        if self.map_grid[i][j] == 1:
            return True

        # Check neighbors for collision (with robot radius)
        radius = int(math.ceil(self.robot_radius))
        for di in range(-radius, radius + 1):
            for dj in range(-radius, radius + 1):
                ni, nj = i + di, j + dj
                if 0 <= ni < grid_rows and 0 <= nj < grid_cols:
                    if self.map_grid[ni][nj] == 1:
                        # Calculate actual distance
                        dist = math.sqrt(di**2 + dj**2)
                        if dist <= self.robot_radius:
                            return True
        return False

    def heuristic(self, node, goal):
        dx = goal.x - node.x
        dy = goal.y - node.y
        dtheta = min(abs(goal.theta - node.theta), 2 * math.pi - abs(goal.theta - node.theta))
        return math.sqrt(dx**2 + dy**2) + 0.2 * dtheta  # Adjusted weight
    
    def discretize(self,node, res=0.25, angle_bins=8):
        xi = round(node.x / res)
        yi = round(node.y / res)
        ti = round((node.theta % (2 * math.pi)) / (2 * math.pi / angle_bins))
        return (xi, yi, ti)

    # def find_path(self, start, goal):
    #     open_set = []
    #     start.g = 0
    #     start.h = self.heuristic(start, goal)
    #     start.f = start.g + start.h
    #     heapq.heappush(open_set, start)
        
    #     closed_set = set()

    #     while open_set:
          
    #         current_node = heapq.heappop(open_set)
            
    #         # Check if current node is close enough to goal
    #         if (abs(current_node.x - goal.x) < 0.5 and 
    #             abs(current_node.y - goal.y) < 0.5 and 
    #             min(abs(current_node.theta - goal.theta), 
    #                 2*math.pi - abs(current_node.theta - goal.theta)) < math.pi/4):
    #             return self.reconstruct_path(current_node)

    #         closed_set.add((current_node.x, current_node.y, current_node.theta))

    #         # Generate motion primitives
    #         possible_velocities = np.linspace(0, self.max_vel, num=5)
    #         possible_omegas = np.linspace(-self.max_omega, self.max_omega, num=5)
    #         dt = 0.5  # time step

    #         for v in possible_velocities:
    #             for omega in possible_omegas:
    #                 # print(v,omega)
    #                 if v == 0 and omega == 0:
    #                     continue  # skip zero motion
                        
    #                 traj = self.calculate_trajectory(
    #                     v, omega, dt, 
    #                     current_node.x, current_node.y, current_node.theta
    #                 )
                    
    #                 # Check for collisions along trajectory
    #                 collision = False
    #                 for point in traj:
    #                     if self.is_collision(point.x, point.y):
    #                         collision = True
    #                         break
    #                 if collision:
    #                     continue

    #                 final_point = traj[-1]
    #                 print(final_point.x, final_point.y, final_point.theta)
    #                 neighbor_pos = (final_point.x, final_point.y, final_point.theta)
                    
    #                 # Skip if already in closed set
    #                 if neighbor_pos in closed_set:
    #                     continue

    #                 # Create neighbor node
    #                 neighbor = Node(
    #                     0, final_point.x, final_point.y, final_point.theta
    #                 )
    #                 neighbor.g = current_node.g + math.sqrt(
    #                     (final_point.x - current_node.x)**2 + 
    #                     (final_point.y - current_node.y)**2
    #                 )
    #                 neighbor.h = self.heuristic(neighbor, goal)
    #                 neighbor.f = neighbor.g + neighbor.h
    #                 neighbor.parent = current_node

    #                 # Check if this neighbor is already in open set with lower cost
    #                 found = False
    #                 for open_node in open_set:
    #                     if (abs(open_node.x - neighbor.x) < 0.1 and 
    #                         abs(open_node.y - neighbor.y) < 0.1 and 
    #                         abs(open_node.theta - neighbor.theta) < math.pi/4):
    #                         if open_node.f <= neighbor.f:
    #                             found = True
    #                             break
    #                         else:
    #                             open_set.remove(open_node)
    #                             heapq.heapify(open_set)
    #                             break
                    
    #                 if not found:
    #                     heapq.heappush(open_set, neighbor)

    #     return None  # No path found

    def find_path(self, start, goal):
        open_set = []
        open_dict = {}  # Maps discretized (x, y, theta) to cost
        closed_set = set()

    # Heuristic function

    # Discretization


    # Initialize
        start.g = 0
        start.f = self.heuristic(start,goal)
        heapq.heappush(open_set, (start.f, start))
        open_dict[self.discretize(start)] = start.f

        while open_set:
            _, current_node = heapq.heappop(open_set)
            current_state = self.discretize(current_node)

            if current_state in closed_set:
                continue
            closed_set.add(current_state)

            if self.heuristic(current_node,goal) < 0.05:
                return self.reconstruct_path(current_node)

        # Motion primitives
            for velocity in np.linspace(0, self.max_vel, 3):
                for omega in np.linspace(-self.max_omega, self.max_omega, 5):
                    trajectory = self.calculate_trajectory(current_node, velocity, omega, current_node.x, current_node.y, current_node.theta)

                    if not trajectory or self.is_collision(trajectory[-1][0], trajectory[-1][1]):
                        continue

                    final_point = trajectory[-1]
                    neighbor = Node(0, final_point.x, final_point.y, final_point.theta)
                    neighbor.parent = current_node
                    neighbor.g = current_node.g + self.calculate_distance(current_node, neighbor)
                    neighbor.f = neighbor.g + self.heuristic(neighbor,goal)

                    neighbor_state = self.discretize(neighbor)

                    if neighbor_state in closed_set:
                        continue

                    if neighbor_state not in open_dict or neighbor.f < open_dict[neighbor_state]:
                        open_dict[neighbor_state] = neighbor.f
                        heapq.heappush(open_set, (neighbor.f, neighbor))

        return None  # No path found

# Test the algorithm
grid = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
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

astar = Kinematic_AStar_Path_Follower(grid, 1.0, math.pi/4, 0.5)

start_node = Node(0, 0, 0, 0)
goal_node = Node(0, 7, 7, 0)

path = astar.find_path(start_node, goal_node)

if path:
    x_list = [p.x for p in path]
    y_list = [p.y for p in path]
    
    plt.figure(figsize=(10, 10))
    
    # Plot the grid
    for y in range(len(grid)):
        for x in range(len(grid[0])):
            if grid[y][x] == 1:
                plt.plot(x, y, 'sk')  # Obstacles in black
    
    # Plot the path
    plt.plot(x_list, y_list, '-o', color='blue', linewidth=2, markersize=5)
    plt.plot(start_node.x, start_node.y, 'og', markersize=10)  # Start in green
    plt.plot(goal_node.x, goal_node.y, 'or', markersize=10)    # Goal in red
    
    plt.grid()
    plt.title("Kinematic A* Path Planning")
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.show()
else:
    print("No path found!")