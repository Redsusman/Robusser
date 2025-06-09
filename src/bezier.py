import matplotlib.pyplot as plt
import math
import numpy as np
import heapq

searched_poses = []

class Nodee:
    def __init__(self, value, cell_idx, cell_idy):
        self.value = value
        self.idx = cell_idx
        self.idy = cell_idy
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



def sign(x):
    """Return the sign of x."""
    return (x > 0) - (x < 0)

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.point = (x, y)

class Pose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.pose = (x, y, theta)

class Node:
    def __init__(self, x, y, theta, value, resolution=0.1, angle_resolution=0.1):
        self.resolution = resolution
        self.angle_res = angle_resolution
        self.x = x
        self.y = y
        self.theta = theta
        self.idx = x
        self.idy = y
        self.idt = theta
        self.value = value
        self.h = 0.0
        self.f = 0.0
        self.g = 0.0
        self.parent = None
        self.neighbors = []
        self.path_from_parent: list[Pose] = []  # Add path_from_parent attribute with explicit type
        self.pose = Pose(self.x, self.y, self.theta)
        self.visited = False
        self.velocity_sign = 0.0

    def __lt__(self, other):
        return self.f < other.f

    def __eq__(self, other):
        return (
            self.idx == other.idx
            and self.idy == other.idy
            and self.theta == other.theta
        )

    def __hash__(self):
        return hash((self.idx, self.idy, self.theta))

def calculate_optimal_path(pose_a, pose_b, num_points) -> list[Pose]:
    p0 = [pose_a.x, pose_a.y]
    p1 = [pose_b.x, pose_b.y]
    
    dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    v0 = [math.cos(pose_a.theta) * dist * 0.5, math.sin(pose_a.theta) * dist * 0.5]
    v1 = [math.cos(pose_b.theta) * dist * 0.5, math.sin(pose_b.theta) * dist * 0.5]
    
    h00 = lambda t: (1 - 3 * t**2 + 2 * t**3)
    h10 = lambda t: (t - 2 * t**2 + t**3)
    h01 = lambda t: (3 * t**2 - 2 * t**3)
    h11 = lambda t: (-(t**2) + t**3)
    
    points = []
    for i in range(num_points):
        t = i / (num_points - 1)
        position_x = h00(t) * p0[0] + h10(t) * v0[0] + h01(t) * p1[0] + h11(t) * v1[0]
        position_y = h00(t) * p0[1] + h10(t) * v0[1] + h01(t) * p1[1] + h11(t) * v1[1]
        points.append(Pose(position_x, position_y, 0.0))  # Default theta to 0.0
    return points



class HybridStar_Path_Follower:
    def __init__(self, map_grid, robot_radius):
        self.map_grid = map_grid
        self.node_grid = [
            [
                Node(row, col, 0.0, self.map_grid[row][col])
                for col in range(len(self.map_grid[0]))
            ]
            for row in range(len(self.map_grid))
        ]
        self.robot_radius = robot_radius

    def is_collision(self, x, y):
    # Convert to grid coordinates
        grid_x = int(math.floor(x))
        grid_y = int(math.floor(y))

    # Check bounds
        if (
        grid_x < 0
        or grid_x >= len(self.node_grid)
        or grid_y < 0
        or grid_y >= len(self.node_grid[0])
        ):
            return True

    # Check the exact cell first
        if self.node_grid[grid_x][grid_y].value == 1:
            return True

    # Only check surrounding cells if radius is large enough to reach them
        if self.robot_radius > 0:
            radius_cells = math.ceil(self.robot_radius)
        # Calculate the actual squared distance threshold
            radius_sq = self.robot_radius**2
        
            for dx in range(-radius_cells, radius_cells + 1):
                for dy in range(-radius_cells, radius_cells + 1):
                # Skip the center cell (already checked)
                    if dx == 0 and dy == 0:
                        continue

                # Check if this cell could possibly be within the circle
                    if (dx**2 + dy**2) > (radius_cells + 0.5)**2:
                        continue

                    nx, ny = grid_x + dx, grid_y + dy
                    if 0 <= nx < len(self.node_grid) and 0 <= ny < len(self.node_grid[0]):
                        if self.node_grid[nx][ny].value == 1:
                        # Check exact distance from original point
                        # Using the center of each cell for more accurate collision
                            cell_center_x = nx + 0.5
                            cell_center_y = ny + 0.5
                            distance_sq = (x - cell_center_x)**2 + (y - cell_center_y)**2
                            if distance_sq <= radius_sq:
                                return True
        return False

    def is_outside_grid(self, x, y):
        if x < 0 or x >= len(self.node_grid) or y < 0 or y >= len(self.node_grid[0]):
            return True
        return False

    def search_neighbor(self, grid, cell_i, cell_j):
        offsets = [(1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1)]
        neighbors = []
        for x, y in offsets:
            neighbor_x, neighbor_y = cell_i + x, cell_j + y
            if 0 <= neighbor_x < len(grid) and 0 <= neighbor_y < len(grid[0]):
                neighbors.append((grid[neighbor_x][neighbor_y]))
        return neighbors


    def reconstruct_path(self, end_node):
        path = []
        current = end_node
        while current is not None:
            path.append(Pose(current.x, current.y, current.theta))  # Store as Pose
            current = current.parent
        path.reverse()
        return path
    # def reconstruct_path(self, node):
    #     path = []
    #     while node.parent is not None:
    #         path = node.path_from_parent + path
    #         node = node.parent
    #     path = [node.pose] + path  # Include start pose
    #     return path
    def heuristic(self, a, b):
        dx = b.x - a.x
        dy = b.y - a.y
        dist = math.hypot(dx, dy) # Add angle cost to heuristic
        # Add directional component (penalize moving away)
    
        return dist

    def calculate_trajectory(self, v, omega, max_time, steps, initial_pose):
        poses = []
        time = np.linspace(0, max_time, steps)
        x, y, theta = initial_pose.x, initial_pose.y, initial_pose.theta
        for t in time:
            theta += omega * t
            theta = (theta + math.pi) % (2 * math.pi) - math.pi  # Normalize angle to [-pi, pi]
            x += v * math.cos(theta)
            y += v * math.sin(theta)
            poses.append(Pose(x, y, theta))
        return poses

    def is_trajectory_collision(self, trajectory):
        for i in range(len(trajectory)):
            pose = trajectory[i]
            # Check current pose
            if self.is_collision(pose.x, pose.y):
                return True

            # Check intermediate points between current and previous pose
            if i > 0:
                prev_pose = trajectory[i - 1]
                # Interpolate between points
                steps = max(abs(pose.x - prev_pose.x), abs(pose.y - prev_pose.y)) * 2
                for t in np.linspace(0, 1, int(steps)):
                    interp_x = prev_pose.x + t * (pose.x - prev_pose.x)
                    interp_y = prev_pose.y + t * (pose.y - prev_pose.y)
                    if self.is_collision(interp_x, interp_y):
                        return True
        return False

    def calculate_arc_length(self, trajectory):
        dist = 0.0
        for i in range(1, len(trajectory)):
            dx = trajectory[i].x - trajectory[i - 1].x
            dy = trajectory[i].y - trajectory[i - 1].y
            dist += math.hypot(dx, dy)  # Update dist inside the loop
        return dist  # Explicitly return the calculated distance

    def is_goal_reached(self, node, end, pos_tolerance=0.3, angle_tolerance=0.1):
        dist = math.hypot(node.x - end.x, node.y - end.y)
        pos_close = dist <= pos_tolerance
        angle_diff = abs((node.theta - end.theta + math.pi) % (2 * math.pi) - math.pi)
        # print(dist,angle_diff)
        return pos_close and (angle_diff <= angle_tolerance)

    def angle_cost(self, pose_a, pose_b):
        pose_a_theta = pose_a.theta
        pose_b_theta = pose_b.theta
        angle_diff = abs(
            (pose_a_theta - pose_b_theta + math.pi) % (2 * math.pi) - math.pi
        )
        return angle_diff  # Weight factor
    
    def generate_trajectories(self, initial_pose, v_max, ω_max):
        trajectories = []
        motions = [
            (v_max, 0.0), 
            (v_max, ω_max ),  
            (v_max, -ω_max), 
            (v_max, ω_max / 2),
            (v_max, -ω_max / 2), 
            (-v_max, 0.0), 
            (-v_max, ω_max ),  
            (-v_max, -ω_max), 
            (-v_max, ω_max / 2),
            (-v_max, -ω_max / 2), 
            
        ]
        for v, omega in motions:
            trajectory = self.calculate_trajectory(v, omega, 1.0, 5, initial_pose)
            trajectories.append((trajectory, sign(v)))
        return trajectories

    def calculate_cost(self, trajectory, node_a, node_b, velocity_sign):
        base_cost = self.calculate_arc_length(trajectory)
        angle_cost = self.angle_cost(node_a.pose, node_b.pose)
        reverse_cost = 1.0 if velocity_sign < 0 else 0.0
        return base_cost + angle_cost + reverse_cost

    def find_path(self, start, end, v_max, ω_max):
        open = [start]
        heapq.heapify(open)
        closed = set()
        while open:
            current = heapq.heappop(open)
            optimal_path = calculate_optimal_path(current.pose, end.pose, 15)
            if not self.is_trajectory_collision(optimal_path) and not self.is_outside_grid(optimal_path[-1].x, optimal_path[-1].y):
                path_to_current = self.reconstruct_path(current)
                return path_to_current + optimal_path[1:]
            if self.is_goal_reached(current, end):
                return self.reconstruct_path(current)
            closed.add(current)
            trajectories = self.generate_trajectories(current.pose, v_max, ω_max)
            searched_poses.append(Pose(current.x, current.y,0.0))
            for trajectory, velocity_sign in trajectories:
                if self.is_trajectory_collision(trajectory):
                    continue
                end_pose = trajectory[-1]
                searched_poses.append(end_pose)
                if self.is_outside_grid(end_pose.x, end_pose.y):
                    continue
                child_node = Node(end_pose.x, end_pose.y, end_pose.theta, 0)
                if child_node in closed:
                    continue
                cost = self.calculate_cost(trajectory, current, child_node, velocity_sign)
                child_node.g = current.g + cost
                child_node.h = self.heuristic(child_node.pose, end)
                child_node.f = child_node.g + child_node.h
                child_node.parent = current
                child_node.velocity_sign = velocity_sign
                
                heapq.heappush(open, child_node)
            print(current.x, current.y, current.theta)
        print("hello")
        return None

   
    def quantize(self, node: Node, resolution):
        return (round(node.x / resolution) * resolution, 
            round(node.y / resolution) * resolution, 
            round(node.theta / resolution) * resolution)

def simplify_path(points, epsilon):
    """Ramer-Douglas-Peucker path simplification for Point class"""
    if len(points) < 3:
        return points.copy()
    
    # Find point with maximum distance
    dmax = 0
    index = 0
    end = len(points) - 1
    
    for i in range(1, end):
        d = perpendicular_distance(points[i], points[0], points[end])
        if d > dmax:
            index = i
            dmax = d
    
    # If max distance > epsilon, recursively simplify
    if dmax > epsilon:
        left = simplify_path(points[:index+1], epsilon)
        right = simplify_path(points[index:], epsilon)
        return left[:-1] + right
    else:
        return [points[0], points[end]]

def perpendicular_distance(point, line_start, line_end):
    """Calculate perpendicular distance from Point to line segment"""
    x, y = point.point
    x1, y1 = line_start.point
    x2, y2 = line_end.point
    
    if x1 == x2 and y1 == y2:
        return math.hypot(x-x1, y-y1)
    
    numerator = abs((x2-x1)*(y1-y) - (x1-x)*(y2-y1))
    denominator = math.hypot(x2-x1, y2-y1)
    return numerator / denominator

class AStar_Path_Follower:
    def __init__(self, map_grid):
        self.map_grid = map_grid
        self.node_grid = [
            [
                Nodee(self.map_grid[row][col], row, col)
                for col in range(len(self.map_grid[0]))
            ]
            for row in range(len(self.map_grid))
        ]

    def search_neighbor(self, grid, cell_i, cell_j):
        offsets = [(1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1)]
        neighbors = []
        for x, y in offsets:
            neighbor_x, neighbor_y = cell_i + x, cell_j + y
            if 0 <= neighbor_x < len(grid) and 0 <= neighbor_y < len(grid[0]):
                neighbors.append((grid[neighbor_x][neighbor_y]))
        return neighbors

    def reconstruct_path(self, end_node):
        path = []
        current = end_node
        while current is not None:
            path.append(Point(current.idx, current.idy))
            current = current.parent
        path.reverse()
        return path

    def inflate_grid(self, inflation_radius: int):
        inflation_radius_cells = inflation_radius  # Since each cell is 1 ft
        rows = len(self.node_grid)
        cols = len(self.node_grid[0]) if rows > 0 else 0

        to_inflate = set()

        for row in self.node_grid:
            for node in row:
                if node.value == 1:
                    x, y = node.idx, node.idy
                    for dx in range(
                        -inflation_radius_cells, inflation_radius_cells + 1
                    ):
                        for dy in range(
                            -inflation_radius_cells, inflation_radius_cells + 1
                        ):
                            if (
                                abs(dx) + abs(dy) <= inflation_radius_cells
                            ):  # Manhattan distance
                                nx, ny = x + dx, y + dy
                                if 0 <= nx < rows and 0 <= ny < cols:
                                    to_inflate.add((nx, ny))
        for idx, idy in to_inflate:
            self.node_grid[idx][idy].value = 1

    def find_path(self, start, end):
        obstacle = 1
        closed = []
        open = [start]
        heapq.heapify(open)
        while len(open) > 0:
            current_node = heapq.heappop(open)
            if current_node == end:
                return self.reconstruct_path(end)
            closed.append(current_node)
            children = self.search_neighbor(
                self.node_grid, current_node.idx, current_node.idy
            )
            for child in children:
                if child in closed or child.value == obstacle:
                    continue
                if child in open and child.g <= current_node.g:
                    continue
                child.g = current_node.g + math.hypot(
                    child.idx - current_node.idx, child.idy - current_node.idy
                )
                child.h = math.hypot(end.idx - child.idx, end.idy - child.idy)
                child.f = child.h + child.g
                child.parent = current_node  # new code
                if child not in open:
                    heapq.heappush(open, child)

        return None



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
        [1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1],
        [1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1],
        [1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1],
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



finder = HybridStar_Path_Follower(grid, robot_radius=1.5)
alessfinder = AStar_Path_Follower(grid)

path = finder.find_path(
    Node(1,0,math.pi/2, 0),
    Node(8,16,0, 0),
    v_max=0.1,
    ω_max=math.pi/8,
)

other_path = alessfinder.find_path(alessfinder.node_grid[1][0], alessfinder.node_grid[8][16])
 #default, v_max = 0.1, ω_max = math.pi/8
# path = calculate_optimal_path(Pose(8,7,math.pi), Pose(0,0,-math.pi/2), 15)

# path = calculate_optimal_path(Pose(4, 6, 0), Pose(8, 15, 0), 15)
node_grid = finder.node_grid
# trajectories = finder.generate_trajectories(Pose(0,0,0),0.1,math.pi/8)
# Define start and end nodes with orientation
# path = finder.find_path(Node(0, 0, 0, 0), Node(8,6,0,0), v_max=0.5, ω_max=0.1)
# x=[]
# y=[]
# for trajectory, velocity_sign in trajectories:
#     for pose in trajectory:
#         x.append(pose.x)
#         y.append(pose.y)
# plt.plot(x, y, marker='o', linestyle='None', color='blue', label='Points')  # No line, only points
# plt.xlabel("X Coordinate")
# plt.ylabel("Y Coordinate")
# plt.title("Point Visualization")
# plt.legend()
# plt.grid()
# plt.show()
x = []
y = []
colors = []

# Collect grid data for visualization
for row in node_grid:
    for node in row:
        x.append(node.idx)  # X-coordinate of the node
        y.append(node.idy)  # Y-coordinate of the node
        
        colors.append(
            "red" if node.value == 1 else "blue"
        )  # Color based on obstacle or free cell

class Chaikin_Smooth:
    def __init__(self, points: list[Point]):
        self.points = points

    def smooth_path(self, num_iterations: int):
        for _ in range(num_iterations):
            new_points = []
            for i in range(len(self.points) - 1):
                p0 = (self.points[i].point)
                p1 = (self.points[i + 1].point)
                q = (0.75 * p0[0] + 0.25 * p1[0], 0.75 * p0[1] + 0.25 * p1[1])
                r = (0.25 * p0[0] + 0.75 * p1[0], 0.25 * p0[1] + 0.75 * p1[1])
                q_point = Point(q[0], q[1])
                r_point = Point(r[0], r[1])
                new_points.append(q_point)
                new_points.append(r_point)
            new_points.insert(0, self.points[0])  # add first point
            new_points.append(self.points[-1])
            self.points = new_points
        return self.points

def transform_global_to_local(global_list: list[Point], local_origin: Point, theta: float) -> list[Point]:
    """Transform global coordinates to local coordinates."""
    local_points = []
    for global_point in global_list:
        dx = global_point.x - local_origin.x
        dy = global_point.y - local_origin.y
        x_local = dx * math.cos(theta) + dy * math.sin(theta)
        y_local = -dx * math.sin(theta) + dy * math.cos(theta)
        local_points.append(Point(x_local, y_local))
    return local_points
# Plot the grid
plt.figure(figsize=(10, 10))
plt.scatter(x, y, c=colors, s=10, label="Grid")  # Scatter plot for grid cells

path_x = []
path_y = []
path_smoothx = []
path_smoothy = []
searched_x = []
searched_y = []
path_transformx = []
path_transformy = []
simplified_x = []
simplified_y = []
path_smoothx_2 = []
path_smoothy_2 = []
if path:
    print(len(path))
# Plot the path if it exists
if path and other_path:
    chaikin = Chaikin_Smooth([Point (pose.x, pose.y) for pose in path])  # Convert Pose to Point
    smoothed_path_second =  Chaikin_Smooth(other_path).smooth_path(num_iterations=4)
    smoothed_path = chaikin.smooth_path(num_iterations=4)  # Smooth the path
    transformed_path = transform_global_to_local(smoothed_path, Point(1, 0), math.pi/2)  # Transform to local coordinates
    simplified_path = simplify_path([Point(pose.x, pose.y) for pose in smoothed_path], epsilon=0.05)
    for pose in path:
        path_x.append(pose.x) # X-coordinates of the path
        path_y.append(pose.y)  # Y-coordinates of the path
    for pose in smoothed_path:
        path_smoothx.append(pose.x)
        path_smoothy.append(pose.y)
    for pose in transformed_path:
        path_transformx.append(pose.x)
        path_transformy.append(pose.y)
    for pose in simplified_path:
        simplified_x.append(pose.x)
        simplified_y.append(pose.y)
    for point in smoothed_path_second:
        path_smoothx_2.append(point.x)
        path_smoothy_2.append(point.y)


for pose in searched_poses:
    searched_x.append(pose.x)  # X-coordinates of the searched poses
    searched_y.append(pose.y)  # Y-coordinates of the searched poses

plt.plot(path_x, path_y, linestyle="-", color="green", label="Path Points")
plt.plot(path_smoothx, path_smoothy, linestyle="--", color="purple", label="Hybrid A* Smoothed Path")
plt.plot(path_smoothx_2, path_smoothy_2, linestyle="--", color="orange", label="A* Smoothed Path")
# plt.plot(path_transformx, path_transformy, linestyle="-", color="blue", label="Transformed Path")
# plt.scatter(searched_x, searched_y, linestyle="-",color="orange", s=10, label="Searched Points")
# plt.scatter(simplified_x, simplified_y, linestyle="-",color="black", s=30, label="Simplified Path")
# plt.plot(simplified_x, simplified_y, linestyle="--", color="red", label="Smoothed Path")
plt.legend()

    # plt.scatter(
    #     path_x, path_y, color="green", s=30, label="Path Points"
    # )  # Highlight path points

# # # Add labels and grid
# plt.xlabel("X Coordinate")
# plt.ylabel("Y Coordinate")
# plt.title("Path in Grid Visualization")
# plt.legend()
# plt.grid()

# Show the plot
plt.show()
