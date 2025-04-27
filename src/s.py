import numpy as np
import matplotlib.pyplot as plt
import keyboard
import time
import math
import heapq
import cv2


class Point:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.point = (x, y)


class PIDController:
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0
        self.error = 0

    def compute(self, measured_value: float, setpoint: float, dt: float) -> float:
        error = setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.previous_error = error
        return output

    def get_error(self):
        return self.previous_error


class Drive:
    def __init__(self, x, y, theta, track_width):
        self.x = x
        self.y = y
        self.theta = theta
        self.track_width = track_width
        self.speed = 0
        self.omega = 0
        self.x_list = []
        self.y_list = []
        self.theta_list = []

    def forward(self, left_speed: float, right_speed: float, dt: float):
        speed = (right_speed + left_speed) / 2
        omega = (right_speed - left_speed) / self.track_width
        self.theta += omega * dt
        self.x += speed * np.cos(self.theta) * dt
        self.y += speed * np.sin(self.theta) * dt
        self.speed = speed
        self.omega = omega
        self.x_list.append(self.x)
        self.y_list.append(self.y)
        self.theta_list.append(self.theta)

    def inverse(self, forward: float, omega: float) -> tuple[float, float]:
        vl = forward - ((omega * self.track_width) / 2)
        vr = forward + ((omega * self.track_width) / 2)
        return vl, vr

    def reset_pose(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.x_list = [x]
        self.y_list = [y]
        self.theta_list = [theta]


class Chaikin_Smooth:
    def __init__(self, points: list[Point]):
        self.points = points

    def smooth_path(self, num_iterations: int):
        for _ in range(num_iterations):
            new_points = []
            for i in range(len(self.points) - 1):
                p0 = np.array(self.points[i].point)
                p1 = np.array(self.points[i + 1].point)
                q = tuple(0.75 * p0 + 0.25 * p1)
                r = tuple(0.25 * p0 + 0.75 * p1)
                q_point = Point(q[0], q[1])
                r_point = Point(r[0], r[1])
                new_points.append(q_point)
                new_points.append(r_point)
            new_points.insert(0, self.points[0])  # add first point
            new_points.append(self.points[-1])
            self.points = new_points
        return self.points


class Pure_Pursuit_Controller:
    def __init__(
        self,
        drive: Drive,
        forward_controller: PIDController,
        omega_controller: PIDController,
        lookahead_distance: float,
        forward_velocity: float,
        distance_parameter: float,
        curvature_parameter: float,
    ):
        self.forward_controller = forward_controller
        self.omega_controller = omega_controller
        self.drive = drive
        self.lookahead_distance = lookahead_distance
        self.forward_velocity = forward_velocity
        self.distance_parameter = distance_parameter
        self.curvature_parameter = curvature_parameter
        self.previous_curvature = 0

    def calculate(self, path: list[Point]):
        self.lookahead_distance = (
            self.lookahead_distance
            + self.distance_parameter * self.drive.speed
            - self.curvature_parameter * self.previous_curvature
        )
        robot_pose = np.array([self.drive.x, self.drive.y])
        path_to_array = np.array([point.point for point in path])
        theta = self.drive.theta
        distances = np.linalg.norm(path_to_array - robot_pose, axis=1)
        closest_idx = np.argmin(distances)
        for i in range(closest_idx, len(path_to_array)):
            dist = np.linalg.norm(path_to_array[i] - robot_pose)
            if dist >= self.lookahead_distance:
                lookahead_point = path_to_array[i]
                break
        else:
            lookahead_point = path_to_array[-1]

        dx, dy = lookahead_point - robot_pose
        alpha = np.arctan2(dy, dx) - theta
        dist = np.linalg.norm(lookahead_point - robot_pose)
        curvature = (2 * np.sin(alpha)) / dist
        self.previous_curvature = curvature
        return curvature

    def follow_path(self, path: list[Point]):
        dt = 0.1
        for t in range(1000):
            curvature = self.calculate(path)
            scaled_velocity = self.forward_velocity * np.clip(
                1 / (abs(curvature) + 1e-6), 0.5, 1
            )
            vl, vr = self.drive.inverse(scaled_velocity, scaled_velocity * curvature)
            self.drive.forward(vl, vr, dt)
            if (
                np.linalg.norm(
                    np.array([self.drive.x, self.drive.y]) - np.array(path[-1].point)
                )
                < 0.1
            ):
                break


class Node:
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


class AStar_Path_Follower:
    def __init__(self, map_grid):
        self.map_grid = map_grid
        self.node_grid = [
            [
                Node(self.map_grid[row][col], row, col)
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


class DWA_Controller:
    def __init__(
        self,
        max_vel: float,
        max_accel: float,
        max_steer: float,
        dt: float,
        total_search_time,
        alpha,
        beta,
        gamma,
        robot_radius,
    ):
        self.max_vel = max_vel
        self.max_accel = max_accel
        self.max_steer = max_steer
        self.dt = dt
        self.total_search_time = total_search_time
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.robot_radius = robot_radius
        self.min_vel = 0.01
        self.min_omega = 0.01

    def find_best_velocity(
        self,
        current_vel,
        current_omega,
        target_point,
        obstacles,
        search_step,
        current_x,
        current_y,
        current_theta,
    ):
        velocity_window = np.arange(
            current_vel - self.max_accel * self.dt,
            current_vel + self.max_accel * self.dt,
            search_step,
        )
        omega_window = np.arange(
            current_omega - self.max_steer * self.dt,
            current_omega + self.max_steer * self.dt,
            search_step,
        )

        best_score = -float("inf")
        best_v, best_omega = 0.0, 0.0

        for v in velocity_window:
            for omega in omega_window:
                x, y, theta = current_x, current_y, current_theta
                min_obstacle_dist = float("inf")
                collided = False
                for t in np.arange(0, self.total_search_time, self.dt):
                    x += v * np.cos(theta) * self.dt
                    y += v * np.sin(theta) * self.dt
                    theta += omega * self.dt
                    for obs in obstacles:
                        dist = np.linalg.norm([x - obs[0], y - obs[1]])
                        min_obstacle_dist = min(min_obstacle_dist, dist)
                        if dist < self.robot_radius:
                            collided = True
                            break
                    if collided:
                        break
                if collided:
                    continue
                admissable_vel = math.sqrt(2 * self.max_accel * min_obstacle_dist)
                if abs(v) > admissable_vel:
                    continue
                heading_error = (
                    np.arctan2(target_point[1] - y, target_point[0] - x) - theta
                )
                heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
                score = (
                    self.alpha * heading_error
                    + self.beta * min_obstacle_dist
                    + self.gamma * abs(v)
                )
                if score > best_score:
                    best_score = score
                    best_v, best_omega = v, omega

        return best_v, best_omega, best_score

    def robot_coords_to_grid(self, x, y, grid):
        grid_x = int(x / grid.cell_size)
        grid_y = int(y / grid.cell_size)
        return grid_x, grid_y


class USB_Server_Communicator:
    def __init__(self):
        self.port = None
        self.baudrate = None
        self.serial = None

    def connect(self):
        pass

    def send_data(self, args):
        pass

    def recieve_data(self):
        pass

    def close(self):
        pass


class Elevator:
    def __init__(self):
        self = None


def generate_gridmap(rows, cols):
    grid = np.random.randint(0, 2, size=(rows, cols))
    grid[0][0] = 0
    grid[rows - 1][cols - 1] = 0
    return grid


# Initialize the A* path follower
# (0, 0)(1, 1)(2, 1)(3, 2)(3, 3)(4, 3)(5, 4)(5, 5)(5, 6)(6, 5)(7, 6)(7, 7)(7, 8)(8, 9)(9, 9)
row = 10
col = 10
grid = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 1, 1, 1, 0, 0, 0, 0],
    [0, 0, 1, 1, 1, 1, 0, 0, 0, 0],
    [0, 0, 1, 1, 0, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 1, 1, 1, 0],
    [0, 1, 1, 1, 1, 0, 1, 1, 1, 0],
    [0, 1, 1, 1, 1, 0, 1, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]

astar = AStar_Path_Follower(grid)

# Define start and end nodes
start = astar.node_grid[0][7]  # Top-left corner
end = astar.node_grid[4][4]  # Bottom-right corner

# Find the path
path = astar.find_path(start, end)
# plot grid:
node_grid = astar.node_grid
colors = ["red" if node.value == 1 else "blue" for row in node_grid for node in row]

# Extract x and y coordinates
x_grid = [node.idx for row in node_grid for node in row]
y_grid = [node.idy for row in node_grid for node in row]

# Plot the grid nodes with colors
plt.scatter(x_grid, y_grid, c=colors, label="Grid Nodes", alpha=0.5)
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()

if path:
    x = [point.x for point in path]
    y = [point.y for point in path]
    plt.plot(x, y, "", label="A* Path")
else:
    print("No path found.")


# plt.ion()
drive = Drive(0, 7, 0, 10)
dt = 0.1
left_speed = 0
right_speed = 0

control = Pure_Pursuit_Controller(
    drive, PIDController(1, 0, 0), PIDController(1, 0, 0), 0.5, 1, 0, 0
)
if path is not None:
    smoother = Chaikin_Smooth(path)
    smoothed_path = smoother.smooth_path(4)
    control.follow_path(smoothed_path)
else:
    print("No valid path found. Cannot proceed with smoothing or following.")
plt.plot(
    [x.point[0] for x in smoothed_path],
    [x.point[1] for x in smoothed_path],
    label="True Path",
)
plt.plot(drive.x_list, drive.y_list, label="Robot Path")
plt.legend()
# plt.show(block=True)

plt.show(block=True)
