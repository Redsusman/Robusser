import numpy as np
import matplotlib.pyplot as plt
import math
from matplotlib.animation import FuncAnimation


def _siftdown(heap, startpos, pos):
    newitem = heap[pos]
    while pos > startpos:
        parentpos = (pos - 1) >> 1
        parent = heap[parentpos]
        if newitem.f < parent.f:
            heap[pos] = parent
            pos = parentpos
            continue
        break
    heap[pos] = newitem


def _siftup(heap, pos):
    endpos = len(heap)
    startpos = pos
    newitem = heap[pos]
    childpos = 2 * pos + 1
    while childpos < endpos:
        rightpos = childpos + 1
        if rightpos < endpos and not heap[childpos].f < heap[rightpos].f:
            childpos = rightpos
        heap[pos] = heap[childpos]
        pos = childpos
        childpos = 2 * pos + 1
    heap[pos] = newitem
    _siftdown(heap, startpos, pos)


def heappush(heap, item):
    heap.append(item)
    _siftdown(heap, 0, len(heap) - 1)


def heappop(heap):
    lastelt = heap.pop()
    if heap:
        returnitem = heap[0]
        heap[0] = lastelt
        _siftup(heap, 0)
        return returnitem
    return lastelt


def heapify(heap):
    n = len(heap)
    for i in reversed(range(n // 2)):
        _siftup(heap, i)


class Point:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.point = (x, y)


def transform_global_to_local(
    global_list: list[Point], local_origin: Point, theta: float
) -> list[Point]:
    """Transform global coordinates to local coordinates."""
    local_points = []
    for global_point in global_list:
        dx = global_point.x - local_origin.x
        dy = global_point.y - local_origin.y
        x_local = dx * np.cos(theta) + dy * np.sin(theta)
        y_local = -dx * np.sin(theta) + dy * np.cos(theta)
        local_points.append(Point(x_local, y_local))
    return local_points


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
    def __init__(self, x, y, theta, track_width, wheel_base):
        self.pid_forward = PIDController(0.1, 0.0, 0.0)
        self.pid_omega = PIDController(5.0, 0.0, 0.0)
        self.x = x
        self.y = y
        self.theta = theta
        self.track_width = track_width
        self.wheel_base = wheel_base
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
        # print(self.x, self.y, math.degrees(self.theta))

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

    def pid_test_to_point(self, start_point: Point, end_point: Point, dt: float):
        distance_to_target = math.hypot(
            end_point.point[0] - self.x, end_point.point[1] - self.y
        )
        angle_to_target = math.atan2(
            end_point.point[1] - start_point.point[1],
            end_point.point[0] - start_point.point[0],
        )
        control_forward = self.pid_forward.compute(float(distance_to_target), 0, dt)
        control_omega = self.pid_omega.compute(self.theta, angle_to_target, dt)
        return -control_forward, control_omega, distance_to_target

    def pid_to_points(self, points: list[Point], dt: float):
        for i in range(len(points) - 1):
            forward, omega, _ = self.pid_test_to_point(points[i], points[i + 1], dt)
            vl, vr = self.inverse(forward, omega)
            self.forward(vl, vr, dt)


class Chaikin_Smooth:
    def __init__(self, points: list[Point]):
        self.points = points

    def smooth_path(self, num_iterations: int):
        for _ in range(num_iterations):
            new_points = []
            for i in range(len(self.points) - 1):
                p0 = self.points[i].point
                p1 = self.points[i + 1].point
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

    def compute_intersection_circle(self, path: list[Point], lookahead):
        robot_pose = (self.drive.x, self.drive.y)
        for i in range(len(path) - 1):
            first_point = path[i].point
            second_point = path[i + 1].point
            dx = second_point[0] - first_point[0]
            dy = second_point[1] - first_point[1]
            a = dx**2 + dy**2
            b = 2 * (
                dx * (first_point[0] - robot_pose[0])
                + dy * (first_point[1] - robot_pose[1])
            )
            c = (
                (first_point[0] - robot_pose[0]) ** 2
                + (first_point[1] - robot_pose[1]) ** 2
                - lookahead**2
            )
            discriminant = b**2 - 4 * a * c
            if discriminant < 0:
                continue
            t = (-b - math.sqrt(discriminant)) / (2 * a)
            if 0 <= t <= 1:
                intersection_x = first_point[0] + t * dx
                intersection_y = first_point[1] + t * dy
                return (intersection_x, intersection_y)

    # def calculate(self, path: list[Point]):
    #     self.lookahead_distance = (
    #         self.lookahead_distance
    #         + self.distance_parameter * self.drive.speed
    #         - self.curvature_parameter * self.previous_curvature
    #     )
    #     robot_pose = (self.drive.x, self.drive.y)
    #     theta = self.drive.theta
    #     lookahead_point = self.compute_intersection_circle(path, self.lookahead_distance)
    #     if lookahead_point is None:
    #         return 0.0
    #     dx = lookahead_point[0] - robot_pose[0]
    #     dy = lookahead_point[1] - robot_pose[1]
    #     alpha = math.atan2(dy, dx) - theta
    #     alpha = math.atan2(math.sin(alpha), math.cos(alpha))
    #     dist = math.sqrt(dx**2 + dy**2)
    #     if abs(dist) < 1e-10:
    #         curvature = 0.0
    #     else:
    #         curvature = (2.0 * math.sin(alpha)) / dist
    #     return curvature

    # def calculate(self, path: list[Point]):
    #     self.lookahead_distance = (
    #         self.lookahead_distance
    #         + self.distance_parameter * self.drive.speed
    #         - self.curvature_parameter * self.previous_curvature
    #     )
    #     robot_pose = (self.drive.x, self.drive.y)
    #     path_points = [(point.point[0], point.point[1]) for point in path]
    #     theta = self.drive.theta
    #     distances = [
    #         math.sqrt((p[0] - robot_pose[0]) ** 2 + (p[1] - robot_pose[1]) ** 2)
    #         for p in path_points
    #     ]
    #     closest_idx = min(range(len(distances)), key=lambda i: distances[i])
    #     lookahead_point = path_points[-1]
    #     for i in range(closest_idx, len(path_points)):
    #         p = path_points[i]
    #         dist = math.sqrt((p[0] - robot_pose[0]) ** 2 + (p[1] - robot_pose[1]) ** 2)
    #         if dist >= self.lookahead_distance:
    #             lookahead_point = p
    #             break
    #     dx = lookahead_point[0] - robot_pose[0]
    #     dy = lookahead_point[1] - robot_pose[1]
    #     alpha = math.atan2(dy, dx) - theta
    #     dist = math.sqrt(dx**2 + dy**2)
    #     if abs(dist) < 1e-10:
    #         curvature = 0.0
    #     else:
    #         curvature = (2.0 * math.sin(alpha)) / dist
    #     self.previous_curvature = curvature
    #     return curvature
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
        for t in range(3000):
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


class Stanley_Controller:
    def __init__(self, drive: Drive, k: float, vel, lookahead):
        self.drive = drive
        self.k = k
        self.vel = vel
        self.lookahead = lookahead
        self.speed_controller = PIDController(2.0, 0.0, 0)
        self.cte_controller = PIDController(2.0, 0, 0)
        self.theta_controller = PIDController(1, 0, 0)
        pass

    def calculate(self, path: list[Point]):
        robot_pose = (self.drive.x, self.drive.y)
        theta = self.drive.theta
        distances = [
            math.hypot(p.point[0] - robot_pose[0], p.point[1] - robot_pose[1])
            for p in path
        ]
        closest_idx = distances.index(min(distances))
        shortest_point = path[closest_idx]
        next_point = (
            path[closest_idx + 1] if closest_idx + 1 < len(path) else shortest_point
        )
        dx = next_point.point[0] - shortest_point.point[0]
        dy = next_point.point[1] - shortest_point.point[1]
        path_heading = math.atan2(dy, dx)
        heading_error = math.atan2(
            math.sin(path_heading - theta), math.cos(path_heading - theta)
        )
        rx = robot_pose[0] - shortest_point.point[0]
        ry = robot_pose[1] - shortest_point.point[1]
        cross_track_error = rx * math.sin(path_heading) - ry * math.cos(path_heading)
        v = max(self.drive.speed, 0.1)
        steering_angle = heading_error + math.atan((self.k * cross_track_error) / v)
        steering_radius = self.drive.wheel_base / (math.tan(steering_angle) + 1e-6)

        return steering_radius

    def calculate_feedback(self, path: list[Point], heading_kP, cte_kP, dist_kP):
        robot_pose = (self.drive.x, self.drive.y)
        theta = self.drive.theta
        distances = [
            math.hypot(p.point[0] - robot_pose[0], p.point[1] - robot_pose[1])
            for p in path
        ]
        closest_idx = distances.index(min(distances))
        shortest_point = path[closest_idx]
        next_point = (
            path[closest_idx + 1] if closest_idx + 1 < len(path) else shortest_point
        )
        dx = next_point.point[0] - shortest_point.point[0]
        dy = next_point.point[1] - shortest_point.point[1]
        path_heading = math.atan2(dy, dx)
        heading_error = path_heading - theta
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        rx = robot_pose[0] - shortest_point.point[0]
        ry = robot_pose[1] - shortest_point.point[1]
        cross_track_error = rx * math.sin(path_heading) - ry * math.cos(path_heading)
        # v = max(self.drive.speed, 0.1)
        # steering_angle = heading_error + math.atan((self.k * cross_track_error) / v)
        # steering_radius = self.drive.wheel_base / (math.tan(steering_angle) + 1e-6)
        dist_error = math.hypot(
            shortest_point.point[0] - robot_pose[0],
            shortest_point.point[1] - robot_pose[1],
        )

        heading_correction = heading_kP * heading_error
        cte_correction = cte_kP * cross_track_error
        dist_correction = dist_kP * dist_error

        print(cte_correction, dist_correction)
        # heading_correction = -self.theta_controller.compute(heading_error, 0,0.01)
        # cte_correction = -self.cte_controller.compute(heading_error, 0,0.01)
        # dist_correction = self.speed_controller.compute(heading_error, 0,0.01)

        return dist_correction, cte_correction + heading_correction

    def follow_path_feedback(self, path: list[Point], heading_kP, cte_kP, dist_kP):
        dt = 0.1
        for t in range(1000):
            dist_correction, omega = self.calculate_feedback(
                path, heading_kP, cte_kP, dist_kP
            )
            v = 0.5 + dist_correction
            vl, vr = self.drive.inverse(v, omega)
            # print(v,omega)
            self.drive.forward(vl, vr, dt)
            if (
                np.linalg.norm(
                    np.array([self.drive.x, self.drive.y]) - np.array(path[-1].point)
                )
                < 0.5
            ):
                break

    def follow_path(self, path: list[Point]):
        dt = 0.1
        for t in range(1000):
            radius = self.calculate(path)
            # scaled_velocity = self.vel * np.clip(
            #     1 / (abs(curvature) + 1e-6), 0.5, 1
            # )
            vl, vr = self.drive.inverse(self.vel, self.vel / (radius + 1e-7))
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
        heapify(open)
        while len(open) > 0:
            current_node = heappop(open)
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
                    heappush(open, child)

        return None

    def find_path_dstar_lite(self):
        pass


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


# Ramer-Douglas-Peucker Algorithm
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
        left = simplify_path(points[: index + 1], epsilon)
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
        return math.hypot(x - x1, y - y1)

    numerator = abs((x2 - x1) * (y1 - y) - (x1 - x) * (y2 - y1))
    denominator = math.hypot(x2 - x1, y2 - y1)
    return numerator / denominator


class USB_Server_Communicator:
    def __init__(self):
        self.port = None
        self.baudrate = None
        self.serialPort = None

    def connect(self):
        pass

    def send_data(self, args):
        data_to_send = b"example_data"  # Replace with the actual data you want to send
        # self.serialPort.write(data_to_send)

    def recieve_data(self):
        pass

    def close(self):
        pass


class Elevator:
    def __init__(self):
        self = None


class Potential_Field_Method:
    def __init__(self):
        pass

    def calculate_potential(self, path, obstacles):
        pass


# import keyboard

# drive = Drive(0, 0, 1.57, 10)
# path = [Point(0, 0), Point(0,3), Point(3,3), Point(10,3), Point(10,7)] #ft
# smooth = Chaikin_Smooth(path)
# smoothed_path = smooth.smooth_path(4)

# points = [Point(0,0), Point(1,1), Point(2,1), Point(3,2), Point(3,3), Point(4,3), Point(5,4), Point(5,5), Point(5,6), Point(6,5), Point(7,6), Point(7,7), Point(7,8), Point(8,9), Point(9,9)]

# for _ in range(1000):
#     drive.pid_to_points(points, 0.01)

# x = [x for x in drive.x_list]
# y= [y for y in drive.y_list]
# plt.plot(x,y,label="Robot Path")
# plt.show()
# Main loop to control the vehicle and update the plot
# print("Use 'W', 'A', 'S', 'D' to move the vehicle. Press 'Space' to stop.")

# Print the current position and orientation
# print(f"x: {drive.x:.2f}, y: {drive.y:.2f}, theta: {math.degrees(drive.theta):.2f}")

# start_point = Point(0,0)
# end_point = Point(12,0)

# for _ in range(10000):
#     forward, omega, distance_to_target = drive.pid_test_to_point(start_point, end_point, 0.01)
#     # print(distance_to_target)
#     if(distance_to_target >= 2):
#         vl, vr = drive.inverse(forward, omega)
#         drive.forward(vl,vr,0.01)
#     else:
#         break


# x = [x for x in drive.x_list]
# y = [y for y in drive.y_list]

# x_path = [start_point.point[0], end_point.point[0]]
# y_path = [start_point.point[1], end_point.point[1]]

# plt.plot(x, y, label="Robot Path")
# plt.plot(x_path, y_path, label="Path")
# plt.show(block=True)

# usb = USB_Server_Communicator()

# usb.send_data("hello")
# # Initialize the A* path follower
# # # (0, 0)(1, 1)(2, 1)(3, 2)(3, 3)(4, 3)(5, 4)(5, 5)(5, 6)(6, 5)(7, 6)(7, 7)(7, 8)(8, 9)(9, 9)
# # row = 10
# col = 10

grid = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
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
    [1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
]
astar = AStar_Path_Follower(grid)
astar_two = AStar_Path_Follower(grid)
astar.inflate_grid(0)
# dwa = DWA_Controller(1.0, 0.5, math.pi, 0.05, 2, 0.1, 0.1, 0.1, 0.2)

# # Define start and end nodes
start = astar.node_grid[1][0]  # Top-left corner
end = astar.node_grid[28][16]  # Bottom-right corner

# Find the path
path = astar.find_path(start, end)
if path is not None:
    simplified_path = simplify_path(path, 0.5)
else:
    simplified_path = []
# plot grid:

node_grid = astar_two.node_grid

drive = Drive(0, 0, 0, 2, 3)
drive_2 = Drive(0, 0, 0, 2, 3)
# transformed_path = transform_global_to_local(simplified_path, Point(1,0), math.pi/2)
# smooth = Chaikin_Smooth(simplified_path)

# # path = [Point(0,0), Point(1,0), Point(4,-4)]
# if transformed_path:
#     x = [point.x for point in transformed_path]
#     y = [point.y for point in transformed_path]
#     x1 = [point.point[0] for point in simplified_path]
#     y1 = [point.point[1] for point in simplified_path]
#     smoothed_path = smooth.smooth_path(4)
#     x2= [point.point[0] for point in smoothed_path]
#     y2= [point.point[1] for point in smoothed_path]
#     plt.plot(x, y, "", label="A* Path")
#     plt.plot(x1, y1, "", label="Simplified Path")
#     plt.plot(x2, y2, "", label="Smoothed Path")
# else:
#     print("No path found.")


# # drive = Drive(0, 0, 0, 10)
dt = 0.01
# # left_speed = 0
# # right_speed = 0

# control = Stanley_Controller(
#     drive=drive, k=1.5, vel=0.1, lookahead = 0.0
# )
control = Stanley_Controller(drive, 5, 0.5, 0)
pure = Pure_Pursuit_Controller(
    drive_2, PIDController(1, 0, 0), PIDController(1, 0, 0), 1.0, 0.4, 0, 0
)

if path is not None:
    smoother = Chaikin_Smooth(path)
    smoothed_path = smoother.smooth_path(4)
    control.follow_path_feedback(smoothed_path[::-1], 2.0, 2.0, 0.5)
    pure.follow_path(smoothed_path)
# else:
#     print("No valid path found. Cannot proceed with smoothing or following.")
# plt.plot(
#     [x.point[0] for x in smoothed_path],
#     [x.point[1] for x in smoothed_path],
#     label="True Path",
# )

# # Generate grid visualization
plt.figure(figsize=(10, 8))

# Plot the occupancy grid
x_grid = [node.idx for row in node_grid for node in row]
y_grid = [node.idy for row in node_grid for node in row]
colors = ["red" if node.value == 1 else "blue" for row in node_grid for node in row]
plt.scatter(x_grid, y_grid, c=colors, s=10, alpha=0.5, label="Occupancy Grid")

# Plot the planned path
if path is not None:
    smoother = Chaikin_Smooth(path)
    smoothed_path = smoother.smooth_path(4)

    # Reference path
    # plt.plot(
    #     [p.point[0] for p in smoothed_path],
    #     [p.point[1] for p in smoothed_path],
    #     '--',
    #     linewidth=2,
    #     label="Smoothed Path (Chaikin)",
    # )

    # # Robot trajectory
    # plt.plot(
    #     drive.x_list,
    #     drive.y_list,
    #     "r-",
    #     linewidth=1.5,
    #     label="Robot Path (Stanley Controller)",
    # )

    plt.plot(
        drive_2.x_list,
        drive_2.y_list,
        "g-",
        linewidth=1.5,
        label="Robot Path (Pure Pursuit)",
    )
else:
    print("No valid path found")





# Formatting
plt.xlabel("X Coordinate (meters)", fontsize=12)
plt.ylabel("Y Coordinate (meters)", fontsize=12)
plt.title("AGV Path Tracking\n(Stanley Controller + Chaikin Smoothing)", fontsize=14)
plt.legend(loc="upper right")
plt.grid(True, alpha=0.3)
plt.axis("equal")
plt.tight_layout()
plt.show()
# To save: anim.save('path_tracking.mp4', writer='ffmpeg', fps=20)
