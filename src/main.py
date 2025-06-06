from vex import *
import math


class Elevator:
    def __init__(self, brain: Brain):
        self.pneumatic = Pneumatics(brain.three_wire_port.g)

    def up(self):
        self.pneumatic.close()

    def down(self):
        self.pneumatic.open()


def linspace(start, stop, num):
    if num <= 0:
        return []
    if num == 1:
        return [start]
    step = (stop - start) / (num - 1)
    return [start + step * i for i in range(num)]


def sign(x):
    """Return the sign of x."""
    return (x > 0) - (x < 0)


def hypot(x, y):
    return math.sqrt(x**2 + y**2)


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
        x_local = dx * math.cos(theta) + dy * math.sin(theta)
        y_local = -dx * math.sin(theta) + dy * math.cos(theta)
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


class Distance_Sensor:
    def __init__(self):
        self.recorded = 0.0
        self.sensor = Distance(Ports.PORT5)

    def get_distance(self) -> float:
        return self.sensor.object_distance(DistanceUnits.IN)

    def continous_distance(self):
        while True:
            self.recorded_distance = self.get_distance()


class Drive:
    def __init__(
        self,
        x: float,
        y: float,
        theta: float,
        left_motor_port: int,
        right_motor_port: int,
        imu_port: int,
    ):
        self.track_width = 16.625 / 12
        self.wheel_base = 16.125 / 12
        self.wheel_diameter = 4 / 12
        self.wheel_radius = self.wheel_diameter / 2
        self.left_motor = Motor(left_motor_port)
        self.right_motor = Motor(right_motor_port)
        self.right_motor.set_reversed(True)
        self.imu = Inertial(imu_port)
        self.imu.calibrate()
        self.imu.set_turn_type(TurnType.LEFT)
        self.x = x
        self.y = y
        self.theta = 0.0
        self.speed = 0
        self.omega = 0
        self.previous_distance = 0
        self.previous_theta = 0
        self.x_list = []
        self.y_list = []
        self.theta_list = []
        self.theta_controller = PIDController(0.01, 0.01, 0.0)

    def forward(self, left_speed: float, right_speed: float):
        speed = (right_speed + left_speed) / 2
        omega = (right_speed - left_speed) / self.track_width
        return speed, omega

    def odometer(
        self,
        left_encoder_rev: float,
        right_encoder_rev: float,
        gyro_angle: float,
    ):
        left_distance = left_encoder_rev * 2 * math.pi * self.wheel_radius
        right_distance = right_encoder_rev * 2 * math.pi * self.wheel_radius
        distance = (left_distance + right_distance) / 2
        delta_distance = distance - self.previous_distance
        self.theta = gyro_angle
        self.x += delta_distance * math.cos(gyro_angle)
        self.y += delta_distance * math.sin(gyro_angle)

        self.previous_distance = distance

        self.speed = (
            self.left_motor.velocity(RPM) + self.right_motor.velocity(RPM)
        ) / 2
        left_speed_fts = (
            (self.left_motor.velocity(RPM) * math.pi * self.wheel_diameter) / 60
        ) / 12
        right_speed_fts = (
            (self.right_motor.velocity(RPM) * math.pi * self.wheel_diameter) / 60
        ) / 12
        self.speed = (left_speed_fts + right_speed_fts) / 2

    def update_pose(self):
        while True:
            self.odometer(
                self.left_motor.position(RotationUnits.REV),
                self.right_motor.position(RotationUnits.REV),
                math.radians(self.imu.heading()),
            )

    def update_pose_static(self):
        self.odometer(
            self.left_motor.position(RotationUnits.REV),
            self.right_motor.position(RotationUnits.REV),
            math.radians(self.imu.heading()),
        )

    def inverse(self, forward: float, omega: float) -> tuple[float, float]:
        vl = forward - ((omega * self.track_width) / 2)
        vr = forward + ((omega * self.track_width) / 2)
        return vl, vr

    def rpm_to_rads(self, rpm: float) -> float:
        return (rpm * 2 * math.pi) / 60

    def reset_pose(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.x_list = [x]
        self.y_list = [y]
        self.theta_list = [theta]

    def stop_drive(self):
        self.left_motor.stop()
        self.right_motor.stop()

    def drive(self, forward, omega):
        vl, vr = self.inverse(forward, omega)
        vl_rpm = (vl * 60) / (2 * math.pi * self.wheel_radius)
        vr_rpm = (vr * 60) / (2 * math.pi * self.wheel_radius)
        self.left_motor.spin(FORWARD, vl_rpm, RPM)
        self.right_motor.spin(FORWARD, vr_rpm, RPM)

    def turn_to_angle(self, angle_rad: float):
        while True:
            current_angle = math.radians(self.imu.heading())
            angle_diff = angle_rad - current_angle
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
            if abs(angle_diff) < math.radians(2):
                self.stop_drive()
                break

            omega = self.theta_controller.compute(current_angle, angle_rad, 0.1)
            self.drive(0, omega)
            wait(10, MSEC)


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
            new_points.insert(0, self.points[0])
            new_points.append(self.points[-1])
            self.points = new_points
        return self.points


class Stanley_Controller:
    def __init__(self, drive: Drive, k: float, vel, lookahead):
        self.drive = drive
        self.k = k
        self.vel = vel
        self.lookahead = lookahead

    def calculate_feedback(self, path: list[Point], heading_kP, cte_kP, dist_kP):
        robot_pose = (self.drive.x, self.drive.y)
        theta = self.drive.theta
        distances = [
            hypot(p.point[0] - robot_pose[0], p.point[1] - robot_pose[1]) for p in path
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
        dist_error = hypot(
            shortest_point.point[0] - robot_pose[0],
            shortest_point.point[1] - robot_pose[1],
        )

        heading_correction = heading_kP * heading_error
        cte_correction = cte_kP * cross_track_error
        dist_correction = dist_kP * dist_error
        return dist_correction, cte_correction + heading_correction

    def follow_path_feedback(
        self, path: list[Point], heading_kP, cte_kP, dist_kP, distance_sensor: Distance
    ):
        bool_drive = True
        while bool_drive:
            object_distance = distance_sensor.object_distance(DistanceUnits.IN)
            if object_distance >= 20:
                dist_correction, omega = self.calculate_feedback(
                    path, heading_kP, cte_kP, dist_kP
                )
                v = self.vel + dist_correction
                self.drive.drive(v, omega)
                dx = self.drive.x - path[-1].point[0]
                dy = self.drive.y - path[-1].point[1]
                distance_to_end = math.sqrt(dx**2 + dy**2)
            else:
                drive.stop_drive()
            print(object_distance)

            if distance_to_end < 0.2:
                bool_drive = False
                self.drive.stop_drive()
                break


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
        self.path_from_parent: list[Pose] = []
        self.pose = Pose(self.x, self.y, self.theta)
        self.visited = False

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

    dist = hypot(p1[0] - p0[0], p1[1] - p0[1])
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
        points.append(Pose(position_x, position_y, 0.0))
    return points


class AStar_Path_Follower:
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
        grid_x = int(math.floor(x))
        grid_y = int(math.floor(y))
        if (
            grid_x < 0
            or grid_x >= len(self.node_grid)
            or grid_y < 0
            or grid_y >= len(self.node_grid[0])
        ):
            return True
        if self.node_grid[grid_x][grid_y].value == 1:
            return True

        if self.robot_radius > 0:
            radius_cells = math.ceil(self.robot_radius)
            radius_sq = self.robot_radius**2

            for dx in range(-radius_cells, radius_cells + 1):
                for dy in range(-radius_cells, radius_cells + 1):
                    if dx == 0 and dy == 0:
                        continue
                    if (dx**2 + dy**2) > (radius_cells + 0.5) ** 2:
                        continue

                    nx, ny = grid_x + dx, grid_y + dy
                    if 0 <= nx < len(self.node_grid) and 0 <= ny < len(
                        self.node_grid[0]
                    ):
                        if self.node_grid[nx][ny].value == 1:
                            cell_center_x = nx + 0.5
                            cell_center_y = ny + 0.5
                            distance_sq = (x - cell_center_x) ** 2 + (
                                y - cell_center_y
                            ) ** 2
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
            path.append(Pose(current.x, current.y, current.theta))
            current = current.parent
        path.reverse()
        return path

    def heuristic(self, a, b):
        dx = b.x - a.x
        dy = b.y - a.y
        dist = hypot(dx, dy)
        return dist

    def calculate_trajectory(self, v, omega, max_time, steps, initial_pose):
        poses = []
        time = linspace(0, max_time, steps)
        x, y, theta = initial_pose.x, initial_pose.y, initial_pose.theta
        for t in time:
            theta += omega * t
            theta = (theta + math.pi) % (2 * math.pi) - math.pi
            x += v * math.cos(theta)
            y += v * math.sin(theta)
            poses.append(Pose(x, y, theta))
        return poses

    def is_trajectory_collision(self, trajectory):
        for i in range(len(trajectory)):
            pose = trajectory[i]
            if self.is_collision(pose.x, pose.y):
                return True
            if i > 0:
                prev_pose = trajectory[i - 1]
                steps = max(abs(pose.x - prev_pose.x), abs(pose.y - prev_pose.y)) * 2
                for t in linspace(0, 1, int(steps)):
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
            dist += hypot(dx, dy)
        return dist

    def is_goal_reached(self, node, end, pos_tolerance=0.3, angle_tolerance=0.1):
        dist = hypot(node.x - end.x, node.y - end.y)
        pos_close = dist <= pos_tolerance
        angle_diff = abs((node.theta - end.theta + math.pi) % (2 * math.pi) - math.pi)
        return pos_close and (angle_diff <= angle_tolerance)

    def angle_cost(self, pose_a, pose_b):
        pose_a_theta = pose_a.theta
        pose_b_theta = pose_b.theta
        angle_diff = abs(
            (pose_a_theta - pose_b_theta + math.pi) % (2 * math.pi) - math.pi
        )
        return angle_diff

    def generate_trajectories(self, initial_pose, v_max, ω_max):
        trajectories = []
        motions = [
            (v_max, 0.0),
            (v_max, ω_max),
            (v_max, -ω_max),
            (v_max, ω_max / 2),
            (v_max, -ω_max / 2),
            (-v_max, 0.0),
            (-v_max, ω_max),
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
        heapify(open)
        closed = set()
        while open:
            current = heappop(open)
            optimal_path = calculate_optimal_path(current.pose, end.pose, 15)
            if not self.is_trajectory_collision(
                optimal_path
            ) and not self.is_outside_grid(optimal_path[-1].x, optimal_path[-1].y):
                path_to_current = self.reconstruct_path(current)
                return path_to_current + optimal_path[1:]
            if self.is_goal_reached(current, end):
                return self.reconstruct_path(current)
            closed.add(current)
            trajectories = self.generate_trajectories(current.pose, v_max, ω_max)
            for trajectory, velocity_sign in trajectories:
                if self.is_trajectory_collision(trajectory):
                    continue
                end_pose = trajectory[-1]
                if self.is_outside_grid(end_pose.x, end_pose.y):
                    continue
                child_node = Node(end_pose.x, end_pose.y, end_pose.theta, 0)
                if child_node in closed:
                    continue
                cost = self.calculate_cost(
                    trajectory, current, child_node, velocity_sign
                )
                child_node.g = current.g + cost
                child_node.h = self.heuristic(child_node.pose, end)
                child_node.f = child_node.g + child_node.h
                child_node.parent = current

                heappush(open, child_node)
            print(current.x, current.y, current.theta)
        print("hello")
        return None

    def quantize(self, node: Node, resolution):
        return (
            round(node.x / resolution) * resolution,
            round(node.y / resolution) * resolution,
            round(node.theta / resolution) * resolution,
        )


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
    [1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
]


finder = AStar_Path_Follower(grid, robot_radius=1.5)

path = finder.find_path(
    Node(1, 0, math.pi / 2, 0),
    Node(8, 7, 0 - 0.3, 0),
    v_max=0.1,
    ω_max=math.pi / 8,
)

node_grid = finder.node_grid


brain = Brain()
controller = Controller()
drive = Drive(0, 0, 0, Ports.PORT1, Ports.PORT4, Ports.PORT3)
control = Stanley_Controller(drive, 2.0, 0.3, 0.0)
elevator = Elevator(brain)
distance_sensor = Distance_Sensor()


if path is not None:
    smoothed_path = Chaikin_Smooth(
        [Point(pose.x, pose.y) for pose in path]
    ).smooth_path(4)
    transformed_path = transform_global_to_local(
        smoothed_path, Point(1, 0), math.pi / 2
    )

else:
    print("Pathfinding failed. No valid path found.")


def run():
    control.follow_path_feedback(
        transformed_path, 2.0, 2.0, 0.0, distance_sensor.sensor
    )
    elevator.down()


while drive.imu.is_calibrating():
    wait(100, MSEC)

stanley_thread = Thread(lambda: run())
odometry_thread = Thread(drive.update_pose())
