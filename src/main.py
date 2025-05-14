from vex import *
import math


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


class Distance_Sensor:
    def __init__(self):
        self.sensor = Distance(Ports.PORT5)

    def get_distance(self) -> float:
        return self.sensor.object_distance(INCHES)


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
        self.pid_forward = PIDController(0.2, 0.0, 0.0)
        self.pid_omega = PIDController(0.1, 0.0, 0.0)
        self.track_width = 16.625
        self.wheel_base = 16.125
        self.wheel_diameter = 4
        self.wheel_radius = self.wheel_diameter / 2
        self.x = x
        self.y = y
        self.theta = theta
        self.speed = 0
        self.omega = 0
        self.previous_distance = 0
        self.previous_theta = 0
        self.x_list = []
        self.y_list = []
        self.theta_list = []
        self.left_motor = Motor(left_motor_port)
        self.right_motor = Motor(right_motor_port)
        self.right_motor.set_reversed(True)
        self.imu = Inertial(imu_port)
        self.imu.calibrate()
        self.imu.set_turn_type(TurnType.LEFT)

    def hypot(self, x, y):
        return math.sqrt(x**2 + y**2)

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
        left_distance = (left_encoder_rev * 2 * math.pi * self.wheel_radius) / 12
        right_distance = (right_encoder_rev * 2 * math.pi * self.wheel_radius) / 12
        distance = (left_distance + right_distance) / 2
        delta_distance = distance - self.previous_distance
        self.theta = gyro_angle
        self.x += delta_distance * math.cos(gyro_angle)
        self.y += delta_distance * math.sin(gyro_angle)
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
            print(self.x, self.y, self.theta)

    def inverse(self, forward: float, omega: float) -> tuple[float, float]:
        vl = forward - ((omega * self.track_width) / 2)
        vr = forward + ((omega * self.track_width) / 2)
        return vl, vr

    def rpm_to_rads(self, rpm: float) -> float:
        return (rpm * 2 * math.pi) / 60

    def pid_test_to_point(self, target_point: Point, dt: float):
        distance_to_target = self.hypot(
            target_point.point[0] - self.x, target_point.point[1] - self.y
        )
        angle_to_target = math.atan2(
            target_point.point[1] - self.y, target_point.point[0] - self.x
        )
        heading_error = angle_to_target - self.theta
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        control_forward = -self.pid_forward.compute(distance_to_target, 0, dt)
        control_omega = self.pid_omega.compute(heading_error, 0, dt)
        return control_forward, control_omega, distance_to_target, heading_error

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
        vl_rpm = (vl * 60) / (2 * math.pi * drive.wheel_radius)
        vr_rpm = (vr * 60) / (2 * math.pi * drive.wheel_radius)
        self.left_motor.spin(FORWARD, vl_rpm, RPM)
        self.right_motor.spin(FORWARD, vr_rpm, RPM)


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
        forward_velocity: float,
        distance_parameter: float,
        curvature_parameter: float,
        max_lookahead: float,
        min_lookahead: float,
    ):
        self.forward_controller = forward_controller
        self.omega_controller = omega_controller
        self.drive = drive
        self.forward_velocity = forward_velocity
        self.distance_parameter = distance_parameter
        self.curvature_parameter = curvature_parameter
        self.previous_curvature = 0
        self.max_lookahead = max_lookahead
        self.min_lookahead = min_lookahead
        self.base_lookahead = 1

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))

    def calculate(self, path: list[Point]):
        robot_pose = (self.drive.x, self.drive.y)
        current_speed = abs(self.drive.speed)
        curvature_scale = self.previous_curvature * self.curvature_parameter
        lookahead = self.clamp(
            self.base_lookahead
            + (current_speed * self.distance_parameter)
            - curvature_scale,
            self.min_lookahead,
            self.max_lookahead,
        )
        path_points = [(point.point[0], point.point[1]) for point in path]
        theta = self.drive.theta
        distances = [
            math.sqrt((p[0] - robot_pose[0]) ** 2 + (p[1] - robot_pose[1]) ** 2)
            for p in path_points
        ]
        closest_idx = min(range(len(distances)), key=lambda i: distances[i])
        lookahead_point = path_points[-1]
        for i in range(closest_idx, len(path_points)):
            p = path_points[i]
            dist = math.sqrt((p[0] - robot_pose[0]) ** 2 + (p[1] - robot_pose[1]) ** 2)
            if dist >= lookahead:
                lookahead_point = p
                break
        dx = lookahead_point[0] - robot_pose[0]
        dy = lookahead_point[1] - robot_pose[1]
        alpha = math.atan2(dy, dx) - theta
        dist = math.sqrt(dx**2 + dy**2)
        if abs(dist) < 1e-10:
            curvature = 0.0
        else:
            curvature = (2.0 * math.sin(alpha)) / dist
        self.previous_curvature = curvature
        return curvature, lookahead

    def follow_path(self, path: list[Point]):
        bool_drive = True
        while bool_drive:
            curvature, _ = self.calculate(path)
            curvature_abs = abs(curvature)
            denominator = curvature_abs + 1e-6
            clip_value = 1 / denominator
            scaled_velocity = self.forward_velocity * max(0.5, min(1.0, clip_value))
            scaled_omega = scaled_velocity * curvature
            drive.drive(scaled_velocity, scaled_omega)
            dx = self.drive.x - path[-1].point[0]
            dy = self.drive.y - path[-1].point[1]
            distance_to_end = math.sqrt(dx**2 + dy**2)
            print(distance_to_end)
            if distance_to_end < 0.5:
                bool_drive = False
                self.drive.stop_drive()
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


brain = Brain()
controller = Controller()
drive = Drive(0, 0, 0, Ports.PORT1, Ports.PORT4, Ports.PORT3)
pure_pursuit_controller = Pure_Pursuit_Controller(
    drive, drive.pid_forward, drive.pid_omega, 7.0, 3.0, 0.5, 3.0, 0.5
)
path = [Point(0, 0), Point(11, 0), Point(11, 9)]  # ft
smooth = Chaikin_Smooth(path)
smoothed_path = smooth.smooth_path(4)
max_speed = 30
max_omega = math.pi

pure_pursuit_thread = Thread(lambda: pure_pursuit_controller.follow_path(smoothed_path))
odometry_thread = Thread(drive.update_pose())
