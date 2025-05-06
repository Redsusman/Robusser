from vex import *
import math

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

    def hypot(self, x, y):
        return math.sqrt(x**2 + y**2)

    def forward(self, left_speed: float, right_speed: float, dt: float):
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
        delta_theta = gyro_angle - self.previous_theta
        self.theta += delta_theta
        self.x += delta_distance * math.cos(gyro_angle + (delta_theta / 2))
        self.y += delta_distance * math.sin(gyro_angle + (delta_theta / 2))
        self.previous_distance = distance
        self.previous_theta = gyro_angle

    def update_pose(self, dt: float):
        while True:
            self.odometer(
                self.left_motor.position(RotationUnits.REV),
                self.right_motor.position(RotationUnits.REV),
                math.radians(self.imu.heading()),
            )
            wait(dt)

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

    def calculate(self, path: list[Point]):
        self.lookahead_distance = (
            self.lookahead_distance
            + self.distance_parameter * self.drive.speed
            - self.curvature_parameter * self.previous_curvature
        )
        robot_pose = (self.drive.x, self.drive.y)
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
            if dist >= self.lookahead_distance:
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
        return curvature

    def linspace_std(self, start, stop, num):
        if num <= 0:
            return []
        if num == 1:
            return [start]
        step = (stop - start) / (num - 1)
        return [start + i * step for i in range(num)]

    def follow_path(self, path: list[Point], dt):
        time_interval = self.linspace_std(0, 100, dt)
        for t in time_interval:
            curvature = self.calculate(path)
            curvature_abs = abs(curvature)
            denominator = curvature_abs + 1e-6
            clip_value = 1 / denominator
            scaled_velocity = self.forward_velocity * max(0.5, min(1.0, clip_value))

            vl, vr = self.drive.inverse(scaled_velocity, scaled_velocity * curvature)
            vl_rpm = (vl * 60) / (2 * math.pi * drive.wheel_radius)
            vr_rpm = (vr * 60) / (2 * math.pi * drive.wheel_radius)
            drive.left_motor.spin(FORWARD, vl_rpm, RPM)
            drive.right_motor.spin(FORWARD, vr_rpm, RPM)
            dx = self.drive.x - path[-1].point[0]
            dy = self.drive.y - path[-1].point[1]
            distance_to_end = math.sqrt(dx**2 + dy**2)

            if distance_to_end < 0.1:
                break
            wait(dt)


brain = Brain()
controller = Controller()
drive = Drive(0, 0, 0, Ports.PORT1, Ports.PORT4, Ports.PORT3)
pure_pursuit_controller = Pure_Pursuit_Controller(
    drive, drive.pid_forward, drive.pid_omega, 5, 25, 0, 0
)
path = [Point(0, 0), Point(6, 30), Point(12, 60), Point(24, 70)]
smooth = Chaikin_Smooth(path)
smoothed_path = smooth.smooth_path(4)
max_speed = 30
max_omega = math.pi
dt = 0.05

odometry_thread = Thread(lambda: drive.update_pose(dt))
path_following_thread = Thread(
    lambda: pure_pursuit_controller.follow_path(smoothed_path, dt)
)
