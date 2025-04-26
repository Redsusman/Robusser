import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
from matplotlib import patches
import pint as units
x_list=[]
y_list=[]

def inch_to_meter(value: float) -> float:
    return value / 39.37


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


class SimulatedDrivetrain:
    track_width_inches = 17.1875
    wheel_base_inches = 16
    wheel_diameter_inches = 4
    wheel_diameter_radius = wheel_diameter_inches / 2

    left_motor_port = 1
    right_motor_port = 2
    inertial_port = 4
    max_speed = 10

    def __init__(self, x_initial, y_initial, theta_initial):
        self.vl, self.vr = 0, 0
        self.v = (self.vl+self.vr)/2
        self.x = x_initial
        self.y = y_initial
        self.theta = theta_initial
        self.width = 2
        self.length = 1
        self.omega = (self.vr-self.vl)/self.track_width_inches
        self.x_trajectory: list[float] = [self.x]
        self.y_trajectory: list[float] = [self.y]
        self.theta_trajectory: list[float] = [self.theta]
        self.velocity: list[float] = []

    def forward_kinematics(self, left_speed, right_speed, dt):
        left_speed = (left_speed * 2 * math.pi * self.wheel_diameter_radius) / 60
        right_speed = (right_speed * 2 * math.pi * self.wheel_diameter_radius) / 60
        speed = (left_speed + right_speed) / 2
        omega = (right_speed - left_speed) / self.track_width_inches
        self.theta += omega * dt
        self.x += speed * math.cos(self.theta) * dt
        self.y += speed * math.sin(self.theta) * dt
        self.vr = right_speed
        self.vl = left_speed
        self.x_trajectory.append(self.x)
        self.y_trajectory.append(self.y)
        self.theta_trajectory.append(self.theta)
        self.velocity.append(speed)
     
        # print(abs(omega))

        # if abs(omega) < 1e-6:
        #     self.x += speed * math.cos(self.theta) * dt
        #     self.y += speed * math.sin(self.theta) * dt
        # else:
        #     R = speed / omega
        #     icc_x = self.x - R * math.sin(self.theta)
        #     icc_y = self.y + R * math.cos(self.theta)
        #     # self.theta += omega * dt
        #     self.x = icc_x + R * math.sin(self.theta)
        #     self.y = icc_y - R * math.cos(self.theta)
        #     self.x_trajectory.append(self.x)
        #     self.y_trajectory.append(self.y)
        #     self.theta_trajectory.append(self.theta)
        #     self.velocity.append(speed)
        return self.x, self.y, self.theta

    def inverse_kinematics(self, velocity, omega):
        left_speed = velocity - (omega * self.track_width_inches) / 2
        right_speed = velocity + (omega * self.track_width_inches) / 2
        angular_left_speed_rpm = (
            left_speed * 60 / (self.wheel_diameter_radius * 2 * math.pi)
        )
        angular_right_speed_rpm = (
            right_speed * 60 / (self.wheel_diameter_radius * 2 * math.pi)
        )

        return angular_left_speed_rpm, angular_right_speed_rpm


class Point:
    def __init__(self, x: float, y: float):
        self.point = (x, y)


class Controller:
    def __init__(
        self,
        forward_controller: PIDController,
        theta_controller: PIDController,
        drivetrain: SimulatedDrivetrain,
    ):
        self.forward_controller = forward_controller
        self.theta_controller = theta_controller
        self.drivetrain = drivetrain
        self.measured_values = []
        self.theta_measured = []

    def reset_pose(self, x: float, y: float, theta: float) -> None:
        self.drivetrain.x = x
        self.drivetrain.y = y
        self.drivetrain.theta = theta
        self.drivetrain.x_trajectory = [x]
        self.drivetrain.y_trajectory = [y]
        self.drivetrain.theta_trajectory=[theta]
        self.drivetrain.velocity = []

    def pure_pursuit(self, path: list[Point], max_lookahead_distance: float) -> tuple[float,float]:
        robot_pose = np.array([self.drivetrain.x, self.drivetrain.y])
        path_to_array = np.array([point.point for point in path])
        theta = self.drivetrain.theta
        distances = np.linalg.norm(path_to_array - robot_pose, axis=1)
        lookahead_indices = np.where(distances >= max_lookahead_distance)[0]
        lookahead_index = lookahead_indices[0] if len(lookahead_indices) > 0 else len(path_to_array) - 1
        lookahead_point = path_to_array[lookahead_index]
        alpha = np.arctan2(*(lookahead_point-robot_pose))-theta
        dx,dy = lookahead_point-robot_pose
        dist = np.linalg.norm(lookahead_point-robot_pose)
        y_lateral = -np.sin(theta)*dx+np.cos(theta)*dy
        curvature = (2*y_lateral)/(dist+1e-6)**2
        steering_angle = np.tan(curvature*self.drivetrain.wheel_base_inches)

        return curvature, steering_angle


        
    def run_app(self, path: list[Point]):
        max_vel=1.0
        time = np.linspace(0,100,100)
        dt=time[1]-time[0]
        for t in time:
            curvature, steering_angle = self.pure_pursuit(path,5)
            scaled_vel = np.clip(max_vel * ((1/(abs(curvature)+1e-10))**1), 0, max_vel)
            vl, vr = self.drivetrain.inverse_kinematics(max_vel, curvature*max_vel)
            self.drivetrain.forward_kinematics(vl,vr, dt)
# Initialize the drivetrain and controller
# Initialize the drivetrain and controller


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


# filepath: /c:/Users/2168/Documents/vex-vscode-projects/Robusser/src/sim.py
# Initialize the drivetrain and controller
from matplotlib.patches import Rectangle

# filepath: /c:/Users/2168/Documents/vex-vscode-projects/Robusser/src/sim.py
plt.ion()
drive = SimulatedDrivetrain(0, 0, 0)
control = Controller(PIDController(1, 0.0, 0), PIDController(20.0,0, 0), drive)
# Define the path
path = [Point(0, 0), Point(3,4), Point(5,8), Point(10,4), Point(14,8)]
smoother = Chaikin_Smooth(path)
smoothed_path = smoother.smooth_path(3)
control.run_app(smoothed_path)
fig, ax = plt.subplots()
ax.set_xlim(
    min(drive.x_trajectory) - 1, max(drive.x_trajectory) + 1
)  # Set x-axis limits
ax.set_ylim(
    min(drive.y_trajectory) - 1, max(drive.y_trajectory) + 1
)  # Set y-axis limits
ax.plot(
    [x.point[0] for x in smoothed_path],
    [x.point[1] for x in smoothed_path],
    "",
    label="True Path",
)

ax.plot([x for x in drive.x_trajectory], [y for y in drive.y_trajectory], label="Robot Path")

# # Initialize the quiver (arrow) to represent the robot
# quiver = ax.quiver(
#     drive.x_trajectory[0],  # Initial x position
#     drive.y_trajectory[0],  # Initial y position
#     np.cos(drive.theta_trajectory[0]),  # Initial x direction (cosine of theta)
#     np.sin(drive.theta_trajectory[0]),  # Initial y direction (sine of theta)
#     angles="xy",
#     scale_units="xy",
#     scale=1,
#     color="red",
#     label="Robot",
# )

# Add a hollow rectangle to represent the robot
# robot_width = 0.5  # Width of the robot
# robot_height = 0.3  # Height of the robot
# robot_rect = Rectangle(
#     (
#         drive.x_trajectory[0] - robot_width / 2,
#         drive.y_trajectory[0] - robot_height / 2,
#     ),  # Initial position
#     robot_width,
#     robot_height,
#     angle=np.degrees(drive.theta_trajectory[0]),  # Initial orientation
#     edgecolor="blue",
#     fill=False,
#     label="Robot Body",
# )
# ax.add_patch(robot_rect)


# Animation update function
# def update(frame):
#     # Update the quiver's position and direction
#     quiver.set_offsets(
#         [drive.x_trajectory[frame], drive.y_trajectory[frame]]
#     )  # Update position
#     velocity = (drive.velocity[frame]*3)**2
#     quiver.set_UVC(
#         np.cos(drive.theta_trajectory[frame]),  # Update x direction
#         np.sin(drive.theta_trajectory[frame]),  # Update y direction
#     )

#     # Update the rectangle's position and orientation
#     robot_rect.set_xy(
#         (
#             drive.x_trajectory[frame] - robot_width / 2,
#             drive.y_trajectory[frame] - robot_height / 2,
#         )
#     )
#     robot_rect.set_angle(
#         np.degrees(drive.theta_trajectory[frame])
#     )  # Update orientation

#     return quiver, robot_rect


# # Create the animation
# ani = FuncAnimation(
#     fig, update, frames=range(0, len(drive.x_trajectory), 30), interval=1, blit=False
# )

# Show the animation
plt.show(block=True)
