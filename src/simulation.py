import numpy as np
import math
import matplotlib.pyplot as plt

lookaheads=[]

class Point:
    def __init__(self, x, y):
        self.point = (x, y)


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


class Drive:
    def __init__(self, x, y, theta, track_width, wheel_base):
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
        self.velocities = []
        self.time_steps = []  # Track time/step index

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
        self.time_steps.append(len(self.time_steps))  # Track step index

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

class Stanley_Controller:
    def __init__(self, drive: Drive, k: float, vel, lookahead):
        self.drive = drive
        self.k = k
        self.vel = vel
        self.lookahead = lookahead
        pass
    
    def calculate(self, path: list[Point]):
        robot_pose = (self.drive.x, self.drive.y)
        theta = self.drive.theta
        distances = [math.hypot(p.point[0] - robot_pose[0], p.point[1] - robot_pose[1]) for p in path]
        closest_idx = distances.index(min(distances))
        shortest_point = path[closest_idx]
        next_point = path[closest_idx + 1] if closest_idx + 1 < len(path) else shortest_point
        dx = next_point.point[0] - shortest_point.point[0]
        dy = next_point.point[1] - shortest_point.point[1]
        path_heading = math.atan2(dy, dx)
        heading_error = math.atan2(math.sin(path_heading - theta), math.cos(path_heading - theta))
        rx = robot_pose[0] - shortest_point.point[0]
        ry = robot_pose[1] - shortest_point.point[1]
        cross_track_error = rx * math.sin(path_heading) - ry * math.cos(path_heading)
        v = max(self.drive.speed, 0.1)
        steering_angle = heading_error + math.atan((self.k * cross_track_error) / v)
        steering_radius = self.drive.wheel_base / (math.tan(steering_angle) + 1e-6)

        return steering_angle
    
    def calculate_feedback(self, path: list[Point], heading_kP, cte_kP, dist_kP):
        robot_pose = (self.drive.x, self.drive.y)
        theta = self.drive.theta
        distances = [math.hypot(p.point[0] - robot_pose[0], p.point[1] - robot_pose[1]) for p in path]
        closest_idx = distances.index(min(distances))
        shortest_point = path[closest_idx]
        next_point = path[closest_idx + 1] if closest_idx + 1 < len(path) else shortest_point
        dx = next_point.point[0] - shortest_point.point[0]
        dy = next_point.point[1] - shortest_point.point[1]
        path_heading = math.atan2(dy, dx)
        heading_error = math.atan2(math.sin(path_heading - theta), math.cos(path_heading - theta))
        rx = robot_pose[0] - shortest_point.point[0]
        ry = robot_pose[1] - shortest_point.point[1]
        cross_track_error = rx * math.sin(path_heading) - ry * math.cos(path_heading)
        v = max(self.drive.speed, 0.1)
        steering_angle = heading_error + math.atan((self.k * cross_track_error) / v)
        steering_radius = self.drive.wheel_base / (math.tan(steering_angle) + 1e-6)
        dist_error = math.hypot(shortest_point.point[0]-robot_pose[0], shortest_point.point[1] - robot_pose[1])

        heading_correction = heading_kP*heading_error
        cte_correction = cte_kP*cross_track_error
        dist_correction = dist_kP*dist_error

        return dist_correction, cte_correction+heading_correction

    def follow_path_feedback(self, path: list[Point],heading_kP, cte_kP, dist_kP):
        dt = 0.1
        for t in range(1000):
            dist_correction, omega = self.calculate_feedback(path,heading_kP, cte_kP, dist_kP)
            v = 0.5+dist_correction
            vl, vr = self.drive.inverse(v, omega)
            print(v,omega)
            self.drive.forward(vl, vr, dt)
            if (
                np.linalg.norm(
                    np.array([self.drive.x, self.drive.y]) - np.array(path[-1].point)
                )
                < 0.1
            ):
                break

    
    def follow_path(self, path: list[Point]):
        dt = 0.1
        for t in range(1000):
            radius = self.calculate(path)
            vl, vr = self.drive.inverse(self.vel, radius)
            self.drive.forward(vl, vr, dt)
            if (
                np.linalg.norm(
                    np.array([self.drive.x, self.drive.y]) - np.array(path[-1].point)
                )
                < 0.1
            ):
                break


class Pure_Pursuit_Controller:
    def __init__(
        self, drive: Drive, speed_parameter: float
    ):
        self.drive = drive
        self.speed_parameter = speed_parameter
        self.min_lookahead = 0.2
        self.max_lookahead = 0.75
        self.max_accel = 0.5
        self.max_vel = 0.5
        self.max_omega = math.pi

    def velocity_profile(self, max_accel, max_vel, path: list[Point]):
        if not path:
            return []
        distances = [math.hypot(p2.point[0] - p1.point[0], p2.point[1] - p1.point[1]) 
                for p1, p2 in zip(path[:-1], path[1:])]
    
    # Initialize velocity arrays
        forward_vel = [0.0] * len(path)  # Forward pass (accelerate)
        backward_vel = [0.0] * len(path)  # Backward pass (decelerate)
        forward_vel[0] = 0.0  # Start at rest
        for i in range(1, len(path)):
            forward_vel[i] = math.sqrt(forward_vel[i-1]**2 + 2 * max_accel * distances[i-1])
            if forward_vel[i] > max_vel:
                forward_vel[i] = max_vel

        backward_vel[-1] = 0.0  
        for i in range(len(path)-2, -1, -1):
            backward_vel[i] = math.sqrt(backward_vel[i+1]**2 + 2 * max_accel * distances[i])
            if backward_vel[i] > max_vel:
                backward_vel[i] = max_vel
    
        profile = [min(fv, bv) for fv, bv in zip(forward_vel, backward_vel)]
        profile[0]+=0.01
        return profile
        
        
    def calculate(self, path: list[Point]):
        robot_pose = (self.drive.x, self.drive.y)
        current_speed = abs(self.drive.speed)
        lookahead = np.clip(self.speed_parameter*current_speed, self.min_lookahead, self.max_lookahead)
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
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        dist = math.sqrt(dx**2 + dy**2)
        if abs(dist) < 1e-10:
            curvature = 0.0
        else:
            curvature = (2.0 * math.sin(alpha)) / lookahead #use lookahead instead of dist
        return curvature, lookahead
    
    def velocity_scaler(self, curvature: float, max_speed: float, max_omega: float):
    # Small curvature threshold to avoid division by near-zero
        curvature_threshold = 1e-6
    
        if abs(curvature) < curvature_threshold:
            return max_speed
        radius = 1.0 / max(abs(curvature), curvature_threshold)
        max_centripetal_accel = 2.0  # Maximum allowed centripetal acceleration (m/s²)
        smoothing_factor = 0.2# Controls how gradually velocity changes with curvature
        max_curve_speed = math.sqrt(max_centripetal_accel * radius)
        max_omega_speed = max_omega / max(abs(curvature), curvature_threshold)
        base_speed = min(max_speed, max_curve_speed, max_omega_speed)
        if hasattr(self, 'last_speed'):
            smoothed_speed = (1 - smoothing_factor) * self.last_speed + smoothing_factor * base_speed
        else:
            smoothed_speed = base_speed
        self.last_speed = smoothed_speed  
        return smoothed_speed

    def follow_path(self, path: list[Point]):
        dt = 0.05  # Time step
        global_velocities = self.velocity_profile(self.max_accel, self.max_vel, path)
    
        for t in range(1000):
            robot_pos = (self.drive.x, self.drive.y)
            curvature, lookahead = self.calculate(path)
            closest_idx = min(range(len(path)), 
                         key=lambda i: math.hypot(path[i].point[0]-robot_pos[0], 
                                               path[i].point[1]-robot_pos[1]))
            
            target_velocity_global = global_velocities[closest_idx]
            target_velocity_local = self.velocity_scaler(curvature, self.max_vel, self.max_omega)
            target_velocity = min(target_velocity_global, target_velocity_local)
            if hasattr(self, 'last_velocity'):
                max_dv = self.max_accel * dt
                target_velocity = np.clip(target_velocity, 
                                    self.last_velocity - max_dv, 
                                    self.last_velocity + max_dv)
            omega = curvature * target_velocity
            vl, vr = self.drive.inverse(target_velocity, omega)
            self.drive.forward(vl, vr, dt)
            self.last_velocity = target_velocity
            self.drive.velocities.append(target_velocity)
            if np.linalg.norm(np.array([self.drive.x, self.drive.y]) - np.array(path[-1].point)) < 0.1:
                break

path = [Point(0, 0), Point(3,3), Point(6,7), Point(10,5), Point(14,3), Point(10,1), Point(7,-3)]
smoothener = Chaikin_Smooth(path)
smooth_path = smoothener.smooth_path(3)
drive = Drive(0, 0,3*math.pi/2, 3,3)
controller = Pure_Pursuit_Controller(drive, 0.4)
stanley = Stanley_Controller(drive, 3.0, 1.0, 3)
stanley.follow_path_feedback(smooth_path,2.0,2.0,0.5)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), gridspec_kw={'height_ratios': [2, 1]})

# 1. Plot Robot Path vs. Reference (Top Subplot)
ax1.plot(drive.x_list, drive.y_list, label="Robot Path", color="blue", linewidth=2)
ax1.plot(
    [p.point[0] for p in smooth_path],
    [p.point[1] for p in smooth_path],
    label="Reference Path",
    linestyle="--",
    color="orange",
)
ax1.scatter(drive.x_list[0], drive.y_list[0], color='green', marker='o', s=100, label='Start')
ax1.scatter(drive.x_list[-1], drive.y_list[-1], color='red', marker='x', s=100, label='End')
ax1.set_xlabel("X Position")
ax1.set_ylabel("Y Position")
ax1.set_title("Robot Trajectory vs. Reference Path")
ax1.legend()
ax1.grid(True)
ax1.axis('equal')

# 2. Plot Velocity vs. Path Position (Bottom Subplot)
# Compute arc length (skip the first point to match velocities)
# arc_length = np.cumsum([np.hypot(drive.x_list[i] - drive.x_list[i-1], 
#                                 drive.y_list[i] - drive.y_list[i-1]) 
#                        for i in range(1, len(drive.x_list))])

# # Ensure velocities and arc_length have the same shape
# assert len(arc_length) == len(drive.velocities), "Mismatched dimensions!"

# # Plot velocity vs. arc length
# ax2.plot(arc_length, drive.velocities, label="Velocity", color="purple", linewidth=2)

# ax2.plot(arc_length, drive.velocities, label="Velocity", color="purple", linewidth=2)
# ax2.plot(arc_length, lookaheads, label="Lookahead", linestyle="--", color="red")
# ax2.set_xlabel("Distance Along Path (m)")
# ax2.set_ylabel("Velocity (m/s)")
# ax2.set_title("Velocity Profile vs. Path Position")
# ax2.legend()
# ax2.grid(True)

# plt.tight_layout()
plt.show()
