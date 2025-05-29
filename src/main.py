from vex import *
import math

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
        return self.sensor.object_distance(DistanceUnits.IN)


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
        self.track_width = 16.625/12
        self.wheel_base = 16.125/12
        self.wheel_diameter = 4/12
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
        self.theta_controller = PIDController(0.01,0.01,0.0)

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
        left_distance = (left_encoder_rev * 2 * math.pi * self.wheel_radius)
        right_distance = (right_encoder_rev * 2 * math.pi * self.wheel_radius)
        distance = (left_distance + right_distance) / 2
        delta_distance = distance - self.previous_distance
        self.theta = gyro_angle
        self.x += delta_distance * math.cos(gyro_angle)
        self.y += delta_distance * math.sin(gyro_angle)

        self.previous_distance = distance
        # print(left_distance, right_distance)
        # 

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
            new_points.insert(0, self.points[0])  # add first point
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
        distances = [hypot(p.point[0] - robot_pose[0], p.point[1] - robot_pose[1]) for p in path]
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
        dist_error = hypot(shortest_point.point[0]-robot_pose[0], shortest_point.point[1] - robot_pose[1])

        heading_correction = heading_kP*heading_error
        cte_correction = cte_kP*cross_track_error
        dist_correction = dist_kP*dist_error
        return dist_correction, cte_correction+heading_correction

    def follow_path_feedback(self, path: list[Point],heading_kP, cte_kP, dist_kP):
        bool_drive = True
        while bool_drive:
            dist_correction, omega = self.calculate_feedback(path,heading_kP, cte_kP, dist_kP)
            v = self.vel+dist_correction
            self.drive.drive(v, omega)
            dx = self.drive.x - path[-1].point[0]
            dy = self.drive.y - path[-1].point[1]
            distance_to_end = math.sqrt(dx**2 + dy**2)

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
                child.g = current_node.g + hypot(
                    child.idx - current_node.idx, child.idy - current_node.idy
                )
                child.h = hypot(end.idx - child.idx, end.idy - child.idy)
                child.f = child.h + child.g
                child.parent = current_node  # new code
                if child not in open:
                    heappush(open, child)

        return None

    def find_path_dstar_lite(self):
        pass

class Elevator:
    def __init__(self, brain: Brain):
        self.pneumatic = Pneumatics(brain.three_wire_port.g)

    def up(self):
        self.pneumatic.close()

    def down(self):
        self.pneumatic.open()

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
    denominator = hypot(x2-x1, y2-y1)
    return numerator / denominator





# controller.buttonA.pressed(lambda: elevator.up())
# controller.buttonB.pressed(lambda: elevator.down())

grid = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
    [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
    [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
    [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
    [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
    [1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1],
    [1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1],
    [1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1],
    [1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1],
    [1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0],
    [1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1],
    [1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1],
    [1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1],
    [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
    [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
    [1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1],
    [1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1],
    [1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1],
    [1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1],
    [1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1],
    [1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1],
    [1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1],
    [1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1],
    [1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1],
    [1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1],
    [1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1],
    [1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1],
    [1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1],
    [1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
]

astar = AStar_Path_Follower(grid)

# Define start and end nodes
start = astar.node_grid[0][22]  # Top-left corner
end = astar.node_grid[6][8]  # Bottom-right corner

# Find the path
path = astar.find_path(start, end)

# dx = path[1].point[0] - path[0].point[0]
# dy = path[1].point[1] - path[0].point[1]
# initial_heading = math.atan2(dy, dx)

brain = Brain()
controller = Controller()
drive = Drive(0, 23, 0, Ports.PORT1, Ports.PORT4, Ports.PORT3)
control = Stanley_Controller(drive, 2.0, 0.3,0.0)
# controllerr = Pure_Pursuit_Controller(drive, 0.5, 0.5, 5.0, 1.5)
elevator = Elevator(brain)

if path is not None:
    smooth = Chaikin_Smooth(path)
    smoothed_path = smooth.smooth_path(4)
else:
    print("Pathfinding failed. No valid path found.")

def run():
    # dy = smoothed_path[1].point[1] - smoothed_path[0].point[1]
    # dx = smoothed_path[1].point[0] - smoothed_path[0].point[0]
    # angle = math.atan2(dy, dx)
    # normalized_angle = (angle + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]
    # drive.turn_to_angle(normalized_angle)
    control.follow_path_feedback(smoothed_path,2.0,2.0, 0.0)
    # elevator.down()

# pure_pursuit_thread = Thread(lambda: controllerr.follow_path(smoothed_path))

while drive.imu.is_calibrating():
    wait(100, MSEC)
stanley_thread = Thread(lambda: run())
odometry_thread = Thread(drive.update_pose())



# # left_motor = Motor(Ports.PORT1)
# # right_motor = Motor(Ports.PORT4)
# # controller = Controller()

# # while True:
# #     if controller.buttonR1.pressing():
# #         left_motor.spin(FORWARD, 100, PERCENT)
# #         right_motor.spin(FORWARD, 100, PERCENT)
