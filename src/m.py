# from hub import *
# import runloop
# import math
# import motor_pair
# import motor

# class Point:
#     def __init__(self,x,y):
#         self.point = (x,y)

# class Drive:
#     def __init__(self):
#         self.wheel_radius = 1.0
#         self.wheel_diameter = self.wheel_radius*2
#         self.track_width = 4.0
#         motor_pair.pair(motor_pair.PAIR_1, port.C, port.D)
#         self.x=0.0
#         self.y=0.0
#         self.theta=0.0

#     def inverse(self,forward_velocity, omega):
#         vl = (forward_velocity - (omega*self.track_width)/2)
#         vr = (forward_velocity + (omega*self.track_width)/2)
#         return vl, vr

#     def drive(self, vl, vr):
#         motor_pair.move_tank(motor_pair.PAIR_1, vl, vr)

#     def odometer(self):
#         right_encoder = (self.drive.wheel_diameter * math.pi) * (motor.absolute_position(port.D)/360)
#         left_encoder = (self.drive.wheel_diameter*math.pi) * (motor.absolute_position(port.C)/360)
#         theta = (right_encoder-left_encoder)/self.track_width
#         distance = (right_encoder + left_encoder)/2
#         self.theta+=theta
#         self.x+= distance * math.cos(theta)
#         self.y+= distance * math.sin(theta)

# class Pure_Pursuit_Controller:
#     def __init__(self, lookahead, drive):
#         self.lookahead = lookahead
#         self.drive = drive

#     def calculate(self, path: list[Point]):
#         robot_pose = (self.drive.x, self.drive.y)
#         path_points = [(point.point[0], point.point[1]) for point in path]
#         theta = self.drive.theta
#         distances = [
#             math.sqrt((p[0] - robot_pose[0]) ** 2 + (p[1] - robot_pose[1]) ** 2)
#             for p in path_points
#         ]
#         closest_idx = min(range(len(distances)), key=lambda i: distances[i])
#         lookahead_point = path_points[-1]
#         for i in range(closest_idx, len(path_points)):
#             p = path_points[i]
#             dist = math.sqrt((p[0] - robot_pose[0]) ** 2 + (p[1] - robot_pose[1]) ** 2)
#             if dist >= self.lookahead:
#                 lookahead_point = p
#                 break
#         dx = lookahead_point[0] - robot_pose[0]
#         dy = lookahead_point[1] - robot_pose[1]
#         alpha = math.atan2(dy, dx) - theta
#         dist = math.sqrt(dx**2 + dy**2)
#         if abs(dist) < 1e-10:
#             curvature = 0.0
#         else:
#             curvature = (2.0 * math.sin(alpha)) / dist
#         self.previous_curvature = curvature
#         return curvature

# class Chaikin_Smooth:
#     def __init__(self, points: list[Point]):
#         self.points = points

#     def smooth_path(self, num_iterations: int):
#         for _ in range(num_iterations):
#             new_points = []
#             for i in range(len(self.points) - 1):
#                 p0 = (self.points[i].point)
#                 p1 = (self.points[i + 1].point)
#                 q = (0.75 * p0[0] + 0.25 * p1[0], 0.75 * p0[1] + 0.25 * p1[1])
#                 r = (0.25 * p0[0] + 0.75 * p1[0], 0.25 * p0[1] + 0.75 * p1[1])
#                 q_point = Point(q[0], q[1])
#                 r_point = Point(r[0], r[1])
#                 new_points.append(q_point)
#                 new_points.append(r_point)
#             new_points.insert(0, self.points[0])# add first point
#             new_points.append(self.points[-1])
#             self.points = new_points
#         return self.points

# drivetrain = Drive()
# path = [Point(0,0), Point(7,0), Point(7,5), Point(0,5)]
# smooth = Chaikin_Smooth(path)
# smoothened_path = smooth.smooth_path(3)
# controller = Pure_Pursuit_Controller(1.5,drivetrain)
# max_vel = 2.0

# async def main():
#     # write your code here
#     drivetrain = Drive()
#     drivetrain.odometer()
#     curvature = controller.calculate(smoothened_path)
#     vl, vr = drivetrain.inverse(max_vel, max_vel*curvature)
#     drivetrain.drive(vl, vr)
    


# runloop.run(main())
