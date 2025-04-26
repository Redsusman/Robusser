from vex import *
import math


# class Point:
#     def __init__(self, x: float, y: float):
#         self.point = (x, y)


# class PIDController:
#     def __init__(self, kp: float, ki: float, kd: float):
#         self.kp = kp
#         self.ki = ki
#         self.kd = kd
#         self.previous_error = 0
#         self.integral = 0
#         self.error = 0

#     def compute(self, measured_value: float, setpoint: float, dt: float) -> float:
#         error = setpoint - measured_value
#         self.integral += error * dt
#         derivative = (error - self.previous_error) / dt
#         output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
#         self.previous_error = error
#         return output

#     def get_error(self):
#         return self.previous_error


class Drive:
    def __init__(self, x:float, y:float, theta: float, left_motor_port: int, right_motor_port: int, imu_port:int):
        self.track_width = 16.625
        self.wheel_base = 16.125
        self.wheel_diameter = 4
        self.wheel_radius = self.wheel_diameter/2
        self.x = x
        self.y = y
        self.theta = theta
        self.speed = 0
        self.omega = 0
        self.x_list = []
        self.y_list = []
        self.theta_list = []
        self.left_motor = Motor(left_motor_port)
        self.right_motor = Motor(right_motor_port)
        self.imu = Inertial(imu_port)

    # def forward(self, left_speed: float, right_speed: float, dt: float):
    #     speed = (right_speed + left_speed) / 2
    #     omega = (right_speed - left_speed) / self.track_width
    #     self.theta += omega * dt
    #     self.x += speed * math.cos(self.theta) * dt
    #     self.y += speed * math.sin(self.theta) * dt
    #     self.speed = speed
    #     self.omega = omega
    #     self.x_list.append(self.x)
    #     self.y_list.append(self.y)
    #     self.theta_list.append(self.theta)
    #     return speed, omega

    def inverse(self, forward: float, omega: float) -> tuple[float, float]:
        vl = forward - ((omega * self.track_width) / 2)
        vr = forward + ((omega * self.track_width) / 2)
        return vl,vr

    # def reset_pose(self, x, y, theta):
    #     self.x = x
    #     self.y = y
    #     self.theta = theta
    #     self.x_list = [x]
    #     self.y_list = [y]
    #     self.theta_list = [theta]

brain = Brain()
controller = Controller()
drive = Drive(0,0,0,Ports.PORT1,Ports.PORT4,Ports.PORT3)
max_speed = 30
max_omega = math.pi
dt = 5

while True:
    forward = (controller.axis1.position()/100) * max_speed
    omega = (controller.axis2.position()/100) * max_omega
    vl, vr = drive.inverse(forward, omega)
    vl_rpm = (vl * 60)/(2 * math.pi * drive.wheel_radius)
    vr_rpm = (vr * 60)/(2 * math.pi * drive.wheel_radius)
    drive.left_motor.spin(FORWARD, vl_rpm, RPM)
    drive.right_motor.spin(FORWARD, vr_rpm, RPM)
    
