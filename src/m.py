import math
import matplotlib.pyplot as plt

class Point:
     def __init__(self,x,y):
        self.point = (x,y)

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

def velocity_profile(max_accel, max_vel, path: list[Point]):
        if not path:
            return []
        distances = [math.hypot(p2.point[0] - p1.point[0], p2.point[1] - p1.point[1]) 
                for p1, p2 in zip(path[:-1], path[1:])]
    
        forward_vel = [0.0] * len(path) 
        backward_vel = [0.0] * len(path)
        forward_vel[0] = 0.0 
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
    
        return profile

# Define the path
path = [Point(0, 0), Point(1, 1), Point(2, 0), Point(3, 1)]

# Smooth the path
smoothener = Chaikin_Smooth(path)
smooth_path = smoothener.smooth_path(3)

# Generate the velocity profile
vel_profile = velocity_profile(1, 1, smooth_path)

# Calculate distances between points
distances = [
    math.sqrt(
        (smooth_path[i + 1].point[0] - smooth_path[i].point[0]) ** 2
        + (smooth_path[i + 1].point[1] - smooth_path[i].point[1]) ** 2
    )
    for i in range(len(smooth_path) - 1)
]

# Plot the smoothed path
plt.figure(figsize=(12, 6))

# Subplot 1: Path
plt.subplot(1, 3, 1)
plt.plot(
    [p.point[0] for p in smooth_path],
    [p.point[1] for p in smooth_path],
    marker="o",
    label="Smooth Path",
)
plt.title("Smoothed Path")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.grid()

# Subplot 2: Distances
plt.subplot(1, 3, 2)
plt.bar(range(len(distances)), distances, color="orange", label="Distance Steps")
plt.title("Distance Between Points")
plt.xlabel("Segment Index")
plt.ylabel("Distance")
plt.legend()
plt.grid()

# Subplot 3: Velocity Profile
plt.subplot(1, 3, 3)
plt.plot(vel_profile, marker="o", label="Velocity Profile")
plt.title("Velocity Profile")
plt.xlabel("Path Index")
plt.ylabel("Velocity")
plt.legend()
plt.grid()

# Show the plots
plt.tight_layout()
plt.show()