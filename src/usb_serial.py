import math
list = [[0,10], [12,14], [23, 15], [20, 12]]

pose = [19,11]

distances = [math.hypot(point[0] - pose[0], point[1] - pose[1]) for point in list]
print(list[distances.index(min(distances))])

