import numpy as np

def angle2rad(angle_list):
    reult = [angle / 180 * 3.1415926 for angle in angle_list]
    print("  yaw: {}\n  pitch: {}\n  roll: {}\n".format(reult[0], reult[1], reult[2]))


data = [[100.464, 0.636, 0.773],
        [0.75, 0.436, -0.05],
        [50.3, 0.447, -0.502],
        [50.3, 0.447, -0.0502],
        [-50.526, 0.915, 1.052],
        [-98.991, 0.194, -0.221],
        [-148.7018, 0.899, -0.849],
        [150.695, -0.078, 0.034],
        [1.002, 0.045, -0.962],
        [-0.272, -0.438, 0.45]]

print(angle2rad(data[-1]))
