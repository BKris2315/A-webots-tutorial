import numpy as np

wheel_radius = 0.025
dist_betw_wheel = 0.09

wheel_circ = 2 * 3.1415 * wheel_radius
enc_un = wheel_circ / 6.283

lin_vel = wheel_radius * 6.28
    
angle_of_rot =  3.1415 / 3
rot_rate = (2 * lin_vel) / dist_betw_wheel
rot_time = 1.5754 / rot_rate * 0.75
print(rot_time*1000)