#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped

import matplotlib.pyplot as plt
import numpy as np
import math
import a_star

def main():
    sx = -20.0  # [m]
    sy = 0.0  # [m]
    gx = 20.0  # [m]
    gy = 0.0  # [m]

    grid_size = 0.5  # [m]
    robot_size = 2.0  # [m]

    ox, oy = [], []

    for i in range(-30, 30):
        ox.append(i * grid_size)
        oy.append(-30)
    for i in range(-30, 30):
        ox.append(30)
        oy.append(i * grid_size)
    for i in range(-30, 30):
        ox.append(i * grid_size)
        oy.append(30)
    for i in range(-30, 30):
        ox.append(-30)
        oy.append(i * grid_size)
    for i in range(-20, 20):
        ox.append(-10)
        oy.append(i * grid_size)
    for i in range(-20, 20):
        ox.append(10)
        oy.append(i * grid_size)

    plt.plot(ox, oy, ".k")
    plt.plot(sx, sy, "xr")
    plt.plot(gx, gy, "xb")
    plt.grid(True)
    plt.axis("equal")

    rx, ry = a_star.a_star_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size)

    plt.plot(rx, ry, "-r")
    plt.show()


if __name__ == "__main__":
    main()