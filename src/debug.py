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
import reeds_shepp

is_a_star = False
is_reeds_shepp = True

def main():
    if is_a_star:
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

    if is_reeds_shepp:
        start_x = 0.0  # [m]
        start_y = 0.0  # [m]
        start_yaw = math.radians(0.0)  # [rad]

        end_x = 0.5  # [m]
        end_y = 0.5  # [m]
        end_yaw = math.radians(120.0)  # [rad]

        curvature = 1.0
        step_size = 0.1

        px, py, pyaw, mode, clen = reeds_shepp.reeds_shepp_path_planning(
            start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature, step_size)

        plt.cla()
        plt.plot(px, py, ".-b", label="final course " + str(mode))

        # plotting
        reeds_shepp.plot_arrow(start_x, start_y, start_yaw)
        reeds_shepp.plot_arrow(end_x, end_y, end_yaw)

        plt.legend()
        plt.grid(True)
        plt.xlim((min(start_x, end_x) - 3, max(start_x, end_x) + 3))
        plt.ylim((min(start_y, end_y) - 3, max(start_y, end_y) + 3))
        plt.show()

        print max(start_x, end_x)

if __name__ == "__main__":
    main()