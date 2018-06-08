#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped
import math

import matplotlib.pyplot as plt
import a_star
import reeds_shepp

show_animation = True
is_a_star = False
is_reeds_sheep = True


class GlobalPathPlanner:
    def __init__(self):
        self.map = OccupancyGrid()
        self.pub_global_path = rospy.Publisher('/robot0/global_path', Path, queue_size=1)
        self.path = Path()

    def goal_cb(self, goal):
        quaternion_world2goal = (
            goal.pose.orientation.x,
            goal.pose.orientation.y,
            goal.pose.orientation.z,
            goal.pose.orientation.w)
        euler_angle = tf.transformations.euler_from_quaternion(quaternion_world2goal, axes='sxyz')
        rospy.loginfo("Goal set to x = {0} [m], y = {1} [m], yaw = {2} [deg])"
                      .format(str(goal.pose.position.x), str(goal.pose.position.y), str(euler_angle[2]/math.pi*180)))

        # A star algorithm
        if is_a_star:
            sx = 0.0  # [m]
            sy = 0.0  # [m]
            gx = goal.pose.position.x  # [m]
            gy = goal.pose.position.y  # [m]

            grid_size = self.map.info.resolution   # [m]
            offset_x = self.map.info.origin.position.x
            offset_y = self.map.info.origin.position.y
            robot_size = 0.5  # [m]

            ox, oy = [], []

            for i in range(self.map.info.height):
                for j in range(self.map.info.width):
                    if self.map.data[i * self.map.info.width + j] > 0:
                        ox.append(j*grid_size + offset_x)
                        oy.append(i*grid_size + offset_y)

            if show_animation:
                plt.plot(ox, oy, ".k")
                plt.plot(sx, sy, "xr")
                plt.plot(gx, gy, "xb")
                plt.grid(True)
                plt.axis("equal")

            rx, ry = a_star.a_star_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size)
            ryaw = []
            for i in range(len(rx)-1):
                ryaw.append(math.atan2(ry[i]-ry[i+1], rx[i]-rx[i+1]))
            ryaw.append(ryaw[-1])

            if show_animation:
                plt.plot(rx, ry, "-r")
                plt.show()

        if is_reeds_sheep:
            start_x = 0.0  # [m]
            start_y = 0.0  # [m]
            start_yaw = math.radians(0.0)  # [rad]

            end_x = goal.pose.position.x  # [m]
            end_y = goal.pose.position.y  # [m]
            end_yaw = euler_angle[2]  # [rad]

            curvature = 1.0
            step_size = 0.1

            rx, ry, ryaw, mode, clen = reeds_shepp.reeds_shepp_path_planning(
                start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature, step_size)

            if show_animation:
                plt.cla()
                plt.plot(rx, ry, label="final course " + str(mode))

                # plotting
                reeds_shepp.plot_arrow(start_x, start_y, start_yaw)
                reeds_shepp.plot_arrow(end_x, end_y, end_yaw)

                plt.legend()
                plt.grid(True)
                plt.xlim((min(start_x, end_x) - 3, max(start_x, end_x) + 3))
                plt.ylim((min(start_y, end_y) - 3, max(start_y, end_y) + 3))
                plt.show()

        for ix, iy, iyaw in zip(rx, ry, ryaw):
            quaternion_path = tf.transformations.quaternion_from_euler(0, 0, iyaw, axes='sxyz')
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.pose.position.x = float(ix)
            pose.pose.position.y = float(iy)
            pose.pose.position.z = 0.0  # Debug: float(iyaw/math.pi*180)
            pose.pose.orientation.x = float(quaternion_path[0])
            pose.pose.orientation.y = float(quaternion_path[1])
            pose.pose.orientation.z = float(quaternion_path[2])
            pose.pose.orientation.w = float(quaternion_path[3])
            pose.header.seq = self.path.header.seq + 1
            self.path.header.frame_id = "world"
            self.path.header.stamp = rospy.Time.now()
            pose.header.stamp = self.path.header.stamp
            self.path.poses.append(pose)
            self.pub_global_path.publish(self.path)

        self.path = Path()

    def tf_cb(self, event):
        # Send transform between /world and /map
        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, 0),
                         (0, 0, 0, 1),
                         rospy.Time.now(),
                         "/map",
                         "/world")


def main():
    # Init global_path_planner node.
    rospy.init_node("global_path_planner")

    # Create global path planner object
    gpp = GlobalPathPlanner()

    # Read static map
    rospy.wait_for_service('static_map')
    try:
        map_server = rospy.ServiceProxy('static_map', GetMap)
        gpp.map = map_server().map
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    # Timer to refresh TF transform between world and map
    rospy.Timer(rospy.Duration(1.0/4), gpp.tf_cb)

    # Subscribe to goal topic published by RVIZ
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, gpp.goal_cb)

    # Blocks until ROS node is shutdown.
    rospy.spin()


if __name__ == "__main__":
    main()