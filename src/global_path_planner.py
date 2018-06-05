#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped

import matplotlib.pyplot as plt
import a_star

show_animation = True


class GlobalPathPlanner:
    def __init__(self):
        self.map = OccupancyGrid()
        self.pub_global_path = rospy.Publisher('/robot0/global_path', Path, queue_size=1)
        self.path = Path()

    def goal_cb(self, goal):
        rospy.loginfo("Goal set to x = {0}, y = {1})"
                      .format(str(goal.pose.position.x), str(goal.pose.position.y)))

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

        if show_animation:
            plt.plot(rx, ry, "-r")
            plt.show()

        for ix, iy in zip(rx, ry):
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.pose.position.x = float(ix)
            pose.pose.position.y = float(iy)
            pose.header.seq = self.path.header.seq + 1
            self.path.header.frame_id = "world"
            self.path.header.stamp = rospy.Time.now()
            pose.header.stamp = self.path.header.stamp
            self.path.poses.append(pose)

            self.pub_global_path.publish(self.path)


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

    # Timer to refresh TF transform
    rospy.Timer(rospy.Duration(1.0/4), gpp.tf_cb)

    # Subscribe to goal topic published by RVIZ
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, gpp.goal_cb)

    # Blocks until ROS node is shutdown.
    rospy.spin()


if __name__ == "__main__":
    main()