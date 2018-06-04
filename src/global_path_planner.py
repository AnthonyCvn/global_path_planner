#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped


class GlobalPathPlanner:
    def __init__(self):
        self.map = OccupancyGrid()

    def goal_cb(self, goal):
        x_goal = goal.pose.position.x
        y_goal = goal.pose.position.y
        rospy.loginfo("Goal set to x = {0}, y = {1})"
                      .format(str(x_goal), str(y_goal)))

        
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

    # Receive goal from RVIZ
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, gpp.goal_cb)

    # Blocks until ROS node is shutdown.
    rospy.spin()


if __name__ == "__main__":
    main()