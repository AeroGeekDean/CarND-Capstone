#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32

import math

'''
This node will publish waypoints from the car's current position to some `x`
distance ahead.

As mentioned in the doc, you should ideally first implement a version which does
not care about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of
traffic lights too.

Please note that our simulator also provides the exact location of traffic
lights and their current status in `/vehicle/traffic_lights` message. You can
use this message to build this node as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 30 # Number of wpts we will publish. You can change this number
MAX_DECEL = 0.5 # [m/s^2]

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints',
                                                    Lane, queue_size=1)
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.stopline_wpt_idx = -1

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50 Hz
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints: # self.waypoint_tree instead?
                # TODO: Inline publish_waypoints and generate_lane here for brevity.
                self.publish_waypoints()
            rate.sleep()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoint_tree:
            self.waypoints_2d = [[wpt.pose.pose.position.x, wpt.pose.pose.position.y]
                                 for wpt in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.stopline_wpt_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x,y], 1)[1]

        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        closest_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])

        # use vector dot-product to determine if vehicle has gone past closest wpt
        val = np.dot(closest_vect-prev_vect, pos_vect-closest_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self):
        final_lane_data = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane_data)

    def generate_lane(self):
        lane = Lane() # instantiate a new styx_msgs/Lane object

        lane.header = self.base_waypoints.header # are headers even being used?

        start_idx = self.get_closest_waypoint_idx()
        end_idx = start_idx + LOOKAHEAD_WPS
        # Python slice will just grab to end. Need to worry about start/finish wrap?
        baselane_wpts = self.base_waypoints.waypoints[start_idx:end_idx]

        if self.stopline_wpt_idx == -1 or (self.stopline_wpt_idx >= end_idx):
            lane.waypoints = baselane_wpts
        else:
            lane.waypoints = self.decelerate_wpts(baselane_wpts, start_idx)

        return lane

    def decelerate_wpts(self, wpts, start_idx):
        tmp = []
        for i, wp in enumerate(wpts):
            p = Waypoint()
            p.pose = wp.pose

            # stop 2 wpts behind line to prevent nose of car sticking past
            stop_idx = self.stopline_wpt_idx - start_idx - 2
            stop_idx = max(stop_idx, 0)
            dist = self.distance(wpts, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.0:
                vel = 0.

            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            tmp.append(p)
        return tmp

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position,
                       waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
