#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from tf.transformations import euler_from_quaternion
from copy import deepcopy

import numpy as np


'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS  = 50     # Number of waypoints we will publish. You can change this number
MIN_ZERO_WP    = 5      # Need a few zero WP to allow the vehilce to stop in time
MAX_ACCEL      = 1.5    # Acceleration/Deceleration m/s/s

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # TODO: Add other member variables you need below
        # Save all waypoints
        self.waypoints = None
        self.max_waypoint_index = None
        # Save previous pose and waypoint
        self.prev_pose = None
        self.prev_wp = None

        # Save next red traffic light waypoint
        self.red_tl_wp = None
        self.speed = 0.0

        #rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        #rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        if (self.waypoints is None):
            rospy.logwarn('Waypoints not received before pose update')
            return

        if (self.prev_pose == msg.pose):
            next_wp = self.prev_wp
            rospy.loginfo('Pose unchanged!')
        else:
            # Pose update received -> get next waypoint
            next_wp = self.next_waypoint(self.waypoints, msg.pose)
            rospy.loginfo('Current pose ({}, {}) next waypoint ({}, {})'.format(msg.pose.position.x,
                                                                                msg.pose.position.y,
                                                                                self.waypoints[next_wp].pose.pose.position.x,
                                                                                self.waypoints[next_wp].pose.pose.position.y))

            # Save next waypoint for next iteration if pose is not updated
            self.prev_wp = next_wp

            # Save current pose to check if it got updated on the next iteration
            self.prev_pose = msg.pose


        # Init falg to slow down vehicle
        enable_slow_down = False

        # Get the final waypoints from current lane
        final_waypoints = Lane()

        # Get final waypoint number
        final_wp = next_wp + LOOKAHEAD_WPS

        # Check if a red TL is in the upcoming waypoints
        if self.red_tl_wp is not None and self.red_tl_wp <= final_wp:
            # Last waypoint is either the max number of lookahead waypoints or the red TL waypoint
            final_wp = self.red_tl_wp
            if self.red_tl_wp >= next_wp:
                # Red TL in our way so slow down vehicle
                enable_slow_down = True

        # Copy waypoints in range - use deep copy as we don't want to change the original waypoints!
        waypoint_index = next_wp
        for i in range(waypoint_index, final_wp):
            final_waypoints.waypoints.append(deepcopy(self.waypoints[waypoint_index]))
            # Handle wrapping within waypoint list
            if waypoint_index == self.max_waypoint_index:
                waypoint_index = 0
            else:
                waypoint_index += 1


        steps = final_wp - next_wp
        indexes = range(steps)

        # Slow down vehicle by very simple step function
        if enable_slow_down:
            # want to start at the traffic light and work way back to current speed
            indexes.reverse()
            self.speed = 0

            prev_waypoint = None
            zero_speed_count = 0
            for i in indexes:
                waypoint = final_waypoints.waypoints[i]

                # distance between waypoints 
                if prev_waypoint is None:
                    pass
                elif zero_speed_count < MIN_ZERO_WP:
                    # set the waypoints near the TL to zero
                    # need a few to make the car stop before the TL
                    zero_speed_count += 1
                elif self.speed < self.target_velocity:
                    self.speed = self.Accelerate(waypoint, prev_waypoint, self.speed, self.target_velocity, MAX_ACCEL);

                self.set_waypoint_velocity(final_waypoints.waypoints, i, self.speed)
                prev_waypoint = waypoint
        else:
            # either speed up or coast
            prev_waypoint = None

            for i in indexes:
                waypoint = final_waypoints.waypoints[i]
                if prev_waypoint is None:
                    pass
                elif self.speed < self.target_velocity:
                    self.speed = self.Accelerate(waypoint, prev_waypoint, self.speed, self.target_velocity, MAX_ACCEL);
                else:
                    # we have reached the target velocity so no more accelerating to do
                    break

                self.set_waypoint_velocity(final_waypoints.waypoints, i, self.speed)
                prev_waypoint = waypoint


        # Publish the complete waypoint list
        self.final_waypoints_pub.publish(final_waypoints)

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # Waypoints need to be received just once!
        if self.waypoints is None:
            self.waypoints = waypoints.waypoints
            self.max_waypoint_index = len(self.waypoints) - 1
            rospy.loginfo('Waypoints received!')
            self.base_waypoints_sub.unregister()

            self.target_velocity = 0.0
            for i in range(len(self.waypoints)):
                self.target_velocity  = max(self.target_velocity, self.get_waypoint_velocity(self.waypoints[i]))

            # Set queue sizes to 1 to process only the latest message.
            # If it's not 1 slower PCs are getting behind and the car drive off.
            rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
            rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)

    def closest_waypoint(self, waypoints, pose):
        # Init vars to capture closest waypoint
        closest_wp = 0
        closest_dist = 10000

        # Find closest waypoint to current pose by checking distance to all waypoints
        for i in range(len(waypoints)):
            dist = self.distance(waypoints[i].pose.pose.position,
                                 pose.position)

            # Compare distance and update if new minimum has been found
            if (dist < closest_dist):
                closest_wp = i
                closest_dist = dist

        # Return the found waypoint
        return closest_wp

    def next_waypoint(self, waypoints, pose):
        # Find the closest waypoint
        closest_wp = self.closest_waypoint(waypoints, pose)

        # We don't know yet, if the closest waypoint is ahead or behind.
        # Therefore, we have to calculate the heading of the car and compare it to the car's yaw!
        closest_waypoint_x = waypoints[closest_wp].pose.pose.position.x
        closest_waypoint_y = waypoints[closest_wp].pose.pose.position.y
        pose_x = pose.position.x
        pose_y = pose.position.y
        heading = math.atan2(closest_waypoint_y - pose_y,
                             closest_waypoint_x - pose_x)

        # We can the following transformation to converr from quaternions provided by
        # an odometry message to Euler angles (roll, pitch and yaw).
        # Check: http://www.theconstructsim.com/ros-qa-convert-quaternions-euler-angles
        (roll, pitch, yaw) = euler_from_quaternion(np.array([pose.orientation.x,
                                                             pose.orientation.y,
                                                             pose.orientation.z,
                                                             pose.orientation.w]))

        # Now we can check if the closest waypoint is ahead or behind
        if (abs(yaw - heading)) > (math.pi / 4):
            # Waypoint is behind so use the next one
            closest_wp += 1
            if closest_wp > self.max_waypoint_index:
                # Next waypoint is in next lap so reset index
                closest_wp = 0

        # Return the closest waypoint ahead
        return closest_wp

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        if msg.data != -1:
            self.red_tl_wp = msg.data
        else:
            self.red_tl_wp = None



    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        dist = dl(wp1, wp2)
        #for i in range(wp1, wp2+1):
        #    dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
        #    wp1 = i
        return dist

    def Accelerate(self, waypoint, prev_waypoint, speed, target_speed, max_accel):
        # Calculate the speed at each waypoint taking into account the allowable acceleration/deceleration
        distance = self.distance(waypoint.pose.pose.position, prev_waypoint.pose.pose.position)
        # update next speed based on MAX Acceleration/Deceleration and d = vt + 0.5at**2
        time_to_waypoint = 2*distance/(speed + math.sqrt(speed**2 + 2*max_accel*distance))
        speed = speed + max_accel*time_to_waypoint
        if speed > target_speed:
            speed = target_speed
        return speed


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
