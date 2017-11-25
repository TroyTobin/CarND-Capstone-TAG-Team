#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node', log_level=rospy.INFO)

        vehicle_mass    = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity   = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband  = rospy.get_param('~brake_deadband', .1)
        decel_limit     = rospy.get_param('~decel_limit', -5)
        accel_limit     = rospy.get_param('~accel_limit', 1.)
        wheel_radius    = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base      = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio     = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel   = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub    = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub    = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        # Init control parameters retrieved from topics
        self.dbw_enabled       = True # From topic /vehicle/dbw_enabled
        self.proposed_velocity = None # From topic /twist_cmd
        self.current_velocity  = None # From topic /current_velocity

        # TODO: Create `TwistController` object 
        # self.controller = TwistController(<Arguments you wish to provide>)

        # YawController requires wheel_base, steer_ratio, max_lat_accel and max_steer_angle
        # PID requires decel_limit and accel_limit
        self.controller = Controller(wheel_base,
                                     steer_ratio,
                                     max_lat_accel,
                                     max_steer_angle,
                                     decel_limit,
                                     accel_limit,
                                     vehicle_mass,
                                     fuel_capacity,
                                     wheel_radius)

        # TODO: Subscribe to all the topics you need to 
        rospy.Subscriber("/current_velocity", TwistStamped, self.current_velocity_cb, queue_size=1)
        rospy.Subscriber("/twist_cmd", TwistStamped, self.twist_cb)
        rospy.Subscriber("/vehicle/dbw_enabled", Bool, self.dbw_enable_cb)

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller` 
            # You should only publish the control commands if dbw is enabled 
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>, 
            #                                                     <proposed angular velocity>, 
            #                                                     <current linear velocity>, 
            #                                                     <dbw status>, 
            #                                                     <any other argument you need>) 
            # if <dbw is enabled>: 
            #   self.publish(throttle, brake, steer)

            # Use controller only if updated data from topics has been received
            if self.proposed_velocity is not None and self.current_velocity is not None:
                throttle, brake, steering = self.controller.control(self.proposed_velocity.linear.x,
                                                                    self.proposed_velocity.angular.z,
                                                                    self.current_velocity.linear.x,
                                                                    self.dbw_enabled)

                # rospy.logwarn('trottle {:7.2f}, brake {:7.2f}, prop {:7.2f}, curr {:7.2f}'.format(
                #               throttle,
                #               brake,
                #               self.proposed_velocity.linear.x,
                #               self.current_velocity.linear.x))

                if self.dbw_enabled:
                    # Publish new values
                    self.publish(throttle, brake, steering)

            # Wait for the next iteration
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        #rospy.loginfo('Throttle: {}'.format(throttle))
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        #rospy.loginfo('Steer: {}'.format(steer))
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        #rospy.loginfo('Brake: {}'.format(brake))
        self.brake_pub.publish(bcmd)

    def dbw_enable_cb(self, msg):
        self.dbw_enabled = bool(msg.data)

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist

    def twist_cb(self, msg):
        self.proposed_velocity = msg.twist


if __name__ == '__main__':
    DBWNode()
