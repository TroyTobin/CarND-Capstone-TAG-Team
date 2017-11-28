
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

import rospy

from  yaw_controller import YawController
from pid import PID

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        # Init yaw controller with values from DBW node init
        self.yaw_controller = YawController(args[0],    # wheel_base
                                            args[1],    # steer_ratio
                                            3 * ONE_MPH,    # min_speed
                                            args[2],    # max_lat_accel
                                            args[3])    # max_steer_angle

        # Init throttle controller with values from DBW node init
        self.throttle_controller = PID(0.3,     # kp
                                       0.0,     # ki
                                       0.001,   # kd
                                       args[4], # decel_limit
                                       args[5]) # accel_limit

        # Set max brake torque = (vehicle_mass + fuel_capacity * GAS_DENSITY) *
        #                         decel_limit * wheel_radius)
        self.max_brake_torque = ((args[6] + (args[7] * GAS_DENSITY)) *
                                 args[4] * args[8])

        # Init time stamp for sample time calculation
        self.prev_time = None

    def control(self, *args, **kwargs):
        # Re-assign arguments for readability
        proposed_velocity_linear_x  = args[0]
        proposed_velocity_angular_z = args[1]
        current_velocity_linear_x   = args[2]
        dbw_enabled                 = args[3]

        # Init return values that may be published
        throttle = 0.0
        brake    = 0.0
        steering = 0.0

        # Compute difference between target and current velocity as CTE for throttle. 
        velocity_error = proposed_velocity_linear_x - current_velocity_linear_x

        # Get current time in ROS
        current_time = rospy.get_time()

        if dbw_enabled:
            if self.prev_time is not None and current_time != self.prev_time:
                if proposed_velocity_linear_x == 0 and current_velocity_linear_x < 0.01:
                        self.throttle_controller.reset()
                        #rospy.logwarn("Reset PID")
                else:
                    # Calculate time between samples
                    sample_time = current_time - self.prev_time

                    # Get velocity adjustment from throttle controller
                    velocity_adjust = self.throttle_controller.step(velocity_error,
                                                                    sample_time)

                    # Set throttle or brake according to sign of velocity adjustment
                    if velocity_adjust > 0:
                        throttle = velocity_adjust
                        if velocity_error < 0:
                            # When the error is negative, we're too fast
                            # so we shouldn't accelerate!
                            throttle = 0.0
                            brake = velocity_adjust * self.max_brake_torque
                    else:
                        brake = velocity_adjust * self.max_brake_torque

                    # Get steering from yaw controller
                    steering = self.yaw_controller.get_steering(proposed_velocity_linear_x,
                                                                proposed_velocity_angular_z,
                                                                current_velocity_linear_x)

                    # rospy.logwarn('prop {:7.2f}, curr {:7.2f} err {:7.2f}, adj {:7.2f}, throttle {:7.2f}, brake {:7.2f}'.format(
                    #               proposed_velocity_linear_x,
                    #               current_velocity_linear_x,
                    #               velocity_error,
                    #               velocity_adjust,
                    #               throttle,
                    #               brake))

        # Save current time for next iteration
        self.prev_time = current_time

        return throttle, brake, steering