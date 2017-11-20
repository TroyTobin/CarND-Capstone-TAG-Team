
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
                                            0.0,        # min_speed
                                            args[2],    # max_lat_accel
                                            args[3])    # max_steer_angle

        # Init throttle controller with values from DBW node init
        self.throttle_controller = PID(0.2,     # kp
                                       0.02,    # ki
                                       0.1,     # kd
                                       args[4], # decel_limit
                                       args[5]) # accel_limit

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
                # Calculate time between samples
                sample_time = current_time - self.prev_time

                # Get velocity adjustment from throttle controller
                velocity_adjust = self.throttle_controller.step(velocity_error,
                                                                sample_time)

                # Set throttle or brake according to sign of velocity adjustment
                if velocity_adjust > 0:
                    throttle = abs(velocity_adjust)
                else:
                    brake = abs(velocity_adjust)

                # Get steering from yaw controller
                steering = self.yaw_controller.get_steering(proposed_velocity_linear_x,
                                                            proposed_velocity_angular_z,
                                                            current_velocity_linear_x)

        # Save current time for next iteration
        self.prev_time = current_time

        return throttle, brake, steering