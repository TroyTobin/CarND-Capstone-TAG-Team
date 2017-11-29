#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane, Waypoint
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from collections import namedtuple
import tf
import cv2
import yaml
import math


# Classifier has to return this number of times the same state in order to be used
STATE_COUNT_THRESHOLD = 2


# Create a structure similar to hwat is used in styx msg so TL stop_line_positions
# can be represented in the same format
position = namedtuple('position', 'x y z')
pose_token = namedtuple('pose_token', ['position'])

# For debug prints, map TL state to tex
tl_state_text = ['red', 'yellow', 'green', 'undef', 'unknown']

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        # Save all waypoints
        self.waypoints = None

        # Save traffic lights/stop line positions as waypoints
        self.tl_waypoints= []

        #sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        #sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        #sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # Waypoints need to be received just once!
        if self.waypoints is None:
            # Save waypoints
            self.waypoints = waypoints.waypoints

            # List of positions that correspond to the line to stop in front of for a given intersection
            stop_line_positions = self.config['stop_line_positions']

            # Map all TLs to a styx msg structure
            tl_pose = []
            for i in range(len(stop_line_positions)):
                tl_point = pose_token(position(stop_line_positions[i][0],  # x
                                               stop_line_positions[i][1],  # y
                                               0))                         # z
                tl_pose.append(tl_point)

            # Map TL's stop line positions to the closest waypoint
            for i in range(len(stop_line_positions)):
                self.tl_waypoints.append(self.get_closest_waypoint(self.waypoints, tl_pose[i]))

            # print 'tl waypoint {}, total {}'.format(i, len(self.waypoints))
            # print self.tl_waypoints

            # Waypoints are received and are not updated again
            self.sub2.unregister()

            # start all subscribers
            rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
            rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
            rospy.Subscriber('/image_color', Image, self.image_cb)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        # Process image if waypoints have been received
        if self.waypoints is not None and self.tl_waypoints is not None:
            self.has_image = True
            self.camera_image = msg
            light_wp, state = self.process_traffic_lights()

            '''
            Publish upcoming red lights at camera frequency.
            Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
            of times till we start using it. Otherwise the previous stable state is
            used.
            '''
            if self.state != state:
                self.state_count = 0
                self.state = state
            elif self.state_count >= STATE_COUNT_THRESHOLD:
                self.last_state = self.state
                # To be on the safe side stop the car also on yellow
                light_wp = light_wp if state in [TrafficLight.RED, TrafficLight.YELLOW] else -1
                self.last_wp = light_wp
                self.upcoming_red_light_pub.publish(Int32(light_wp))
            else:
                self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            self.state_count += 1

    def get_closest_waypoint(self, waypoints, pose):
        #
        # Same function as in waypoint_updater.py (could be outsourced as common code).
        #
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement

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

    def distance(self, wp1, wp2):
        #
        # Same function as in waypoint_updater.py (could be outsourced as common code).
        #
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        dist = dl(wp1, wp2)
        return dist

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # For sim only - use ground truth data
        return light.state

        # For classifier use
        # if(not self.has_image):
        #     self.prev_light_loc = None
        #     return False

        # cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # #Get classification
        # return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        # --> moved to waypoints_cb()
        # stop_line_positions = self.config['stop_line_positions']

        if(self.pose):
            car_position = self.get_closest_waypoint(self.waypoints, self.pose.pose)

            #TODO find the closest visible traffic light (if one exists)
            # Find closest TL waypoint to car's current waypoint.
            # 1. Calculate distances towards all TLs
            dist_tl_car = []
            for i in range(len(self.tl_waypoints)):
                dist_tl_car.append(self.tl_waypoints[i] - car_position)

            # 2. Check if the max distance is negative. This means we're at the
            # end of ther waypoint list and the next TL is the first one in the
            # waypoint list
            max_tl_dist = max(i for i in dist_tl_car if i != 0)
            if max_tl_dist < 0:
                 # We're at the end of ther waypoint list and the next TL
                 # is the first one in the
                next_tl_id = 0
            else:
                # Next TL is the one with the min distance
                closest_dist = min(i for i in dist_tl_car if i > 0)
                next_tl_id = dist_tl_car.index(closest_dist)

            # 3. Use min distance to get TL waypoint
            next_tl_wp = self.tl_waypoints[next_tl_id]

            # For sim only - use ground truth data
            light = self.lights[next_tl_id]
            # All TL wp [292, 753, 2047, 2580, 6294, 7008, 8540, 9733]
            #print 'Car wp {}, next TL wp {}, state {}'.format(car_position, next_tl_wp, tl_state_text[light.state])

        if light:
            state = self.get_light_state(light)
            return next_tl_wp, state

        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
