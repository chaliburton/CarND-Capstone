#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
from scipy.spatial import KDTree
import yaml

STATE_COUNT_THRESHOLD = 1

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.camera_image = None
        self.camera_image_count = 0
        self.lights = []
        self.inc = 0
        self.dist = 500
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        #sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)


        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier(self.config['is_site'])
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.has_image = False
        self.image_save = False
        self.loop()
           
    def loop(self):
        #tl_node_rate = rospy.Rate(1.0)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            #rospy.logwarn("Calling TL Detector Function (and save function if desired)... ")
            if self.pose_cb and self.waypoint_tree and self.lights: ##and self.camera_image:               # added to ensure three messages are brought in from subscritopns
                self.process_traffic_lights()
                if self.image_save:
                    self.save_image()
            rate.sleep()
        

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            #rospy.loginfo('Constructing waypoint tree')
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        self.camera_image_count = 0
        self.has_image = True
        self.camera_image = msg
        #rospy.loginfo("tl_detector: IMAGE Available on bus")
        
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint
        Args:
            msg (Image): image from car-mounted camera
        """
        #light_wp, state = self.process_traffic_lights()
        
    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        if self.waypoint_tree is None:
            rospy.logwarn("NO waypoint tree")
            return -1
        closest_idx = self.waypoint_tree.query([x,y],1)[1]
        return closest_idx

    def get_light_state(self, light):
        """  commented out for testing stop/start commands
        Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        if self.image_save or not self.has_image: #change to config file paraam
            try: 
                #rospy.logwarn("Ground Truth light color is: {0}" . format(light.state))
                return light.state
            except:
                return TrafficLight.UNKNOWN            
        if self.has_image:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
                # rospy.logwarn("Ground Truth light color is: {0}" . format(light.state))
                #Get classification
                #rospy.loginfo("tl_detector: get_classification()")
                return self.light_classifier.get_classification(cv_image)
            except:
                return TrafficLight.UNKNOWN

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        line_wp_idx = None
        stop_line_positions = self.config['stop_line_positions']
        #rospy.loginfo("tl_detector: process_traffic_lights()")

        if(self.pose):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x , self.pose.pose.position.y)
            if car_wp_idx == -1:
                return -1, TrafficLight.UNKNOWN

            #rospy.logwarn("         Closest Waypoint is: {0}" .format(car_wp_idx))

        #TODO find the closest visible traffic light (if one exists)
            if self.waypoints is None:
                rospy.logwarn("tl_detector: Couldn't process traffic lights due to unavailability of waypoints")
                return -1, TrafficLight.UNKNOWN
            diff = len(self.waypoints.waypoints)*2
            for i, light in enumerate(self.lights):
                # Get stop line waypoint index
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                # Find closest stop line waypoint index
                d = temp_wp_idx - car_wp_idx
                #rospy.loginfo("tl_detector: Stoplight idx: {}, diff: {}".format(d, diff))
                if d >= -2 and d < diff:
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx
                    self.dist = d
            #rospy.logwarn("         Closest Light Waypoint is: {0}" .format(line_wp_idx))

        if closest_light:
            state = self.get_light_state(closest_light)
            #rospy.logwarn("                                       actual state raw: {} previous state:{} count :{}".format(state, self.state, self.state_count))
            #rospy.logwarn("   {}".format(state == TrafficLight.RED))
            if self.state != state:
                self.state_count = 0
                self.state = state
            if self.state_count >= STATE_COUNT_THRESHOLD:
                self.last_state = self.state
                line_wp_idx = line_wp_idx if (state == TrafficLight.YELLOW or state == TrafficLight.RED) else -1
                self.last_wp = line_wp_idx
                self.upcoming_red_light_pub.publish(Int32(line_wp_idx))
                #rospy.logwarn("                                                               A Stop Index is:{}".format(line_wp_idx))

            else:
                self.upcoming_red_light_pub.publish(Int32(self.last_wp))
                #rospy.logwarn("                                                               B Stop Index is:{}".format(self.last_wp))
                self.state_count += 1  

            return line_wp_idx, state
        return -1, TrafficLight.UNKNOWN


    def save_image(self): # Function for saving an image
        light_wp, self.state = self.process_traffic_lights()
        if self.camera_image and self.dist < 500:
            try:
                # Convert your ROS Image message to OpenCV2                
                path = 'images/'
                if not os.path.isdir(path):
                    os.mkdir(path)
                cv2_img = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
            except CvBridgeError, e:
                print(e)
            else:

                # Save your OpenCV2 image as a jpeg                     state, '_', self.inc,'_',
                full_path = os.path.join(path,str(self.state)+'_' + str(self.inc)+ 'img.jpeg')

                rospy.logwarn("PATH IS:    {0}" . format(full_path))
                cv2.imwrite(full_path, cv2_img)
                self.inc += 1         
        
if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
