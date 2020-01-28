#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import yaml

import math
MAX_DECEL = 1.0
FREQUENCY = 30 #50 Hz for Carla, 10Hz for desktop
LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number


'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

'''


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below from self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        config_string = rospy.get_param("/params_config")
        self.config = yaml.load(config_string)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_lane = None
        self.stopline_wp_idx = -1
        #self.base_waypoints = None #?
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.loop()   # //rospy.spin()

    def loop(self):
        waypoint_updater_frequency = self.config["waypoints_updater"]["frequency"]
        rate = rospy.Rate(waypoint_updater_frequency)##updated from 50 to now be variable
        while not rospy.is_shutdown():
            if self.pose and self.base_lane and self.stopline_wp_idx:               # added to ensure three messages are brought in from subscritopns
                self.publish_waypoints()
            rate.sleep()
        
    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
      
        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]
        
        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])
        
        val = np.dot(cl_vect - prev_vect , pos_vect - cl_vect)
        
        if val>0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx
    
    def publish_waypoints(self):
        final_lane = self.generate_lane()        
        self.final_waypoints_pub.publish(final_lane)
    
    def generate_lane(self):
        lane = Lane()                                                               # create new lane object for publishing
        closest_idx = self.get_closest_waypoint_idx()                               # get the waypoint index for the closest waypoint in front 
        farthest_idx = closest_idx + LOOKAHEAD_WPS                                  # build new waypoint size of size [WPS]
        # farthest_idx = closest_idx + (LOOKAHEAD_WPS/2)                                  # build new waypoint size of size [WPS]
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]         # build array of base_waypoints WPS in size from input
        
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):    # if there is no stoplight in range ahead or it is permissive state leave waypoints as is
            lane.waypoints = base_waypoints                                         # populate lane with base_waypoints (don't overwrite base_waypoints)
            # rospy.logwarn("no red light detected") #rospy.logwarn("Filtered Velocity: {0}" .format(self.vel_lpf.get())) # format for data

        else:
            # rospy.logwarn("decelerate to 0") #rospy.logwarn("Filtered Velocity: {0}" .format(self.vel_lpf.get())) # format for data
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx) # call decelerate_waypoints function to ramp speed to zero
        return lane
            
    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []                                                           # Create temporary list of waypoints
        for i, wp in enumerate(waypoints):                                  # for all waypoints in the waypoints list to be decelerated
            p = Waypoint()                                                  # instantiate a new Waypoint() message object
            p.pose = wp.pose                                                # copy the position over
            
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2 , 0)      # Set the stop index of light (-2 waypoints to account for vehiclecenter line to nose), ie not 2.0 meters
            dist = self.distance(waypoints, i, stop_idx)                    # calculated the distance from the current iteration waypoint to the traffic light stop position
            vel = math.sqrt(2 * MAX_DECEL * dist)                           # mapping a velocity target, should shape this more smoothly should consider starting at stop light / reinitializing
            if vel <1.0:                                                    # if target velocity is less than 0, make 0
                vel = 0.0
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)      # setup speed to be the speed limit or the lower braking speed
            temp.append(p)
        return temp
    
    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_lane = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            rospy.loginfo('Constructing waypoint tree')
            self.waypoint_tree = KDTree(self.waypoints_2d)            
  
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
