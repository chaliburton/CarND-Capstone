from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
# Brought arguments in from dbw_node definition for declaration of input parameters in __init__ function
    def __init__(self,vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius,
                 wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        
# Yaw controller is provided, in yaw_controller.py, but need to pass it the arguments it expects
#                                      ---> wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

    
# Setup parameters for throttle controller        
        kp = 0.3
        ki = 0.1
        kd = 0.0
        mn = 0.0 # minimum throttle value
        mx = 0.2 # maximum throttle value
# Initialize throttle controller by passing values to pid.py -> def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
# important to note function for resetting integral error to zero: def reset(self):
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

# Initialize instance of low pass filter __init__(self, tau, ts):
        tau = 0.5 # 1 / 2pi*tau = cutoff frequency 1 /pi hz
        ts = 0.02 #sample time set by program /  
        self.vel_lpf = LowPassFilter(tau, ts)

# assign other parameters to variables to avoid warning of unused variables upon compilation??
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.last_time = rospy.get_time()

# define controller function using inputs, called in dbw_node.py (if updated need to modify in both )
    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
# setup such that if DBW node is disabled that the controller is not active and return all 0's
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0 , 0.0 , 0.0
        current_vel = self.vel_lpf.filt(current_vel)
        
# code walkthrough has # rospy.logwarn outputs to check velocity

# update steering controller
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel
# confirm this timestamp matches the data, shouldn't the latency of calling this function may result in undesired behaviour
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
# Confirm this controller is what we want to implement, can play here with getting the brake and throttle setup well
        
        throttle = self.throttle_controller.step(vel_error, sample_time)
# Will most likely need to update the brake based on future inputs (lights, obstacles etc)
# note at 10:00 there is a discussion about the outerware wrapper not updating fast enough and the waypoint follower code might need to be updated more frequently (currently might wait 
# until the waypoint is passed to adjust target angle  check this 
        brake = 0
    
        if linear_vel == 0.0 and current_vel < 0.1:
            throttle = 0
            brake = 400 #N*m - to hold the car in place 
        elif throttle < 0.1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius # Torque in N*m
        
        return throttle, brake, steering
