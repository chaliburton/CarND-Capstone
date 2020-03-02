from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
from params_config.params_config import ParamsConfig
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
# Brought arguments in from dbw_node definition for declaration of input parameters in __init__ function
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius,
                 wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        
# Yaw controller is provided, in yaw_controller.py, but need to pass it the arguments it expects
#                                      ---> wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        
        self.config = ParamsConfig.getInstance().getConfig()
        if self.config is not None:
            rospy.logwarn("twist_controller: custom parameters used")
            mx = self.config["twist_controller"]["mx"]
        else:
            if ParamsConfig.isSite():
                # running on Carla
                mx = 0.2
            else:
                # running in sumulator
                mx = 0.2 # maximum throttle value
# Setup parameters for throttle controller        
        kp = 0.4
        ki = 0.2
        kd = 0.3
        mn = 0.0 # minimum throttle value
        # Brake PID
        b_kp = 200.0
        b_ki = 0.1
        b_kd = 5.0
        b_mn = 0.0 # minimum brake value
        b_mx = 3250.0 # maximum brake value
# Initialize throttle controller by passing values to pid.py -> def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
# important to note function for resetting integral error to zero: def reset(self):
        self.throttle_controller = PID(kp, ki, kd, mn, mx)
        self.brake_controller = PID(b_kp, b_ki, b_kd, b_mn, b_mx)

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
        self.last_vel = 0.0
        self.last_brake = 0.0

# define controller function using inputs, called in dbw_node.py (if updated need to modify in both )
    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
# setup such that if DBW node is disabled that the controller is not active and return all 0's
        if not dbw_enabled:
            self.throttle_controller.reset()
            self.brake_controller.reset()
            return 0.0 , 0.0 , 0.0
        current_vel = self.vel_lpf.filt(current_vel)
        
# code walkthrough has # rospy.logwarn outputs to check velocity
        #rospy.logwarn("Angular vel: {0}" .format(angular_vel))
        #rospy.logwarn("Target Velocity: {0}" .format(linear_vel))
#        rospy.logwarn("Target Angular Velocity: {0}\n" .format(angular_vel))
        #rospy.logwarn("Current Velocity: {0}" .format(current_vel))
        #rospy.logwarn("Filtered Velocity: {0}" .format(self.vel_lpf.get()))
        
        

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
        brake = 0.0
    
        if linear_vel < 0.4 and current_vel < 5.0:
            self.brake_controller.reset()
            self.throttle_controller.reset()
            throttle = 0.0
            brake = min(self.last_brake+30,700) #N*m - to hold the car in place 
        elif throttle < 0.1 and vel_error < 0.0:
            throttle = 0.0
            decel = max(vel_error, self.decel_limit)
            brake = self.brake_controller.step(-decel, sample_time)
            #brake = abs(decel)*self.vehicle_mass*self.wheel_radius # Torque in N*m
        self.last_brake = brake
        # rospy.logwarn("steering Angle is: {0}" . format(steering))
        
        return throttle, brake, steering
