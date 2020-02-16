This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car.

## Team Members

Our team consits of the following members:
* Chris Haliburton (chris.haliburton@gmail.com) -- team lead
* Søren Rusbjerg (rusbjerg.consulting@gmail.com)
* Todor Ristov (ristovtodor@gmail.com)

[//]: # (Image References)

[system_architecture]: ./system_architecture.png "system_architecture"
[Simulator_prediction]: ./TrainingScript/ssd_keras/examples/SimulatorPrediction.png "Simulator prediction"
[Carla_prediction]: ./TrainingScript/ssd_keras/examples/CarlaPrediction.png "Carla prediction"

## Programming a Real Self-Driving Car
The goal of the project is to program a car that can drive autonomosly, and detect and obey traffic lights, in a constrained real-world environment (parking lot). The underlying platform is ROS and all of the core functionality of the autonomous vehicle system is implemented as ROS nodes. The code is first tested on a simulator by driving around on a fixed track with traffic lights. Once the vehicle drives successfully in the simulator, the code can be loaded on Udacity's self-driving test vehicle, Carla, and tested in the parking lot.

In the following sections we give a brief system architecture overview and describe the system components we've implemented.

### System Architecture
The following system architecture diagram shows a good high-level overview of the system components involved and how they interact with each other.

![alt text][system_architecture]

### Planning
#### Waypoint Loader Node
The Waypoint Loader Node reads in a set of waypoints on the map and creates a vector of waypoints for the vehicle to follow.  These waypoints are comprised of an x,y and z position, an orientation for the vehicle to be in (yaw) as well as a velocity target to be achieved.  These base waypoints are published so that the vehicle can make decisions based on it's perceived surroundings.

As arcitected the Obstacle Detection Node receives image data, current position on the map and base waypoints.  It could be set up to detect obstacles, modify the waypoints to change lanes, speed up, slow down or stop in order to avoid these obstacles.  The Traffic Light Detection Node receives image, position and base waypoints as well and a traffic light classifier was implemented in order to react to the detected traffic light state accordingly. Both the Obstacle Detection Node and Traffic Light Detection Node each return a waypoint which represent a modifier that indicates where an obstacle or traffic light is.  The Waypoint Updater Node then takes action on this information.

#### Waypoint Updater Node
The Waypoint Updater Node takes in the base waypoints, current position and a traffic light waypoint.  The obstacle waypoint is not implemented for this project but could be implemented in a similar fashion.
The traffic light waypoint, traffic_waypoint, is broadcast and received as a waypoint at which the vehicle should stop when the Traffic Light Classifier identifies a red light.  If the state of the light is yellow or green the waypoint is set to a value which the vehicle does not consume (-1).  If the state of the light is red, the waypoint is set to a waypoint index in advance of the light to effect a safe stopping position (vehicle bumper on stop line, clear from intersection).  The waypoint speed is set to zero and a function is implemented to create a target speed between the vehicle's current waypoint and the stop waypoint.  This creates a ramp down in speed.  The brake controller is activated and the throttle command goes to zero and the PID is reset when the speed desired is below the actual speed by a target threshold value.  When the vehicle comes to a stop the brake pressure is set to a 700N-m threshold to overcome torque creep of the automatic transmission.  When the light changes state the waypoint stop index is cleared and the vehicle resets the brake PID controller and enables the throttle controller.  The throttle controller then targets a speed which is unmodified from the original base_waypoints signal.

### Perception
#### Obstacle Detection Node
Obstacle detection is outside the scope of this project as there are no obstacles in the form of cars, bicycles, people or other objects in the simulator or real worl parking lot. Rather we use the traffic light coordinates given to us in the yaml files *stop_line_positions* to generate stop line indices where the car will stop when the traffic light is red as determined by the Traffic Light Detection Node.


#### Traffic light Detection Node
The traffic lights are classified into **Red**, **Yellow**, **Green** and **Unknown** by the classifier. The classifier consist of a SSD (Single Shot Detector) network which is borrowed from  [Pierluigi Ferrari: SSD-Keras](https://github.com/pierluigiferrari/ssd_keras) Github. We have used his 7-layer SSD architecture (SSD-7), which is very fast, yet still capable of giving a good classification score on the traffic lights in the simulator and in Carla. Luigi recorded a speed of 127 frames per second using a Geforce GTX 1070 (mid-tear graffic card). 
Originally the classifier outputs 5 classes ('car', 'truck', 'pedestrian', 'bicyclist', 'light'), but we have retrained it to only predict the three traffic light states as well as the 'unknown' which covers when there are no traffic lights. The bounding boxes have also been trained, but are not used during classification. Here only the top score, ie. best prediction is used to get the classification. 

The SSD-7 network weights are retrained using two small datasets of a couple of houndred images from the simulator and from Carla. After training on each datasets the weights were saved, so we have a weight set for the simulator and one for Carla. 
The decision for which weights to use is done using the **yaml** files (*site_traffic_light_config.yaml* or *site_traffic_light_config.yaml*) together with the **is_site** parameter. 
All the relevant Python scripts for training the net work is found [here](/TrainingScript/ssd_keras).
We have used the script [training](/TrainingScript/ssd_keras/ssd7_training.ipynb) for training the network, and [inference](/TrainingScript/ssd_keras/ssd7_inference.ipynb) for testing the network.

Examples of prediction on the simulator and Carla can be seen below:
![alt text][Simulator_prediction]
![alt text][Carla_prediction]

### Control
#### DBW Node
The DBW Node implements the drive-by-wire controllers necessary to enable the vehicle to follow the waypoints.  A steering command from -1 to 1, throttle command signal from 0-1 and brake command in N-m are broadcast. This node handles messaaging Input/Output between the ROS nodes and vehicle simulator (by way of bridge.py and server.py).  Current velocity and a boolean, dbw_enabled come into this node from the vehicle (Carla or simulator).  The Waypoint Follower does the bulk of the calculations when the dbw_node is enabled.  While dbw_node is not enabled the ROS system does not effect control signals to the vehicle.

#### Waypoint Follower Node
The Waypoint Follower Node is comprosed mainly of some c++ code which translates the waypoints into vehicle coordinates and controls.  A yaw controller for the steering wheel angle was provided and worked sufficiently well so as not to need modifications.
The twist controller is where the brake and throttle controllers are located.  The work in this code is being done to determine whether the intent is to accelerate and maintain speed or decelerate to a stop.  A PID controller for both the brakes and the throttle are created here and tuned on the simulator to provide approximately desireable results (limit jerk and high acceleration values).  The brake controller has two stages, one for high speed coming to a stop and one for stopped.  The first is effected when the throttle command is 0 and the vehicle is going faster than the set point.  The brake controller determines an appropriate torque to apply in order to hit the target speed.  When the vehicle achieves a "stopped state"  (vehicle speed < threshold and vehicle commanded speed is zero) the brake controller rolls in the torque gradually to hit 700N-m to overcome torque creep due to the automatic transmission's torque converter.

### Challenges
This project was a fascinating learning experience.  The largest challenges came from dealing with hardware limitations.  The hardware limitations imposed by our team member's laptops resulted in significant latency issues.  The typical Intel Core i5-7200U CPU@2.5GHz with 8 GB of RAM on a 64 bit Windows system was not powerful enough to run the project without latency.  Comparable Mac computer hardware resulted in the same performance.  Native Linux on similar hardware provided better results but did not solve the issues.  A GPU was used by one team member to test the system and it worked much better.  The workspace also had latency issues.

These latency issues appear to be present when the camera is publshing images.  The amount of data being broadcast by the simulator is too much for our hardware.  The controller achitecture's ROS nodes can't process the inputs fast enough to effect proper control and the vehicle wanders off the track.  We spent a lot of time attempting to overcome these issues without moving to faster local hardware or AWS/Google Cloud based services.  
Outside of recompiling the Simulator code, the server.py file appears to be furthest upstream we can effect change.  This handles the server side TCP/IP data coming in and calls the functions in bridge.py.  The bridge.py file takes the information and publishes the ROS messages.  The tl_detector node then takes in the image data and/or ground truth traffic light position/state signal as a ROS subscriber and decides how to process this information.  The images coming out of the simulator are effectively like drinking from a fire hose and we couldn't get our system to work in symphony as one without testing in chunks.  This leads to our recommendations:

Recommendation #1:  Use the ground truth information in publish_traffic to test your dbw node controllers (brake, steering and throttle) as well as your waypoint updater node without turning on the camera in the simulator.  We did not use the image_cb function to effect this control as this would rely on the image_cb function being called when an image is available.  Therefore it is advisable to have a different method than described in the walkthrough videos for updating the stop waypoint index.

Recommendation #2: Limit the number of images coming into your Python software.  If you want you can attempt to recompile the simulator but this may result in other issues.  We did not take this approach.  We use two approaches but then settled on one preferred method.  The first method was to skip images recieved by the subscriber in tl_detector.py, however this was not the furthest upstream we could effect.  in bridge.py we limited the number of images coming in and being published.  This reduces overhead by not converting an image from cv2 as well as publishing and subscribing.

Recommendation #3: Unless you have a GPU similar to Carla, test your classifier in manual mode to ensure that you can print out the waypoint index that the vehicle will be targetting.

Recommendation #4: Test piecewise to accomplish this project in a reasonable timeframe.  In an attempt to overcome the latency issues we spent about 60 hours attempting to refine the code, find efficiencies to overcome our latency issue, investigate Docker, bootable linux partitions, AWS instances (McAfee opening ports etc..).  In the end the best we could do was understand our system better and setup piecewise testing methods with logwarn messages.   Native Linux maybe the best with an AWS instance however you do not want to go with more power than Carla has as this would not yield the desireable results.

Recommendation #5: Data collection in manual mode.  Collect and save images using manual mode driving or alternatively set your vehicle top throttle input to something incredibly small and walk away for an hour.

Recommendation #6: Use .[labelImg](https://github.com/tzutalin/labelImg) for labelling your images with boxes.  We effectively trained our classifier with about 150 images of each color for the simulator and labelled all of the images provided in the Rosbag file.

In summary this is examplary of the real world issue facing autonomous vehicles, artificial intelligence and real time systems.  Hardware limitations imposed by cost, size and availability are real, software needs to be matched to the hardware capabilities and attempting to do too much results in achieving very little.  The fastest, most efficient code needs to be implemented and high performing image classifiers with extended libraries of classification just aren't currently practical.


## Installation Options

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
