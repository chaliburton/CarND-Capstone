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
### Perception
####
Obstacle detection is not used in this project. Rather we use the traffic light coordinates given to us in the yaml files *stop_line_positions* to generate stop line indices where to the car will stop when the traffic light is red.


#### Traffic light Detection
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
