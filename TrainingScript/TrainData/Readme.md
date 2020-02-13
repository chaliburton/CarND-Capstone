# Questions
* Use Keras
Easier to extend model and train
No SSD  model in Keras pretrained models

One is in Github: https://github.com/pierluigiferrari/ssd_keras
Training code included

Model should be output in TF format
Suggested by Udacity
* Use Tensorflow
- Already have model
- Final model must be in Tensorflow
* Use a detection model and classification model

pro:
- Avoids to label bounding boxes
- Simple classification model

con:
- Detector may not work with simulator
- Must use two steps/NN


# Usefull links
https://machinelearningmastery.com/how-to-perform-object-detection-with-yolov3-in-keras/

SSD implemented in Keras
https://github.com/pierluigiferrari/ssd_keras  

https://github.com/affinis-lab/traffic-light-detection-module

https://github.com/JunshengFu/traffic-light-detector

https://medium.com/@UdacityINDIA/self-driving-vehicles-traffic-light-detection-and-classification-with-tensorflow-object-detection-d6a4d25e99c2

https://github.com/marcomarasca/SDCND-Traffic-Light-Detection

## Howto export frozen graph
In order to use the model for inference in production the graph must be freezed, the tensorflow object API comes with an handy utility to export the frozen model (See https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/exporting_models.md):


## Availabele datasets for simulator already annotated
- Udacity Training bag

- Tool for rapid labelling from picture and video 
https://diffgram.com/
https://github.com/tzutalin/labelImg

Diffgram might also have already uploaded datasets on traffic lights from simulator

Annotation tool of images already done on Udacity images earlier
https://github.com/pierluigiferrari/data_generator_object_detection_2d

# Notes
-From https://medium.com/@anthony_sarkis/self-driving-cars-implementing-real-time-traffic-light-detection-and-classification-in-2017-7d9ae8df1c58
https://medium.com/@UdacityINDIA/self-driving-vehicles-traffic-light-detection-and-classification-with-tensorflow-object-detection-d6a4d25e99c2
R-CNN is superior to SSD on small objects
- Used 260 samples from simulator
- Udacity images is 800x600



# Tasks
* Get pretrained SSD Resnet or Mobilenet model in Python notebook
* Extend model with new layer and/or retrain last layers
* Save model in TF format or Keras (is should be supported)
* Get training data from simulator, no support for bounding boxes from ros
* Label training data (use tool to label bounding boxes)
* Upload Object detection lab to common Github


# Shortcuts
conda activate deep-learning
cd /home/sra/Udacity/SelfDrivingCar/CarND-Capstone/Object-Detection-Lab/TrainingScript/ssd_keras
jupyter notebook
 
