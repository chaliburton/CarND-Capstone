from styx_msgs.msg import TrafficLight
import numpy as np
from keras_layers.keras_layer_AnchorBoxes import AnchorBoxes
from models.keras_ssd7 import build_model
import os
import rospy
from PIL import Image
import cv2
import glob

class TLClassifier(object):
    def __init__(self,is_site):
        ## Load model weights site or simulator
        if is_site:
            rospy.loginfo("tl_classifier: Load model for site!")
            weight_path = 'light_classification/ssd7_TL_epoch-03_weights_Carla.h5'
        else:
            rospy.loginfo("tl_classifier: Load model for simulator!")
            weight_path = 'light_classification/ssd7_TL_epoch-03_weights_simulator.h5'
        
        self.img_height = 600 # Height of the input images
        self.img_width = 800 # Width of the input images
        self.img_channels = 3 # Number of color channels of the input images
        intensity_mean = 127.5 # Set this to your preference (maybe `None`). The current settings transform the input pixel values to the interval `[-1,1]`.
        intensity_range = 127.5 # Set this to your preference (maybe `None`). The current settings transform the input pixel values to the interval `[-1,1]`.
        n_classes = 8 # Number of positive classes
        scales = [0.08, 0.16, 0.32, 0.64, 0.96] # An explicit list of anchor box scaling factors. If this is passed, it will override `min_scale` and `max_scale`.
        aspect_ratios = [0.5, 1.0, 2.0] # The list of aspect ratios for the anchor boxes
        two_boxes_for_ar1 = True # Whether or not you want to generate two anchor boxes for aspect ratio 1
        steps = None # In case you'd like to set the step sizes for the anchor box grids manually; not recommended
        offsets = None # In case you'd like to set the offsets for the anchor box grids manually; not recommended
        clip_boxes = False # Whether or not to clip the anchor boxes to lie entirely within the image boundaries
        variances = [1.0, 1.0, 1.0, 1.0] # The list of variances by which the encoded target coordinates are scaled
        normalize_coords = True # Whether or not the model is supposed to use coordinates relative to the image size
        self.model = build_model(image_size=(self.img_height, self.img_width, self.img_channels),
                            n_classes=n_classes,
                            mode='inference',
                            l2_regularization=0.0005,
                            scales=scales,
                            aspect_ratios_global=aspect_ratios,
                            aspect_ratios_per_layer=None,
                            two_boxes_for_ar1=two_boxes_for_ar1,
                            steps=steps,
                            offsets=offsets,
                            clip_boxes=clip_boxes,
                            variances=variances,
                            normalize_coords=normalize_coords,
                            subtract_mean=intensity_mean,
                            divide_by_stddev=intensity_range)


        # Load the model weights        
        self.model.load_weights(weight_path, by_name=True)

        # Print numpy arrays in nice format when using print
        np.set_printoptions(precision=2, suppress=True, linewidth=90)
        

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        1=Green, 2=Yellow, 3=Red 3<No TLs

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # Convert image to PIL RGB image
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # add a fourth batch dimension to array
        image = np.expand_dims(image, axis=0)

        ## Predict images class
        if image.shape==(1, self.img_height, self.img_width, self.img_channels):
            y_pred = self.model.predict(image)
        else:
            rospy.logwarn("tl_classifier: Wrong image shape: {},{},{},{}".format(image.shape[0],image.shape[1],image.shape[2],image.shape[3]))
            return TrafficLight.UNKNOWN

        # Filter predictions
        confidence_threshold = 0.7
        y_pred_thresh = [y_pred[k][y_pred[k,:,1] > confidence_threshold] for k in range(y_pred.shape[0])]

        # Output predicted classes and scores
        #rospy.loginfo("tl_classifier: class   conf xmin   ymin   xmax   ymax")
  
        # Filter classes prediction
        tl_pred_classes = y_pred_thresh[0][:,0]
        tl_pred_scores = y_pred_thresh[0][:,1]
        # Find classes that contains tl's
        tl_pred_classes = [cl for cl in tl_pred_classes if 1<=cl<=3]


        # Test light state (if prediction is not empty)
        if len(tl_pred_classes) > 0:
            if (tl_pred_classes[0]==1):
                tl_return = TrafficLight.GREEN
                rospy.loginfo("tl_classifier: Green detected, score {:.2f}".format(tl_pred_scores[0]))
            elif (tl_pred_classes[0]==2):
                tl_return = TrafficLight.YELLOW
                rospy.loginfo("tl_classifier: Yellow detected, score {:.2f}".format(tl_pred_scores[0]))
            elif (tl_pred_classes[0]==3):
                tl_return = TrafficLight.RED
                rospy.loginfo("tl_classifier: Red detected, score {:.2f}".format(tl_pred_scores[0]))
            else:
                tl_return = TrafficLight.UNKNOWN
                rospy.loginfo("tl_classifier: Other class detected!")
        else:
            tl_return = TrafficLight.UNKNOWN
            rospy.loginfo("tl_classifier: Unknown detected!")


        return tl_return

if __name__ == "__main__":
    images_list = glob.glob('light_classification/bag_frames_color/*.jpg')
    light_classifier = TLClassifier(True)
    font = cv2.FONT_HERSHEY_SIMPLEX
    for image in images_list:
        print("processing image: ", image)
        image_cv = cv2.imread(image, cv2.IMREAD_COLOR)
        tl_return = light_classifier.get_classification(image_cv)
        if (tl_return == TrafficLight.GREEN):
            text = "GREEN"
            color = (0,255,0)
        elif (tl_return == TrafficLight.YELLOW):
            text = "YELLOW"
            color = (0,255,255)
        elif (tl_return == TrafficLight.RED):
            text = "RED"
            color = (0,0,255)
        else:
            text = "UNKNOWN"
            color = (255,0,0)
        
        cv2.putText(image_cv,text,(10,500), font, 4,color, 5, cv2.LINE_AA)
        cv2.imwrite('light_classification/bag_frames_color_classified/' + os.path.basename(image), image_cv)
    