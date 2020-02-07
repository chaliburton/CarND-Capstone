from styx_msgs.msg import TrafficLight
import numpy as np
from keras_layers.keras_layer_AnchorBoxes import AnchorBoxes
from models.keras_ssd7 import build_model


class TLClassifier(object):
    def __init__(self):
         ## Load model weights
         weight_path = '/tmp/ssd7_TL_epoch-03_weights_simulator.h5'

         img_height = 600 # Height of the input images
         img_width = 800 # Width of the input images
         img_channels = 3 # Number of color channels of the input images
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
         self.model = build_model(image_size=(img_height, img_width, img_channels),
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

         # 2: Load some weights
         self.model.load_weights(weight_path, by_name=True)
        

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        1=Green, 2=Yellow, 3=Red 3<No TLs

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # load one image
        input_images = np.array([image])

        ## Predict images class
        y_pred = self.model.predict(input_images)

        # Filter predictions
        confidence_threshold = 0.9
        y_pred_thresh = [y_pred[k][y_pred[k,:,1] > confidence_threshold] for k in range(y_pred.shape[0])]

        np.set_printoptions(precision=2, suppress=True, linewidth=90)
        print("Predicted boxes:\n")
        print('   class   conf xmin   ymin   xmax   ymax')
        print(y_pred_thresh[0])
        return TrafficLight.UNKNOWN
