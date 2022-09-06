#!/usr/bin/env python
from cv2 import COLOR_RGBA2RGB
import rospy
from sensor_msgs.msg import Image
import cv2 as cv
import torch
import cv_bridge
import math
import numpy as np
import pathlib
from car.semantic_segmentation.model import SemanticSegmentationModel
import time

device = "cuda" if torch.cuda.is_available() else "cpu"

bridge = cv_bridge.CvBridge()
image_pub = rospy.Publisher("test_bounding_boxes",Image)

class SemanticSegmentationNode:
    def __init__(self):
        rospy.init_node('object_tracker_estimation')

        #Load Segmentation Model
        path_to_network = rospy.get_param("semantic_segmentation_python_module_path")
        path_to_package = pathlib.Path(__file__).parent.parent.parent.parent.resolve().__str__()+ "/"
        self.semantic_segmentation_model = SemanticSegmentationModel()
        self.semantic_segmentation_model.load_state_dict(torch.load(path_to_package + path_to_network))
        self.semantic_segmentation_model.to(device)
        self.semantic_segmentation_model.eval()
        self.width_of_network = rospy.get_param("network_width")
        self.height_of_network = rospy.get_param("network_height")
        self.subscriber_topic = rospy.get_param("camera_topic")
        subscriber = rospy.Subscriber(self.subscriber_topic, Image, self.image_callback)
        rospy.spin()

    def image_callback(self,data):
        with torch.no_grad():
            cv_image = bridge.imgmsg_to_cv2(data,desired_encoding='passthrough')
            rgb_image = cv_image
            resized_image = cv.resize(rgb_image,(self.width_of_network,self.height_of_network))
            input_to_network = torch.from_numpy(resized_image).permute((2,0,1)).unsqueeze(0)
            input_to_network = input_to_network.type(torch.FloatTensor).to(device)
            guess = self.semantic_segmentation_model(input_to_network).argmax(1).to("cpu")
            output_image = guess.mul(80).permute((1,2,0)).squeeze().type(torch.ByteTensor).numpy()
            image_pub.publish(bridge.cv2_to_imgmsg(output_image, "8UC1"))

if __name__ == '__main__':
    node = SemanticSegmentationNode()
