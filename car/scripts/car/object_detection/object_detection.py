#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2 as cv
import torch
import car.object_detection.utils as utils
import car.object_detection.constants as constants
from car.object_detection.yolo_model import YOLO
from car.object_detection.pose_model import PoseModel
import cv_bridge
import math
import numpy as np
import geometry_msgs.msg
import tf.transformations
import time
import rospkg
import random
import matplotlib.pyplot as plt
import statistics
device = "cuda" if torch.cuda.is_available() else "cpu"
rospack = rospkg.RosPack()

yolo_model = YOLO()
yolo_model.load_state_dict(torch.load(rospack.get_path('car') + "/models/object_detection/yolo_trained_network.pth"))
yolo_model.to(device)
yolo_model.eval()

pose_model = PoseModel()
pose_model.load_state_dict(torch.load(rospack.get_path('car') + "/models/object_detection/pose_network.pth"))
pose_model.to(device)
pose_model.eval()

bridge = cv_bridge.CvBridge()
image_pub = rospy.Publisher("test_bounding_boxes",Image)
pose_pub = rospy.Publisher("car_local_pointer",geometry_msgs.msg.PoseArray)

def learning_callback(data):
    with torch.no_grad():
        # rospy.loginfo("Test")
        cv_image = bridge.imgmsg_to_cv2(data,desired_encoding='passthrough')
        # For YOLO
        resized_yolo_image = cv.resize(cv_image,(constants.yolo_width_of_image,constants.yolo_height_of_image))
        yolo_tensor = torch.from_numpy(resized_yolo_image).to(device)
        removed_nan_yolo_tensor = torch.nan_to_num(yolo_tensor.to(device), nan=100.0, posinf=100.0, neginf=100.0)
        normalized_yolo_input = removed_nan_yolo_tensor.mul(-1/5)
        # For Pose
        resized_pose_image = cv.resize(cv_image,(constants.pose_network_width,constants.pose_network_height))
        pose_tensor = torch.from_numpy(resized_pose_image).to(device)
        removed_nan_pose_tensor = torch.nan_to_num(pose_tensor.to(device), nan=0.0, posinf=0.0, neginf=0.0)

        network_prediction = yolo_model(normalized_yolo_input.unsqueeze(0).unsqueeze(0))
        bounding_boxes = utils.get_bounding_boxes_for_prediction(network_prediction)

        # Publishing Local Map
        pose_array = geometry_msgs.msg.PoseArray()
        pose_array.poses = []
        pose_topic_model = geometry_msgs.msg.Pose()
        point_topic_model = geometry_msgs.msg.Point()
        point_topic_model.x = 0
        point_topic_model.y = 0
        pose_topic_model.position = point_topic_model
        pose_array.poses.append(pose_topic_model)

        for bounding_box in bounding_boxes:
            #Display
            x_min = max(0,int(constants.original_width_image / constants.yolo_width_of_image *(bounding_box[1]-bounding_box[3]//2)))
            x_max = min(constants.original_width_image,int(constants.original_width_image / constants.yolo_width_of_image *(bounding_box[1]+bounding_box[3]//2)))
            y_min = max(0,int(constants.original_height_image / constants.yolo_height_of_image *(bounding_box[2]-bounding_box[4]//2)))
            y_max = min(constants.original_height_image,int(constants.original_height_image / constants.yolo_height_of_image *(bounding_box[2]+bounding_box[4]//2)))
            
            # y_cap = int(y_max - 1/10 *(y_max-y_min))
            # x_avg = int((x_min+x_max)/2)
            start_point = (x_min, y_min)
            end_point = (x_max, y_max)
            color = 9
            thickness = 2
            cv_image = cv.rectangle(cv_image, start_point, end_point, color, thickness)
            # Find Pose
            yaw = 0
            # Find location
            x_local_frame, y_local_frame = find_location_of_car(removed_nan_pose_tensor,x_min,x_max,y_min,y_max)
            
            pose_topic_model = geometry_msgs.msg.Pose()
            point_topic_model = geometry_msgs.msg.Point()
            point_topic_model.x = y_local_frame
            point_topic_model.y = x_local_frame
            orientation_model = geometry_msgs.msg.Quaternion()
            quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
            orientation_model.w = quaternion[3]
            orientation_model.x = quaternion[0]
            orientation_model.y = quaternion[1]
            orientation_model.z = quaternion[2]
            pose_topic_model.orientation = orientation_model
            pose_topic_model.position = point_topic_model
            pose_array.poses.append(pose_topic_model)

        pose_array.header.frame_id = 'map'
        pose_pub.publish(pose_array)

    ret, reduced_range_image = cv.threshold(cv_image,10,100,cv.THRESH_TRUNC)
    image_pub.publish(bridge.cv2_to_imgmsg(reduced_range_image, "32FC1"))

def find_location_of_car(image, x_min,x_max,y_min,y_max):
    resize_x_min = x_min / constants.original_width_image * constants.pose_network_width
    resize_x_max = x_max / constants.original_width_image * constants.pose_network_width
    resize_y_min = y_min / constants.original_height_image * constants.pose_network_height
    resize_y_max = y_max / constants.original_height_image * constants.pose_network_height
    cropped_image = image
    cropped_image[0:int(resize_y_min), :] = 0
    cropped_image[int(resize_y_max):, :] = 0
    cropped_image[:, 0:int(resize_x_min)] = 0
    cropped_image[:, int(resize_x_max):] = 0
    prediction = pose_model(cropped_image.unsqueeze(0).unsqueeze(0))
    x = prediction[0][0]
    y = prediction[0][1]
    return x,y

def setup():
    rospy.init_node('object_tracker_estimation')
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, learning_callback)
    rospy.spin()

if __name__ == '__main__':
    setup()
