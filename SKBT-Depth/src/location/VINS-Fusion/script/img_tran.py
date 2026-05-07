#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def image_callback(msg):
    
    bridge = CvBridge()
    # 将ROS图像消息转换为OpenCV图像
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    # 获取图像宽度和高度
    width, height, _ = cv_image.shape

    # 截取左半张图像
    left_half_image = cv_image[:, 0:width//2]

    # 将OpenCV图像转换回ROS图像消息
    left_half_msg = bridge.cv2_to_imgmsg(left_half_image, "bgr8")

    # 发布左半张图像到话题B
    publisher.publish(left_half_msg)

    rospy.loginfo("success")

if __name__ == '__main':
    rospy.init_node('image_processor')
    rospy.Subscriber("/vins_fusion/image_track", Image, image_callback)
    publisher = rospy.Publisher("topicB", Image, queue_size=10)
    rospy.spin()