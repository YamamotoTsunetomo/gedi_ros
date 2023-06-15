#!/usr/bin/env python3

import os
import rospy
import numpy as np
import cv2

from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType, DTParam, ParamType
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped 
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage


HOST = os.environ['VEHICLE_NAME']
class FilteringNode(DTROS):
    def __init__(self, node_name):
        super(FilteringNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        self.bridge = CvBridge()

        self.sub_cam = rospy.Subscriber(
            f'/{HOST}/camera_node/image/compressed',
            CompressedImage,
            self.cam_cb
        )

        self.pub_gauss = rospy.Publisher(
            '/gauss',
            CompressedImage,
            queue_size=1
        )

    def cam_cb(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        img_cv2 = self.bridge.compressed_imgmsg_to_cv2(img, 'bgr8')

        gauss = cv2.GaussianBlur(img_cv2, (0,0), 3)
        self.publish_gauss(gauss)

        sobelx = cv2.Sobel(gauss, cv2.CV_64F, 1, 0)
        sobely = cv2.Sobel(gauss, cv2.CV_64F, 0, 1)

        mag = np.sqrt(sobelx*sobelx + sobely*sobely)

        thresh = cv2.threshold(mag, cv2.THRESH_BINARY)
        self.publish_thresh(thresh)



    def publish_gauss(self, img):
        compressed = self.bridge.cv2_to_compressed_imgmsg(img)
        self.pub_gauss.publish(compressed)

    def from_comp(self):
        pass


if __name__ == '__main__':
    node = FilteringNode('filtering_node')
    rospy.spin()