#!/usr/bin/env python
 
import rospy
import cv2
import numpy as np
import os, rospkg
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError
from utils import BEVTransform


class IMGParser:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.img_bgr = None

    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)


def main():

    rp = rospkg.RosPack()
    
    currentPath = rp.get_path("lane_detection_example")
    
    with open(os.path.join(currentPath, 'sensor/sensor_params.json'), 'r') as fp:
        sensor_params = json.load(fp)

    params_cam = sensor_params["params_cam"]

    rospy.init_node('lane_birdview', anonymous=True)

    image_parser = IMGParser()
    bev_op = BEVTransform(params_cam=params_cam)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        if image_parser.img_bgr is not None:

            img_warp = bev_op.warp_bev_img(image_parser.img_bgr)

            img_concat = np.concatenate([image_parser.img_bgr, img_warp], axis=1)

            cv2.imshow("birdview", img_concat)
            
            cv2.waitKey(1)

            rate.sleep()


if __name__ == '__main__':
    
    main()
