#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from std_msgs.msg import String, Int32, Float32MultiArray
from sensor_msgs.msg import CompressedImage, Image

#velocidades = Float32MultiArray()

#velocidades.data = [1 1]


def callback_image_compressed(param):
    np_arr = np.fromstring(param.data, np.uint8)
    print(np_arr.shape)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    scale_percent = 1200  # percent of original size
    width = int(image_np.shape[1] * scale_percent / 100)
    height = int(image_np.shape[0] * scale_percent / 100)
    dim = (width, height)
    resized = cv2.resize(image_np, dim)
    cv2.imshow("Camara", resized)
    cv2.waitKey(1)

def vision_OP():
    rospy.init_node('vision', anonymous=True)  # Inicia el nodo teleop
    rospy.Subscriber("visionSensorData/image_raw/compressed", CompressedImage, callback_image_compressed, queue_size=1)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    vision_OP()
