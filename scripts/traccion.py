#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from std_msgs.msg import String, Int32, Float32MultiArray
from sensor_msgs.msg import CompressedImage, Image

velocidades = Float32MultiArray()
pub = None

#velocidades.data = [1 1]
def movimiento():
    global pub, velocidades
    velIz = 100
    velDer = 100
    velocidades.data = [velIz , velDer]
    pub.publish(velocidades)

def traccion_OP():
    global pub
    rospy.init_node('traccion', anonymous=True)  # Inicia el nodo teleop
    pub = rospy.Publisher('pioneer_motorsVel', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        movimiento()
        rate.sleep()


if __name__ == '__main__':
    traccion_OP()