#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from std_msgs.msg import String, Int32, Float32MultiArray
from sensor_msgs.msg import CompressedImage, Image
import time
from geometry_msgs.msg import Twist

vamosA = 0
vamosV = 0
vamosR = 0

x1 = 0
y1 = 0
x1V = 0
y1V = 0
x1R = 0
y1R = 0
x = 0
y = 0

distanciaA = True
distanciaV = True
distanciaR = True


def callback_image_compressed(param):
    global distanciaA, distanciaV, distanciaR, x, y, vamosA, vamosV, vamosR, x1, y1, x1V, y1V, x1R, y1R

    np_arr = np.fromstring(param.data, np.uint8)  # Pasar de String que viene a uint8
    msj = Float32MultiArray()
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Lo pasa a RGB
    scale_percent = 1200  # percent of original size
    width = int(image_np.shape[1] * scale_percent / 100)  # Reshape ancho
    height = int(image_np.shape[0] * scale_percent / 100)  # Reshape alto
    dim = (width, height)  # Nuevas Dimensiones
    resized = cv2.resize(image_np, dim)  # Realizo el reshape a la imagen
    # cv2.imshow("Camara", resized) #Mostrar lo que mira la camara

    # PARA MIRAR ESFERAS AZULES
    Noquiero_a = cv2.add(resized[:, :, 1], resized[:, :, 2])  # Informacion que no quiero observar
    ret, mejorno_a = cv2.threshold(Noquiero_a, 112, 255,
                                   cv2.THRESH_BINARY)  # REALIZO UN THRESHOLD DE PARA GENERAR MASCARA INVERTIDA
    Si_a = cv2.bitwise_not(mejorno_a)  # Genero la mascara que si quiero
    Azul_1 = cv2.bitwise_and(Si_a, resized[:, :, 0])
    ret, mejorsi_a = cv2.threshold(Azul_1, 100, 255, cv2.THRESH_BINARY)
    kernel1 = np.ones((5, 5), np.uint8)  # El kernel utilizado
    dilation_a = cv2.dilate(mejorsi_a, kernel1, iterations=1)
    # cv2.imshow("Azul",dilation_a)

    # PARA MIRAR ESFERAS VERDES
    Noquiero_v = cv2.add(resized[:, :, 0], resized[:, :, 2])  # Informacion que no quiero observar
    ret, mejorno_v = cv2.threshold(Noquiero_v, 112, 255,
                                   cv2.THRESH_BINARY)  # REALIZO UN THRESHOLD DE PARA GENERAR MASCARA INVERTIDA
    Si_v = cv2.bitwise_not(mejorno_v)  # Genero la mascara que si quiero
    Verde_1 = cv2.bitwise_and(Si_v, resized[:, :, 1])
    ret, mejorsi_v = cv2.threshold(Verde_1, 100, 255, cv2.THRESH_BINARY)
    kernel = np.ones((5, 5), np.uint8)  # El kernel utilizado
    dilation_v = cv2.dilate(mejorsi_v, kernel, iterations=1)
    dilation_v2 = cv2.dilate(dilation_v, kernel, iterations=1)
    # cv2.imshow("Verde",dilation_v2)

    # PARA MIRAR ESFERAS ROJO
    Noquiero_r = cv2.add(resized[:, :, 0], resized[:, :, 1])  # Informacion que no quiero observar
    ret, mejorno_r = cv2.threshold(Noquiero_r, 112, 255,
                                   cv2.THRESH_BINARY)  # REALIZO UN THRESHOLD DE PARA GENERAR MASCARA INVERTIDA
    Si_r = cv2.bitwise_not(mejorno_r)  # Genero la mascara que si quiero
    Rojo_1 = cv2.bitwise_and(Si_r, resized[:, :, 2])
    ret, mejorsi_r = cv2.threshold(Rojo_1, 100, 255, cv2.THRESH_BINARY)
    kernel = np.ones((5, 5), np.uint8)  # El kernel utilizado
    dilation_r = cv2.dilate(mejorsi_r, kernel, iterations=1)
    dilation_r2 = cv2.dilate(dilation_r, kernel, iterations=1)
    # cv2.imshow("Rojo", dilation_r2)
    cv2.waitKey(1)

    rospy.Subscriber('/pioneer_position', Twist, callback_pos, queue_size=1)
    circulosA = encontrarCentros(dilation_a)
    circulosV = encontrarCentros(dilation_v2)
    circulosR = encontrarCentros(dilation_r2)

    if circulosA:
        if distanciaA and x != 0:
            vamosA += 1
            x1 = x
            y1 = y
            distanciaA = False
        else:
            x2 = x
            y2 = y
            if (abs(x2 - x1) ** 2 + abs(y2 - y1) ** 2) ** (1 / 2) > 1:
                distanciaA = True

    if circulosV:
        if distanciaV and x != 0:
            vamosV += 1
            x1V = x
            y1V = y
            distanciaV = False
        else:
            x2V = x
            y2V = y
            if (abs(x2V - x1V) ** 2 + abs(y2V - y1V) ** 2) ** (1 / 2) > 1:
                distanciaV = True

    if circulosR and x != 0:
        if distanciaR:
            x1R = x
            y1R = y
            distanciaR = False
        else:
            x2R = x
            y2R = y
            if (abs(x2R - x1R) ** 2 + abs(y2R - y1R) ** 2) ** (1 / 2) > 1:
                distanciaR = True

    print(circulosA, circulosV, circulosR)
    print(vamosA, vamosV, vamosR)

    pub = rospy.Publisher('/colores', Float32MultiArray, queue_size=10)
    msj.data = [vamosA, vamosV, vamosR]
    pub.publish(msj)


def encontrarCentros(image):  # Esta funcion encuentra los centros de los circulos
    blurred = cv2.GaussianBlur(image, (5, 5), 0)
    mask = cv2.inRange(blurred, 5, 255)  # el diametro es de 5.
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find and use the biggest contour
    circulos = []
    for cnt in contours:
        (c_x, c_y), c_r = cv2.minEnclosingCircle(cnt)
        x = c_x
        y = c_y
        r = c_r

        if c_r >= 50:
            circulos.append([(round(x), round(y)), round(r)])
    return circulos  # Retorna la cantidad de centros que encuentra.


def callback_pos(param):
    global x, y
    x = param.linear.x
    y = param.linear.y


def vision_OP():
    global pub
    rospy.init_node('vision', anonymous=True)  # Inicia el nodo teleop
    # rospy.Subscriber("visionSensorData/image_raw", Image, callback_image_raw, queue_size=1)
    rospy.Subscriber("visionSensorData/image_raw/compressed", CompressedImage, callback_image_compressed, queue_size=1)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    vision_OP()
