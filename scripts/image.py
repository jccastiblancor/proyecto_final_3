#!/usr/bin/env python
import rospy
import cv2
from proyecto_final_3.srv import Image

def click(event, x, y, flags, param):
    """
    Determina la posicion x,y de los clicks del boton izquierod del mouse y los almacena en una lista.
    ----------
    x : int
        pixel en la coordenada x de la imagen
    y : int
        pixel en la coordenada y de la imagen
    """
    if event == cv2.EVENT_LBUTTONDOWN:
        refPt.append((int(x * 2), int(y * 2)))

def handle_image(req):
    # Reading an image in default mode
    ruta_imagen = './src/taller5_3/scripts/results/map_pioneer.pgm'
    imagen = cv2.imread(ruta_imagen)
    dim = (int(500), int(500))  # Dimensiones del mapa
    imagen = cv2.resize(imagen, dim)
    cv2.imshow('Seleccione sus 4 puntos para reorganizar la vista', imagen)  # show the image
    cv2.waitKey(10)
    cv2.setMouseCallback('Seleccione sus 4 puntos para reorganizar la vista', click)
    if len(refPt) >= 4:
        return 0

def image_server(imagen):
    rospy.init_node('image_server', anonymous=True)
    s = rospy.Service('image', Image, handle_image)
    rospy.spin()


if __name__ == '__main__':
    refPt = []
    try:
        image_server()
    except rospy.ROSInterruptException:
        pass
