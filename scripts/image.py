#!/usr/bin/env python
import rospy
import cv2
#from std_msgs.msg import Foo

def click(event, x, y, flags, param): #Determina la posicion x,y de los clicks del boton izquierod del mouse y los almacena en una lista
    if event == cv2.EVENT_LBUTTONDOWN:
        refPt.append((int(x*2), int(y*2)))

def image(imagen):
    #pub = rospy.Publisher('coordenadas', Foo)
    rospy.init_node('image', anonymous=True)
    #msg_to_send = Foo()

    # Reading an image in default mode
    imagen = cv2.imread(imagen)
    dim = (int(500), int(500))  # Dimensiones del mapa
    imagen = cv2.resize(imagen, dim)

    # cv2.setMouseCallback('Seleccione sus 4 puntos para reorganizar la vista', click)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cv2.imshow('Seleccione sus 4 puntos para reorganizar la vista', imagen)  # show the image
        cv2.waitKey(10)
        cv2.setMouseCallback('Seleccione sus 4 puntos para reorganizar la vista', click)
        rate.sleep()
        if len(refPt) >= 4:
            print(refPt)
            #msg_to_send.coords = refPt
            #pub.publish(msg_to_send)
            #cv2.destroyWindow('Seleccione sus 4 puntos para reorganizar la vista')  # Destruye la imagen
            rospy.signal_shutdown('Acabo')


if __name__ == '__main__':
    refPt = []
    ruta_imagen = './src/taller5_3/scripts/results/map_pioneer.pgm'

    try:
        image(ruta_imagen)
    except rospy.ROSInterruptException:
        pass