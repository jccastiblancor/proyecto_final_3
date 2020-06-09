#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from proyecto_final_3.srv import Navegacion, Dibujo

def main():

    """
    Quitar comentarios de abajo para ir al inicio de la zona de dibujo
    """

    """
    
    navegacion = rospy.ServiceProxy('navegacion', Navegacion)

    metodo = "d"

    inicio = [6.235, 6.57] # Ajustar manualmente (3.366, -4.5, 1.57)
    destino = [6.235, 6.57]
    resp = navegacion(metodo, inicio, destino)
    
    """

    dibujo = rospy.ServiceProxy('dibujo', Dibujo)

    figura = "gato" # poner figura que van a dibujar

    inicio = [3.366, -4.5]
    resp = dibujo(figura, inicio)


if __name__ == '__main__':
    main()