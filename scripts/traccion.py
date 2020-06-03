#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import String, Int32, Float32MultiArray
from proyecto_final_3.srv import Navegacion, GridmapPoints

x,y,theta = 0.0
hayRuta = False

def solicitarRuta():
    """
    main del servicio navegacion
    """
    global hayRuta
    rospy.init_node('nodo_test')

    navegacion = rospy.ServiceProxy('navegacion', Navegacion)

    metodo = "d"

    inicio = [6.235, 6.57]
    destino = [-2.331, -6.75]
    resp = navegacion(metodo, inicio, destino)

    print('rutax: "{}"'.format(resp.rutax))
    print('rutay: "{}"'.format(resp.rutay))

    inicio = [-2.331, -6.75]
    destino = [3.366, -4.5]
    resp = navegacion(metodo, inicio, destino)

    print('rutax: "{}"'.format(resp.rutax))
    print('rutay: "{}"'.format(resp.rutay))
    hayRuta = True

def callback_pos(param):
    global x,y,theta
    x=(param.linear.x)
    y=(param.linear.y)
    theta=(param.angular.z)

def acomodar(param):
    global theta, plot_mensaje
    msj = Float32MultiArray()
    pub = rospy.Publisher('/pioneer_motorsVel', Twist, queue_size=1)

    if param >0.01:
        velIzq = 1
        velDer = -1
        
    elif param < -0.01:
        velIzq = -1
        velDer = 1

    elif param >= -0.01 and param <= 0.01:
        velIzq = 0
        velDer = 0
    msj.data = [velIzq , velDer] 
    pub.publish(msj)

#PLS NO TOCAR FALTA INCORPORAR EL CAMBIO DE POSICION FINAL SEGUN LA RUTA Y CAMBIAR DE VELOCIDAD LINEAL Y ANGULAR A LAS VELOCIDADES DE CADA RUEDA
def traccion_OP():
    global theta,x,y,hayRuta
    velocidades = Float32MultiArray()
    R = 0.195
    l = 0.381
    ka = 5
    kb = -0.1

    rospy.init_node('traccion', anonymous=True)
    rospy.Subscriber('/pioneer_position', Twist, callback_pos, queue_size=1)
    pub = rospy.Publisher('/pioneer_motorsVel', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10)
    ruta = True
    while not rospy.is_shutdown():
        if hayRuta:
            for i in length(resp.rutax):
                xf = resp.rutax[i]
                yf = resp.rutay[i]
                thetaf = 0
                while rho > 0.01 or (beta <= -0.01 and beta >= 0.01):
                    error = [xf - x,yf-y]
                    rho = np.sqrt(np.power(error[0],2)+np.power(error[1],2))
                    alpha = np.arctan2(error[1],error[0])-theta
                    beta = -thetaf + theta #Falta sacar el thetaf en cada trayecto

                    if rho > 0.01:
                        kp = 0.8 + 1.5*np.exp(-rho)
                        v = kp*rho*np.cos(alpha)
                        w = ka*alpha+kp*np.sin(alpha)*np.cos(alpha)
                        velIzq=w*(R - l/2)
                        velDer=w*(R + l/2)
                        velocidades.data = [velIzq, velDer]
                        pub.publish(velocidades)
                        rate.sleep()

                    elif(rho<=0.01):
                        acomodar(beta)
                        rate.sleep()
            hayRuta = False
        rate.sleep()

if __name__ == '__main__':
    solicitarRuta()
    traccion_OP()