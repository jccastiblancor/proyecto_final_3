#!/usr/bin/env python3
import rospy
import numpy as np
import time
from std_msgs.msg import String, Int32, Float32MultiArray
from geometry_msgs.msg import Twist
from proyecto_final_3.srv import Navegacion, GridmapPoints

x, y, theta = 0.0, 0.0, 0.0

hayRuta = False
resp = None


def solicitarRuta():
    """
    main del servicio navegacion
    """
    global hayRuta, resp, x, y

    navegacion = rospy.ServiceProxy('navegacion', Navegacion)

    metodo = "A"

    inicio = [-6.23, 6.57]
    destino = [6.5, 6.5]

    x = inicio[0]
    y = inicio[1]

    resp = navegacion(metodo, inicio, destino)

    print('rutax: "{}"'.format(resp.rutax))
    print('rutay: "{}"'.format(resp.rutay))

    hayRuta = True


def callback_pos(param):
    global x, y, theta
    x = (param.linear.x)
    y = (param.linear.y)
    theta = (param.angular.z)


def acomodar(param):
    global theta, plot_mensaje
    msj = Float32MultiArray()
    pub = rospy.Publisher('/pioneer_motorsVel', Float32MultiArray, queue_size=1)

    if param > 0.05:
        velIzq = 1
        velDer = 0

    elif param < -0.05:
        velIzq = 0
        velDer = 1

    elif param >= -0.05 and param <= 0.05:
        velIzq = 0
        velDer = 0
    msj.data = [velIzq, velDer]
    pub.publish(msj)


def girar(param):
    kp = 0.5
    ka = 1.5 + 1 * np.exp(-param)
    R = 0.195
    l = 0.381

    giro = Float32MultiArray()
    pub = rospy.Publisher('/pioneer_motorsVel', Float32MultiArray, queue_size=10)

    w = ka * param + kp * np.sin(param) * np.cos(param)
    velIz = (- w * l / 2) / R
    velDer = (+ w * l / 2) / R
    giro.data = [velIz, velDer]

    pub.publish(giro)


def adelantar(rho, alpha):
    kp = 1.2 + 1.5 * np.exp(-rho)
    ka = 1.5 + 1 * np.exp(-alpha)
    R = 0.195
    l = 0.381
    adelante = Float32MultiArray()
    pub = rospy.Publisher('/pioneer_motorsVel', Float32MultiArray, queue_size=10)

    v = kp * rho * np.cos(alpha)
    w = ka * alpha + kp * np.sin(alpha) * np.cos(alpha)

    velIz = (v - w * l / 2) / R
    velDer = (v + w * l / 2) / R
    adelante.data = [velIz, velDer]

    pub.publish(adelante)


# PLS NO TOCAR FALTA INCORPORAR EL CAMBIO DE POSICION FINAL SEGUN LA RUTA Y CAMBIAR DE VELOCIDAD LINEAL Y ANGULAR A LAS VELOCIDADES DE CADA RUEDA
def traccion_OP():
    global theta, x, y, hayRuta, resp
    # msj = Twist()
    msj = Float32MultiArray()
    rospy.init_node('traccion', anonymous=True)
    rospy.Subscriber('/pioneer_position', Twist, callback_pos, queue_size=1)
    pub = rospy.Publisher('/pioneer_motorsVel', Float32MultiArray, queue_size=10)
    # pub = rospy.Publisher('/pioneer_cmdVel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if hayRuta:
            for i in range(len(resp.rutax)):
                if i != 0:
                    print(i)
                    xf = resp.rutax[i]
                    yf = resp.rutay[i]
                    rho = 100
                    alpha = 100

                    while (alpha > 0.05 or alpha < - 0.05) and not rospy.is_shutdown():
                        error = [xf - x, yf - y]
                        alpha = np.arctan2(error[1], error[0]) - theta
                        print(alpha, np.arctan2(error[1], error[0]), theta)
                        girar(alpha)
                        rate.sleep()

                    msj.data = [0, 0]
                    pub.publish(msj)
                    time.sleep(1)

                    while (alpha > 0.01 or alpha < - 0.01) and not rospy.is_shutdown():
                        error = [xf - x, yf - y]
                        alpha = np.arctan2(error[1], error[0]) - theta
                        print(alpha, np.arctan2(error[1], error[0]), theta)
                        girar(alpha)
                        rate.sleep()

                    print('------------------------------')

                    while (rho > 0.04) and not rospy.is_shutdown():
                        error = [xf - x, yf - y]
                        alpha = np.arctan2(error[1], error[0]) - theta
                        rho = np.sqrt(np.power(error[0], 2) + np.power(error[1], 2))
                        print(rho, alpha, theta)
                        adelantar(rho, alpha)
                        rate.sleep()

            msj.data = [0, 0]
            pub.publish(msj)

        hayRuta = False
        rate.sleep()


if __name__ == '__main__':
    solicitarRuta()
    traccion_OP()

"""
    if hayRuta:
            for i in range(len(resp.rutax)):
                yf = resp.rutax[i]
                xf = resp.rutay[i]
                thetaf = 0

                error = [yf - x, xf - y]
                rho = np.sqrt(np.power(error[0],2)+np.power(error[1],2))
                alpha = np.arctan2(error[1],error[0])-theta
                beta = -thetaf + theta
                print(i)

                while (rho > 0.05 or (beta <= -0.05 and beta >= 0.05)) and not rospy.is_shutdown():
                    print(xf, x, yf, y)
                    error = [yf - x, xf - y]
                    rho = np.sqrt(np.power(error[0],2)+np.power(error[1],2))
                    print(rho)
                    alpha = np.arctan2(error[1],error[0])-theta
                    print(alpha)
                    beta = -thetaf + theta #Falta sacar el thetaf en cada trayecto

                    if rho > 0.05:
                        kp = 0.9 +  1.5*np.exp(-rho)
                        v = kp*rho*np.cos(alpha)
                        w = ka*alpha+kp*np.sin(alpha)*np.cos(alpha)
                        msg.linear.x = v
                        msg.angular.z = w
                        pub.publish(msg)
                        #velIzq=w*(R - l/2)
                        #velDer=w*(R + l/2)
                        #velocidades.data = [velIzq, velDer]
                        #pub.publish(velocidades)

                    elif(rho<=0.05):
                        acomodar(beta)
                    rate.sleep()
            hayRuta = False
"""
