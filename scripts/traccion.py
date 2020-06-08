#!/usr/bin/env python3
import rospy
import numpy as np
import time
from std_msgs.msg import String, Int32, Float32MultiArray
from geometry_msgs.msg import Twist
from proyecto_final_3.srv import Navegacion, GridmapPoints, Dibujo

x, y, theta = 0.0, 0.0, 0.0

hayRuta = False
resp = None
pintar = None

def hacer_dibujo(inicio, figura):
    global hayRuta, resp, x, y

    dibujo = rospy.ServiceProxy('dibujo', Dibujo)

    resp = dibujo(figura, inicio)

    x = inicio[0]
    y = inicio[1]

    hayRuta = True

def solicitarRuta(inicio, destino, algoritmo='A star'):
    """
    main del servicio navegacion
    """
    global hayRuta, resp, x, y

    navegacion = rospy.ServiceProxy('navegacion', Navegacion)

    if algoritmo == 'Dijkstra':
        metodo = "d"
    else:
        metodo = "A"

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

    w_max = 1.5

    giro = Float32MultiArray()
    pub = rospy.Publisher('/pioneer_motorsVel', Float32MultiArray, queue_size=10)

    w = ka * param + kp * np.sin(param) * np.cos(param)

    if w > w_max:
        w = w_max

    if w < -w_max:
        w = -w_max
    velIz = (- w * l / 2) / R
    velDer = (+ w * l / 2) / R
    giro.data = [velIz, velDer]

    pub.publish(giro)


def adelantar(rho, alpha):
    kp = 1.2 + 1.5 * np.exp(-rho)
    ka = 1.5 + 1 * np.exp(-alpha)
    R = 0.195
    l = 0.381

    vmax = 1.5
    adelante = Float32MultiArray()
    pub = rospy.Publisher('/pioneer_motorsVel', Float32MultiArray, queue_size=10)

    v = kp * rho * np.cos(alpha)
    w = ka * alpha + kp * np.sin(alpha) * np.cos(alpha)

    if v > vmax:
        v = vmax

    velIz = (v - w * l / 2) / R
    velDer = (v + w * l / 2) / R

    adelante.data = [velIz, velDer]

    pub.publish(adelante)


def inicio(param):
    global pintar
    mensaje = param.data

    mensaje = mensaje.replace('(', '')
    mensaje = mensaje.replace(')', '')

    info = mensaje.split(' ; ')

    nodo_data = info[0].split(',')
    info[0] = [float(nodo_data[0]), float(nodo_data[1])]

    nodo_data = info[1].split(',')
    info[1] = [float(nodo_data[0]), float(nodo_data[1])]

    nodo_data = info[2].split(',')
    info[2] = [float(nodo_data[0]), float(nodo_data[1])]
    pintar = info[2]

    solicitarRuta(info[0], info[1], info[3])


# PLS NO TOCAR FALTA INCORPORAR EL CAMBIO DE POSICION FINAL SEGUN LA RUTA Y CAMBIAR DE VELOCIDAD LINEAL Y ANGULAR A LAS VELOCIDADES DE CADA RUEDA
def traccion_OP():
    global theta, x, y, hayRuta, resp, pintar
    # msj = Twist()
    msj = Float32MultiArray()
    rospy.init_node('traccion', anonymous=True)
    rospy.Subscriber('/start', String, inicio, queue_size=1)
    rospy.Subscriber('/pioneer_position', Twist, callback_pos, queue_size=1)
    pub = rospy.Publisher('/pioneer_motorsVel', Float32MultiArray, queue_size=10)
    # pub = rospy.Publisher('/pioneer_cmdVel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    pintar = True
    dibujo = False

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
                        angulo = np.arctan2(error[1], error[0])
                        if angulo > 2.5 or angulo < -2.5:
                            alpha = abs(angulo) * np.sign(theta) - theta
                        else:
                            alpha = angulo - theta
                        print(alpha, angulo, theta)
                        girar(alpha)
                        rate.sleep()

                    msj.data = [0, 0]
                    pub.publish(msj)
                    time.sleep(1)

                    print('------------------------------')

                    while (rho > 0.04) and not rospy.is_shutdown():
                        error = [xf - x, yf - y]
                        angulo = np.arctan2(error[1], error[0])
                        if angulo > 2.8 or angulo < -2.8:
                            """
                            if angulo < 3.1:
                                angulo = 3.1
                            if angulo < -3.1:
                                angulo = -3.1
                            alpha = abs(angulo) * np.sign(theta) - theta
                            """
                            angulo = np.arctan2(-error[0], error[1])
                            theta = theta + 90 # TODO falta probar esto
                        alpha = angulo - theta
                        rho = np.sqrt(np.power(error[0], 2) + np.power(error[1], 2))
                        print(rho, alpha, theta)
                        adelantar(rho, alpha)
                        rate.sleep()

            msj.data = [0, 0]
            pub.publish(msj)
            hayRuta = False

            if dibujo:
                figura = 'pez'
                hacer_dibujo([x, y], figura)
                dibujo = False

            if pintar:
                solicitarRuta([x, y], pintar)
                pintar = False
                dibujo = True
            rate.sleep()




if __name__ == '__main__':
    traccion_OP()
