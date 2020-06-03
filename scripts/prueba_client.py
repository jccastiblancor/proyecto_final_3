#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from proyecto_final_3.srv import Navegacion, GridmapPoints

def main():

    """
    main del servicio navegacion
    """
    rospy.init_node('nodo_test')

    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

    '''
    navegacion = rospy.ServiceProxy('navegacion', Navegacion)

    metodo = "d"

    inicio = [6.235, 6.57]
    destino = [-2.331, -6.75]
    resp = navegacion(metodo, inicio, destino)

    # print('rutax: "{}"'.format(resp.rutax))
    # print('rutay: "{}"'.format(resp.rutay))

    inicio = [-2.331, -6.75]
    destino = [3.366, -4.5]
    resp = navegacion(metodo, inicio, destino)

    # print('rutax: "{}"'.format(resp.rutax))
    # print('rutay: "{}"'.format(resp.rutay))
        
    
    '''




if __name__ == '__main__':
    main()