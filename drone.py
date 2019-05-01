#! /usr/bin/env python
# -*- coding:utf-8 -*-

"""
    Para funcionar o drone ja ter decolado
    Opere o aparelho via teleop

"""


import rospy

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry


topico_odom = "/bebop/odom"

x = -1000
x0 = 0
y = -1000
y0 = 0
z = -1000
z0 = 0
distancia = None
inicial = True

def recebeu_leitura(dado):
    """
        Grava nas variáveis x,y,z a posição extraída da odometria
        Atenção: *não coincidem* com o x,y,z locais do drone
    """
    global x
    global x0
    global y 
    global y0
    global z 
    global z0
    global distancia
    global inicial

    x = dado.pose.pose.position.x
    y = dado.pose.pose.position.y
    z = dado.pose.pose.position.z

    if inicial:
        x0 = x
        y0 = y
        z0 = z
        inicial = False

    distancia = ((x0 - x)**2 + (y0 - y)**2 + (z0 - z)**2)**(0.5)



if __name__=="__main__":

    rospy.init_node("print_odom")

    # Cria um subscriber que chama recebeu_leitura sempre que houver nova odometria
    recebe_scan = rospy.Subscriber(topico_odom, Odometry , recebeu_leitura)

    empty_msg = Empty()


    pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
    takeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size = 1, latch=True)
    landing = rospy.Publisher('bebop/land', Empty, queue_size = 1, latch=True)



    try:

        rospy.sleep(3.0)
        takeoff.publish(empty_msg)
        rospy.sleep(5.0)

        while not rospy.is_shutdown():
           # velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            #velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
            #velocidade_saida.publish(velocidade)
            print("x {} y {} z {}".format(x, y, z))

            
            if distancia <= 4.8:
                velocidade = Twist(Vector3(0.2, 0, 0), Vector3(0, 0, 0))
                pub.publish(velocidade)
                rospy.sleep(0.2)
                print(distancia)
            else:
                velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
                pub.publish(velocidade)
                rospy.sleep(1)

                rospy.sleep(3.0)
                landing.publish(empty_msg)
                rospy.sleep(5.0)
                break

        velocidade = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        pub.publish(velocidade)
        rospy.sleep(1)

        



    except rospy.ROSInterruptException:
        pub.publish(velocidade)
        rospy.sleep(1.0)