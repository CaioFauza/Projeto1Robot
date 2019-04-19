#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import visao_module
import cormodule


bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
resultados = list()
bottle = False
xbottle0 = 0
xbottle1 = 0
verde = 0
area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 
vel = Twist(Vector3(0,0,0), Vector3(0,0,0))



# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    global cv_image
    global media
    global centro
    global resultados
    global bottle
    global xbottle0
    global xbottle1
    global area

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    #print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        centro, imagem, resultados =  visao_module.processa(cv_image)
        media, centro, area =  cormodule.identifica_cor(cv_image)
        for i in resultados:
            if i[0] == "bottle":
                if i[3][0] < i[3][1]:
                    xbottle0 = i[3][0]
                    xbottle1 = i[3][1]
                else:
                    xbottle0 = i[3][1]
                    xbottle1 = i[3][0]
                bottle = True
            
        
        depois = time.clock()
        cv2.imshow("Camera", cv_image)
        
    except CvBridgeError as e:
        print('ex', e)


    
if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/kamera"
    
    # Para renomear a *webcam*
    #   Primeiro instale o suporte https://github.com/Insper/robot19/blob/master/guides/debugar_sem_robo_opencv_melodic.md
    #
    #   Depois faça:
    #   
    #   rosrun cv_camera cv_camera_node
    #
    #   rosrun topic_tools relay  /cv_camera/image_raw/compressed /kamera
    #
    # 
    # Para renomear a câmera simulada do Gazebo
    # 
    #   rosrun topic_tools relay  /camera/rgb/image_raw/compressed /kamera
    # 
    # Para renomear a câmera da Raspberry
    # 
    #   rosrun topic_tools relay /raspicam_node/image/compressed /kamera
    # 

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    try:

        while not rospy.is_shutdown():
            if bottle:
                #print("X0 é: " + str(xbottle0))
                #print("X1 é: " + str(xbottle1))
                #print("Centro é: " + str(centro[0]))
                
                if centro[0] > xbottle0 and centro[0] < xbottle1:
                    vel = Twist(Vector3(0.07,0,0), Vector3(0,0,0))
                    bottle = False

                if centro[0] < xbottle0:
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.06))
                    bottle = False    

                if centro[0] > xbottle1:
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,0.06))    
                    bottle = False


                
                
            else:
                vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
            
            if area > 0:
                verde = True

            if verde:
                if media[0] > (centro[0]-40) and media[0] < (centro[0]+40):
                    vel = Twist(Vector3(-0.1,0,0), Vector3(0,0,0))
                    verde = False
                if media[0] < centro[0]-40:
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,0.2))
                    verde = False
                if media[0] > centro[0]+40:
                    vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.2))
                    verde = False

			

                    
            pub.publish(vel)
            rospy.sleep(1.0)
            

            
            
    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")


