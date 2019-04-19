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
from std_msgs.msg import UInt8
from sensor_msgs.msg import LaserScan

import argparse
import imutils
from imutils.video import VideoStream
from imutils.video import FPS

CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
	"bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
	"dog", "horse", "motorbike", "person", "pottedplant", "sheep",
	"sofa", "train", "tvmonitor"]

print("Categorias disponíveis: ")
print("\n")
print(CLASSES)
print("\n")
resp = True

while resp:
    entrance = raw_input("Escolha o objeto a ser seguido.: ")
    if entrance in CLASSES:
        resp = False
        
    else:
        print("Escolha uma categoria válida.")



bridge = CvBridge()
cv_image = None
follow = False
d = None
minimo = None
maximo = None
check_delay = False
localizar = True

contador = 0
front = 0
verde = 0
area = 0.0
med_follow = 0
atraso = 1.5E9

media = []
centro = []
resultados = list()

pos = np.array([])


velocidade = Twist(Vector3(0,0,0), Vector3(0,0,0))
vel = Twist(Vector3(0,0,0), Vector3(0,0,0))

(major, minor) = cv2.__version__.split(".")[:2]

if int(major) == 3 and int(minor) < 3:
	tracker = cv2.Tracker_create(["kcf"].upper())


else:
	
	OPENCV_OBJECT_TRACKERS = {
		"csrt": cv2.TrackerCSRT_create,
		"kcf": cv2.TrackerKCF_create,
		"boosting": cv2.TrackerBoosting_create,
		"mil": cv2.TrackerMIL_create,
		"tld": cv2.TrackerTLD_create,
		"medianflow": cv2.TrackerMedianFlow_create,
		"mosse": cv2.TrackerMOSSE_create
	}

	tracker = OPENCV_OBJECT_TRACKERS["kcf"]()

def sensor(dado):
    global d
    d = dado.data

def scaneou(dado):
    global dmin
    global pos
    global minimo
    global maximo
    global barreira
    
    minimo = dado.range_min
    maximo = dado.range_max
    dmin = np.amin(np.array(dado.ranges).round(decimals=2))

    pos = dado.ranges
	
	


def roda_todo_frame(imagem):
    global cv_image
    global media
    global centro
    global resultados
    global follow
    global area
    global xfollow0
    global xfollow1
    global yfollow0
    global yfollow1
    global med_follow
    global localizar
    global contador

    
    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime
    delay = lag.nsecs
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        centro, imagem, resultados =  visao_module.processa(cv_image)
        media, centro, area =  cormodule.identifica_cor(cv_image)
        
        if localizar:
            for i in resultados:
                
                if i[0] == entrance:
                    xfollow0 = i[2][0]
                    xfollow1 = i[3][0]
                    yfollow0 = i[2][1]
                    yfollow1 = i[3][1]

                    contador += 1
                    
                    if contador == 20:
                        localizar = False
       
        
    #Tracking
        if not localizar:
             
            width, height = cv_image.shape[:2]
            objeto = (xfollow0, yfollow0, (xfollow1-xfollow0), (yfollow1-yfollow0))
            localizar = False
            
            if objeto is not None:
                tracker.init(cv_image, objeto)
                fps = FPS().start()
                (success, box) = tracker.update(cv_image)
                

            if success:
                (x, y, w, h) = [int(v) for v in box]
                cv2.rectangle(cv_image, (x, y), (x + w, y + h),
                    (0, 255, 0), 2)
                med_follow = (x + x+w)/2
        
                follow = True

            fps.update()
            fps.stop()

            info = [
                ("Tracker", ["kcf"]),
                ("Success", "Yes" if success else "No"),
                ("FPS", "{:.2f}".format(fps.fps())),
                ]

            
            for (i, (k, v)) in enumerate(info):
                text = "{}: {}".format(k, v)
                cv2.putText(cv_image, text, (10, height - ((i * 20) + 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
        depois = time.clock()
        cv2.imshow("Camera", cv_image)

    except CvBridgeError as e:
        print('ex', e)


    
if __name__=="__main__":
    rospy.init_node("projeto")

    topico_imagem = "/kamera"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    recebe_bump = rospy.Subscriber("/bumper", UInt8, sensor)
    scan = rospy.Subscriber("/scan", LaserScan, scaneou)


    try:
        while not rospy.is_shutdown():
            #Laser
            for i in range (0, len(pos)):
                if pos[i] > minimo and pos[i] < maximo:
                    if pos[i] < 0.18:
                        if i <= 30:
                            pub.publish(Twist(Vector3(-0.2, 0, 0), Vector3(0, 0, 0)))
                            rospy.sleep(1.0)
                            continue
                                            

                        if i > 30 and i <= 120:
                            pub.publish(Twist(Vector3(-0.1, 0, 0), Vector3(0, 0, -0.35)))
                            rospy.sleep(1.0)
                            continue
                            
                            
                        if i > 180 and i < 270:
                            pub.publish(Twist(Vector3(-0.1, 0, 0), Vector3(0, 0, 0.35)))
                            rospy.sleep(1.0)
                            continue

            #Bumper
            if d == 2:
                pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))) 
                pub.publish(Twist(Vector3(-0.1, 0, 0), Vector3(0, 0, 0.35)))
                rospy.sleep(2.0)
                pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))
                rospy.sleep(1.0)
                d = None
                continue
            

            if d == 1:
                pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))
                pub.publish(Twist(Vector3(-0.1, 0, 0), Vector3(0, 0, -0.35)))
                rospy.sleep(2.0)
                pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))
                rospy.sleep(1.0)
                d = None
                continue
                

            if d == 3:
                pub.publish(Twist(Vector3(0.2, 0, 0), Vector3(0, 0, 0)))
                rospy.sleep(1.0)
                pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))
                rospy.sleep(1.0)
                d = None
                continue
                

            if d == 4:
                pub.publish(Twist(Vector3(0.2, 0, 0), Vector3(0, 0, 0)))
                rospy.sleep(1.0)
                pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))
                rospy.sleep(1.0)
                d = None
                continue
            
            #Seguir objeto
            if d == None: 
                if follow:
                    if med_follow > (centro[1] - 40) and med_follow < (centro[1] + 40):
                        pub.publish(Twist(Vector3(0.21,0,0), Vector3(0,0,0)))
                        rospy.sleep(0.3)
                        follow = False
                        continue
                        


                    #Controle proporcional angular
                    
                    if med_follow < (centro[1]):
                        if centro[1] - med_follow > 20:
                            pub.publish(Twist(Vector3(0,0,0), Vector3(0,0,0.24)))
                            rospy.sleep(0.2)
                            
                        else:
                            pub.publish(Twist(Vector3(0,0,0), Vector3(0,0,0.02)))
                            rospy.sleep(0.2)
                        follow = False
                        continue
                            


                    if med_follow > (centro[1]): 
                        if (med_follow- centro[1] > 20):
                            pub.publish(Twist(Vector3(0,0,0), Vector3(0,0,-0.24)))  
                            rospy.sleep(0.2)
                        else:
                            pub.publish(Twist(Vector3(0,0,0), Vector3(0,0,-0.02)))  
                            rospy.sleep(0.2)
                        follow = False
                        continue
                        
            
            #Fugir verde
            if area > 1000:
                if d == None:
                    if media[0] > (centro[1] - 40) and media[0] < (centro[1] + 40):
                        pub.publish(Twist(Vector3(-0.3,0,0), Vector3(0,0,0)))
                        rospy.sleep(0.4)
                        continue
                        
                    if media[0] < centro[1] - 40:
                        pub.publish(Twist(Vector3(0,0,0), Vector3(0,0,0.3)))
                        rospy.sleep(0.2)
                        continue
                        
                    if media[0] > centro[1] + 40:
                        pub.publish(Twist(Vector3(0,0,0), Vector3(0,0,-0.3)))
                        rospy.sleep(0.2)
                        continue
                
            pub.publish(Twist(Vector3(0,0,0), Vector3(0,0,0)))
            rospy.sleep(0.1)
                    
                    

            
    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
