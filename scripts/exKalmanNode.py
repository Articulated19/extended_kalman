#!/usr/bin/env python

import rospy
from exKalmanClass import ExKalman as exkFilter
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32MultiArray, String
from std_msgs.msg import Float32
from mapserver.srv import getMarkPos
import math

pub = rospy.Publisher('kalman_topic', Float32MultiArray, queue_size=1)

filter = exkFilter()
featureDetected = False
featureData = None
get_marking_pos = None

trailerX = trailerY = x = y = 0.0 
   

def featureCallback(data):
    global featureDetected
    global get_marking_pos
    global featureData
   
    rospy.loginfo("featuredata")
    lista = str(data).split(';')

    state = filter.getState() 

   # x = state.item(0)
   # y = state.item(1)
   
    #rospy.loginfo(""+str(float(lista[1])))
   
    res = get_marking_pos(int(float(lista[1])),int(x*1000),int(9000-y*1000))
    x1 = res.x
    y1 = res.y

    if x1 < 0 or y1 < 0:
        featureDetected = False
        featureData = None
    else:
        #rospy.loginfo("x: " + str(x1) + " y: " + str( y1))
        filter.setFeaturePos(x1/1000.0,y1/1000.0) 
        featureData = [float(lista[0][5:]), float(lista[1]), float(lista[2])]
        featureDetected = True
        

def ackermannCallback(data):
    #rospy.loginfo("got ackermann")
    global featureData
    global featureDetected
    global trailerX
    global trailerY
    
    filter.predict([data.steering_angle, data.speed*1.3])
    if featureDetected and featureData:
        filter.update([featureData[0],featureData[1],featureData[2],data.steering_angle, data.speed*1.3], featureDetected)
    else:
        filter.update([0,0,0,data.steering_angle,data.speed],featureDetected)
    featureData = None
    featureDetected = False

    state = filter.getConvertedState()
    mess = Float32MultiArray()
    mess.data = [x,y, trailerX, trailerY]
    #rospy.loginfo("theta: " + str(state.item(2)))

    pub.publish(mess)

def trailerCallback(msg):
    global trailerX
    global trailerY
    global x
    global y
    alpha = float(msg.data) * -1
    #rospy.loginfo(str(alpha))
    print("trailorsensor angle: " + str(alpha))
    l1 = 40
    l2 = 530
    state = filter.getConvertedState()
    #x = 1200 - float(state.item(0))
    #y = 1200 + float(state.item(1))
    x = float(state.item(0))
    y = float(state.item(1))
    theta = float(state.item(2))
    thetagrade = math.degrees(theta % (2 * math.pi))
    print("thetagrade: " + str(thetagrade))
    print("theta is: " + str(theta % (2 * math.pi)))
    trailerX = x - ((math.cos(theta) * l1) - (math.cos(theta + math.pi + math.radians(float(alpha))) * l2))
    trailerY = y + ((math.sin(theta) * l1) - (math.sin(theta + math.pi + math.radians(float(alpha))) * l2))
    print("========================\n")
    print("trailerX: " + str(trailerX))
    print("trailerY: " + str(trailerY))
    print("x: " + str(x))
    print("y: " + str(y))
    print("=======================\n")

def runKalman():
    rospy.init_node('exKalmanNode', anonymous=True)
    rospy.Subscriber("master_drive_throttle", AckermannDrive, ackermannCallback, queue_size=1)
    rospy.Subscriber("trailer_sensor", Float32, trailerCallback)

    # setup service handle
    global get_marking_pos
    #rospy.wait_for_service("get_marking_posistion")
    try:
        get_marking_pos = rospy.ServiceProxy('get_marking_position',getMarkPos )
        rospy.loginfo("service init ok..")
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        rospy.loginfo("service init failed")   
    
    
    rospy.Subscriber("feature_detection", String, featureCallback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    runKalman()

