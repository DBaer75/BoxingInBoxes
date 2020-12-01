#! /usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import os
import time as t
import numpy as np

box_i = 0

boxProxy = None

def trigger_response(request):
    global box_i
    tick = t.time()

    a = getRobotLocation()

    dropBoxBool = False

    x = a[0]
    y = a[1]

    b = getBoxLocation(x,y)
    if (b != None):
        if not (checkBoxLocation(b[0],b[1])):
            dropBox(b[0],b[1])
            dropBoxBool = True
   
    tock = t.time()
    dt = tock - tick
    return TriggerResponse(
        success=dropBoxBool,
        message="dt = " + str(dt)
    )


    

def getBoxLocation(x,y):
    # x,y robot location
    
    #determine if outside of 50m square
    sSize = 50
    x0 = 0
    y0 = 0
    x_dist = (x-x0)
    y_dist = (y-y0)
    if ((np.abs(x_dist) < (sSize-10)) and (np.abs(y_dist) < (sSize-10))) : #change to square
        return None
    else:
        if (np.abs(x_dist) > np.abs(y_dist)): #
            if (x_dist > 0):
                xn = sSize
            else:
                xn = -sSize
            yn = y
        else:
            if (y_dist > 0):
                yn = sSize
            else:
                yn = -sSize
            xn = x
        return [xn, yn]

def delBox(bi):
    buff = "rosservice call gazebo/delete_model box" +str(bi) + " &"
    os.system(buff)

def delBoxAll(bi):
    for i in range(bi):
        delBox(i)
        t.sleep(0.1)


box_x = []
box_y = []

def saveBoxValues(x,y):
    global box_x, box_y
    box_x.append(x)
    box_y.append(y)


def checkBoxLocation(x,y):
    global box_x, box_y
    for i in range(len(box_x)):
        xx = box_x[i]
        yy = box_y[i]
        d = np.sqrt((xx-x)**2 + (yy-y)**2)
        if (d < 1.0): #change to square valuse
            return True
    return False


def dropBox(x,y):
    global box_i
    saveBoxValues(x,y)
    b0 = "/home/user/catkin_ws/src/BoxingInBoxes/boxing_in_boxes/src/./drop_box.sh" + " "
    b1 = "box" + str(box_i) + " "
    box_i += 1
    b2 = str(x) + " " #x value
    b3 = str(y) #y value
    b4 = "&"
    buff = b0 + b1 + b2 + b3 + b4
    os.system(buff)


def getRobotLocation():
    a = GetModelStateRequest(model_name = 'DD_robot')
    s = boxProxy(a)
    x = s.pose.position.x
    y = s.pose.position.y
    return [x,y]

delBoxAll(20)
rospy.init_node('service_server_box')
my_service = rospy.Service('/box', Trigger, trigger_response)


boxProxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)


rospy.spin()