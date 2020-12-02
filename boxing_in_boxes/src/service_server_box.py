#! /usr/bin/env python

#service server to track robot and calulate when and where a box should be dropped to trap robot

import rospy
from std_srvs.srv import Trigger, TriggerResponse #requests for service come through this message
from gazebo_msgs.srv import GetModelState, GetModelStateRequest #tracking of robot
import os
import time as t
import numpy as np

box_i = 0
boxProxy = None

def trigger_response(request):
    #called whenever requested by the client
    global box_i
    tick = t.time()

    #tracking the robot
    a = getRobotLocation() 
    x = a[0]
    y = a[1]

    dropBoxBool = False

    b = getBoxLocation(x,y) #gets the location for box drops
    if (b != None):
        #check if each drop location is already populated and if not drop a box
        if not (checkBoxLocation(b[0],b[1])):
            dropBox(b[0],b[1])
            dropBoxBool = True
        if not (checkBoxLocation(b[2],b[3])):
            dropBox(b[2],b[3])
            dropBoxBool = True
        if not (checkBoxLocation(b[4],b[5])):
            dropBox(b[4],b[5])
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
    if ((np.abs(x_dist) < (sSize-10)) and (np.abs(y_dist) < (sSize-10))) : 
        #robot is inside bounds
        return None
    else:
        #robot is near bounds
        #calculate drop location of box and one to each side to prevent escape
        if (np.abs(x_dist) > np.abs(y_dist)):
            #nearing max +-X allowed 
            if (x_dist > 0): #nearing +X
                xn1 = sSize
                xn2 = sSize
                xn3 = sSize
            else: #nearing -X
                xn1 = -sSize
                xn2 = -sSize
                xn3 = -sSize
            yn1 = y #centered on the robot
            yn2 = y + 1.0 #one block above the robot
            yn3 = y - 1.0  #one block below the robot
        else:
            #nearing max +-Y allowed 
            if (y_dist > 0): #nearing +Y
                yn1 = sSize
                yn2 = sSize
                yn3 = sSize
            else: #nearing -Y
                yn1 = -sSize
                yn2 = -sSize
                yn3 = -sSize
            xn1 = x
            xn2 = x + 1.0
            xn3 = x - 1.0
        return [xn1, yn1, xn2, yn2, xn3, yn3]

def delBox(bi):
    #deletes a single box with the number bi
    buff = "rosservice call gazebo/delete_model box" +str(bi) + " &"
    os.system(buff)

def delBoxAll(bi):
    #deletes all boxes below number bi
    for i in range(bi):
        delBox(i)
        t.sleep(0.1)

#lists to track where boxes have been already dropped
box_x = []
box_y = []


def saveBoxValues(x,y):
    #adds a box location to the list of dropped boxes
    global box_x, box_y
    box_x.append(x)
    box_y.append(y)


def checkBoxLocation(x,y):
    #checks if a location is within 1m of a box on the list of dropped boxes 
    global box_x, box_y
    for i in range(len(box_x)):
        xx = box_x[i]
        yy = box_y[i]
        d = np.sqrt((xx-x)**2 + (yy-y)**2)
        if (d < 1.0): #change to square valuse
            return True
    return False


def dropBox(x,y):
    #drop a box
    global box_i
    saveBoxValues(x,y) #add new box to the list of dropped boxes

    #build the command for calling the drop_box shell script
    b0 = "/home/user/catkin_ws/src/BoxingInBoxes/boxing_in_boxes/src/./drop_box.sh" + " "
    b1 = "box" + str(box_i) + " "
    box_i += 1 #iterate in order to keep boxes from having the same name
    b2 = str(x) + " " #x value
    b3 = str(y) #y value
    b4 = "&"
    buff = b0 + b1 + b2 + b3 + b4
    os.system(buff)#call the script


def getRobotLocation():
    #track the robot 
    a = GetModelStateRequest(model_name = 'DD_robot')
    s = boxProxy(a)
    x = s.pose.position.x
    y = s.pose.position.y
    return [x,y]


#delete previous boxes. 
#delBoxAll(40)

#create ros node for service server
rospy.init_node('service_server_box')
my_service = rospy.Service('/box', Trigger, trigger_response)

#create ros service proxy in order to be able to call service that tracks robot location
boxProxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

#keep open
rospy.spin()