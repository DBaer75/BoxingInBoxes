#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import ApplyJointEffort
from gazebo_msgs.srv import GetJointProperties
import curses
import time as t

speedLimit = 5
effortStrength = 1

#set up joint rospy
msg_topic = '/gazebo/apply_joint_effort'
joint_left = 'DD_robot::left_wheel_hinge'
joint_right = 'DD_robot::right_wheel_hinge'
msg_topic_feedback = '/gazebo/get_joint_properties'
pub_feedback = rospy.ServiceProxy(msg_topic_feedback, GetJointProperties)
rospy.init_node('DD_ctrl', anonymous=True)
pub = rospy.ServiceProxy(msg_topic, ApplyJointEffort)

start_time = rospy.Time(0,0)
f = float(20)
T = 1/f
Tnano = int(T*1000000000)
end_time = rospy.Time(0,Tnano)
duration = end_time-start_time
rate = rospy.Rate(f)

def decideEffort(wheelRate, wheelStatus):
    #fixed acceleration up to requested speed
    if wheelRate<(speedLimit*wheelStatus):
        wheelEffort = effortStrength
    #fixed decelleration down to requested speed
    if wheelRate > (speedLimit*wheelStatus):
        wheelEffort = -effortStrength
    return wheelEffort

try:
    #set up curses
    stdscr = curses.initscr() 
    curses.cbreak()
    curses.noecho()
    curses.halfdelay(6) #sets to trigger exception after no input for (TenthsSeconds)

    #screen to show current command
    statusScr = curses.newwin(10,35,2,45)

    #create screan for header
    statusHeader = curses.newwin(1,35,0,45)
    statusHeader.clear()
    statusHeader.addstr(0,5,'CURRENT COMMAND')
    statusHeader.refresh()

    #create screen to show input instructions
    instructionsScr = curses.newwin(10,35,0,0)
    instructionsScr.clear()
    instructionsScr.addstr(0,10,'COMMANDS')
    instructionsScr.addstr(3,10,'SPACE = stop')
    instructionsScr.addstr(2,10,'w = forward')
    instructionsScr.addstr(3,0,'a = left')
    instructionsScr.addstr(3,25,'d = right')
    instructionsScr.addstr(4,10,'s = back')
    instructionsScr.addstr(6,10,'q = quit')
    instructionsScr.refresh()


    #main loop
    prevKey  = -1
    wheelL = 0
    wheelR = 0
    while True:
        try:
            currKey = statusScr.getch()
        except KeyboardInterrupt:
            break
        except:
            currKey = -1  
        if (currKey == ord('q')):
                break  # Exit the while loop
        if (prevKey != currKey):
            statusScr.clear()
            if (currKey == -1)|(currKey == 32): #spacebar or catch no input if exception
                statusScr.addstr(1,10,'stop')
                wheelL = 0
                wheelR = 0
            elif (currKey == ord('w')):
                #print('fwd')
                statusScr.addstr(0,10,'forward')
                wheelL = 1
                wheelR = 1
            elif (currKey == ord('a')):
                #print('lft')
                statusScr.addstr(1,0,'left')
                wheelL = -0.5
                wheelR = 0.5
            elif (currKey == ord('s')):
                #print('bwd')
                statusScr.addstr(2,10,'back')
                wheelL = -1
                wheelR = -1
            elif (currKey == ord('d')):
                #print('rtt')
                statusScr.addstr(1,25,'right')
                wheelL = 0.5
                wheelR = -0.5
            statusScr.refresh()
            prevKey = currKey    

        #control
        valLeft = pub_feedback(joint_left)
        valRight = pub_feedback(joint_right)   

        effortL = decideEffort(valLeft.rate[0], wheelL)
        effortR = decideEffort(valRight.rate[0], wheelR)

        pub(joint_left, effortL, start_time, duration)
        pub(joint_right, effortR, start_time, duration)

        #debug printing
        #if valLeft.rate[0]>0:
        #    leftWheelDir = 'forward'
        #else:
        #    leftWheelDir = 'reverse'
        #if valRight.rate[0]>0:
        #    RightWheelDir = 'forward'
        #else:
        #    RightWheelDir = 'reverse'
        #print('Left wheel speed is:', valLeft.rate[0], 'in the ', leftWheelDir, ' direction', 'effort is ', str(effortL))
        #print('Right wheel speed is:', valRight.rate[0], 'in the ', RightWheelDir, ' direction', 'effort is ', str(effortR))


        #rate.sleep()

                 
finally:
    #shut down curses
    curses.nocbreak()
    curses.echo()
    curses.endwin()
