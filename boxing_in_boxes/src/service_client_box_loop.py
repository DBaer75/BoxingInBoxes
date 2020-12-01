#! /usr/bin/env python


import rospy
from std_srvs.srv import Trigger, TriggerRequest
from nav_msgs.srv import GetMap, GetMapRequest
import time as t



rospy.init_node('service_client_box_loop')

rospy.wait_for_service('/box')

my_service = rospy.ServiceProxy('/box', Trigger)

trigreq = TriggerRequest()


while True:
    #tick = t.time()
    result = my_service(trigreq)
    #tock = t.time()
    #dt = tock - tick
    #print "client dt = " + str(dt)
    t.sleep(0.02)

print result