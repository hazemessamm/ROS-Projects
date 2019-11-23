#! /usr/bin/env python

import rospy
from my_custom_srv_msg_pkg.srv import MyCustomServiceMessage, MyCustomServiceMessageResponse
from bb8_move_circle_class import MoveBB8

def my_callback(request):
    rospy.loginfo("The Service move_bb8_in_circle has been called")
    movebb8_object = MoveBB8()
    movebb8_object.move_bb8(request.duration)
    rospy.loginfo("Finished service move_bb8_in_circle")
    response = MyCustomServiceMessageResponse()
    response.success = True
    return response

rospy.init_node('service_move_bb8_in_circle_server') 
my_service = rospy.Service('/move_bb8_in_circle', MyCustomServiceMessage , my_callback)
rospy.loginfo("Service /move_bb8_in_circle Ready")
rospy.spin()



#you can run this by opening it in the first terminal using python bb8_move_server.py and in the second terminal write rosservice call /move_bb8_in_circle [TAB]+[TAB]
