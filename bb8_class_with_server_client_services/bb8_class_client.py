import rospy

from my_custom_srv_msg_pkg.srv import MyCustomServiceMessage, MyCustomServiceMessageRequest


rospy.init_node('my_service_class_node_client')

rospy.wait_for_service('/move_bb8_in_circle')

my_service_object = rospy.ServiceProxy('/move_bb8_in_circle', MyCustomServiceMessage)

my_request_object = MyCustomServiceMessageRequest()

my_request_object.duration = 4

my_service_object(my_request_object)
