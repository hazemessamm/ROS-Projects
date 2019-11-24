import rospy
import actionlib
from geometry_msgs.msg import Twist
from ardrone_as.msg import ArdroneAction, ArdroneGoal, ArdroneResult, ArdroneFeedback
from std_msgs.msg import Empty

nimage = 1
def feedback_callback(feedback):
        global nimage
        print('[feedback] image n.%d received'%nimage)
        nimage += 1
rospy.init_node('drone_action_client')
client = actionlib.SimpleActionClient('ardrone_action_server', ArdroneAction)
client.wait_for_server()

goal = ArdroneGoal()
goal.nseconds = 10

client.send_goal(goal, feedback_cb = feedback_callback)

current_state = client.get_state()

move = Twist()
move.linear.x = 0.2
move.angular.z = 0.3
my_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
rate = rospy.Rate(1)
takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
takeoff_msg = Empty()
landing = rospy.Publisher('/drone/land', Empty, queue_size = 1)
land_msg = Empty()

while current_state < 2:
        takeoff.publish(takeoff_msg)
        rospy.loginfo('Doing the action')
        my_pub.publish(move)
        rate.sleep()
        current_state = client.get_state()

landing.publish(land_msg)
rospy.loginfo('[Result] '+str(current_state))
if current_state == 3:
        rospy.logwarn('There is a warning in the server')
if current_state == 4:
        rospy.logerr('there is an error in the server')
