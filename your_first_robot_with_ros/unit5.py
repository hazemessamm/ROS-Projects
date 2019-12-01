
Building and Rosifying a Robot from Scratch


Unit 4: Creating the motor drivers
SUMMARY

Estimated time to completion: 3 hours

In this unit, you are going to see how to create the motor drivers that will interact with the real ROSbots robot, in order to be able to drive it.

END OF SUMMARY

Basic Usage of motor
The L9110 motor driver has 2 channels — fancy talk to say it has the ability to control 2 motors. There are 2 input control pins per channel, IA and IB, that “drive” the 2 output pins, OA and OB, which are connected to the ROSbots robot’s motors. Hence, there are 4 input pins and 4 output pins total.

There is a set of IA/IB/OA/OB pins for one channel A (which is connected to our left motor/wheel), and another for channel B (which is connected to the right motor/wheel).

The IA pins are the ones we use in order to modulate the motor speed, and the IB pins are used to control the motor's direction.

The following pins table summarizes the input/output behavior of the L9110 motor:

IA (PWM)	IB (Motor Direction)	OA	OB	Motor Behavior
L	L	L	L	Off
H	L	H	L	Forward
L	H	L	H	Reverse
H	H	H	H	Off
Let's do a quick exercise to test this behavior!

**Exercise 1.1**


a) Place the wire connected to A-IA, specifically G-6 on your ROSbots’ breadboard, into any hole on the top row of the breadboard (VCC).

b) Place the wire connected to A-IB, specifically G-5, into any hole on the bottom row of the breadboard (GND).

c) The left wheel will spin forward.

d) Now swap the connections — G-6 into GND, and G-5 into VCC, now the left wheel will spin backwards.

from IPython.display import YouTubeVideo
# Cubli Robot created By ETHZurich
# Video credit: William Stein.
YouTubeVideo('-39QyiIS3j4')

**End of Exercise 1.1**

Understanding the drivers code
We will drive the L9110 motor using the UNO board mounted on our ROSbots robot. For that, we will write a ROS-capable Arduino Sketch for it.

The main.cpp sketch code can be found in rosbots_driver Github repo: https://github.com/ROSbots/rosbots_driver/blob/master/platformio/rosbots_firmware/examples/motor_driver/src/main.cpp

**main.cpp**

#include <ros.h>
#include "Arduino.h"
​
#include <std_msgs/Float32.h>
​
ros::NodeHandle nh;
​
#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif
​
#define M_LEFT_PWM 6
#define M_LEFT_FR 7
#define M_RIGHT_PWM 5
#define M_RIGHT_FR 4
​
void turnWheel( const std_msgs::Float32 &wheel_power,
                unsigned int pwm_pin,
                unsigned int fr_pin ) {
    float factor = max(min(wheel_power.data, 1.0f), -1.0f);
    if( factor >= 0 ) {
        digitalWrite(fr_pin, LOW);
        analogWrite(pwm_pin, (unsigned int)(255 * factor));
    } else {
        digitalWrite(fr_pin, HIGH);
        analogWrite(pwm_pin, (unsigned int)(255 * (1.0f + factor)));
    }   
}
void rightWheelCb( const std_msgs::Float32 &wheel_power ) {
    //nh.loginfo("Wheel Power - Right");
    //char result[8];
    //dtostrf(wheel_power.data, 6, 2, result); 
    //nh.loginfo(result);
    turnWheel( wheel_power, M_RIGHT_PWM, M_RIGHT_FR );
}
void leftWheelCb( const std_msgs::Float32 &wheel_power ) {
    //nh.loginfo("Wheel Power - Left");
    turnWheel( wheel_power, M_LEFT_PWM, M_LEFT_FR );
}
ros::Subscriber<std_msgs::Float32> sub_right("wheel_power_right",
                                            &rightWheelCb );
ros::Subscriber<std_msgs::Float32> sub_left("wheel_power_left",
                                           &leftWheelCb );
​
​
void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
​
  pinMode(M_LEFT_PWM, OUTPUT);
  pinMode(M_LEFT_FR, OUTPUT);
  pinMode(M_RIGHT_PWM, OUTPUT);
  pinMode(M_RIGHT_FR, OUTPUT);
​
  // Init motors to stop
  digitalWrite(M_LEFT_FR, LOW);
  digitalWrite(M_RIGHT_FR, LOW);
  analogWrite(M_LEFT_PWM, 0);
  analogWrite(M_RIGHT_PWM, 0);
​
  nh.initNode();
  nh.subscribe(sub_right);
  nh.subscribe(sub_left);
}
​
void loop()
{
  //nh.loginfo("Log Me");
  nh.spinOnce();
​
  // wait for a second
  //delay(1000);
}
**main.cpp**

Within the code, you can notice that the right wheel’s IA (PWM) and IB (direction) pins are connected to the pins 5 and 4 respectively on the UNO board. Also, left wheel’s IA and IB are connected to pins 6 and 7.

#define M_LEFT_PWM 6
#define M_LEFT_FR 7
#define M_RIGHT_PWM 5
#define M_RIGHT_FR 4
And here you can see how the pins should be connected on your board.



Foto cogidadel blog

Next, let’s have a look at the block of code that actually drives the pins to turn the wheels.

void turnWheel( const std_msgs::Float32 &wheel_power,
                unsigned int pwm_pin,
                unsigned int fr_pin ) {
    float factor = max(min(wheel_power.data, 1.0f), -1.0f);
    if( factor >= 0 ) {
        digitalWrite(fr_pin, LOW);
        analogWrite(pwm_pin, (unsigned int)(255 * factor));
    } else {
        digitalWrite(fr_pin, HIGH);
        analogWrite(pwm_pin, (unsigned int)(255 * (1.0f + factor)));
    }   
}
void rightWheelCb( const std_msgs::Float32 &wheel_power ) {
    //nh.loginfo("Wheel Power - Right");
    //char result[8];
    //dtostrf(wheel_power.data, 6, 2, result); 
    //nh.loginfo(result);
    turnWheel( wheel_power, M_RIGHT_PWM, M_RIGHT_FR );
}
void leftWheelCb( const std_msgs::Float32 &wheel_power ) {
    //nh.loginfo("Wheel Power - Left");
    turnWheel( wheel_power, M_LEFT_PWM, M_LEFT_FR );
}
In the turnWheel(…) function, we clamp the input variable wheel_power between -1.0 (full reverse) and 1.0 (full forward). That modulates the PWM voltages that output to the respective L9110’s IA pin (aka pwm_pin ) and the digital voltage output to the IB pin (aka fr_pin, as in fwd-reverse).

The simple math during the analogWrite(…) linearly adjusts the PWM pulse width according to the direction specified by the sign of wheel_power: positive means forward, negative means reverse.

As the wheel_power goes from -1.0 to 1.0, you can see the pwm_pin and fr_pin transition from full reverse, to stop, to full forward as described by the L9110 pin table in the beginning of the notebook.

Let’s compile and upload this sketch code to our UNO board to see it in action. ROSbots uses platformio to compile and upload ROS-Arduino sketch codes to the UNO board.

**Exercise 1.2**


a) We’ll assume you got your ROSbots robot set up with the instructions explained in the previous notebook, Connecting to your ROSbots real robot.

b) SSH into your ROSbots’ Raspberry Pi — we’ll be doing the remaining steps on the RPi.

c) Run the following in your RPi terminal (feel free to cut-n-paste directly):

upload_firmware ~/gitspace/rosbots_driver/platformio/rosbots_firmware/examples/motor_driver
d) On our RPi, type rosnode list to get a list of ROS nodes running — you should see the following:

Execute in WebShell #1

rosnode list
You should get the following nodes:

/rosout
/uno_serial_node
e) Type rostopic list to get a list of ROS topics being published or subscribed to — you should see the following:

Execute in WebShell #1

rostopic list
You should get the following topics:

/diagnostics
/rosout
/rosout_agg
/wheel_power_left
/wheel_power_right
**End of Exercise 1.2**

Inspired by the publisher/subscriber distributed messaging architecture, ROS is set up as a bunch of independent processes known as ROS nodes that communicate to each other via messages. One ROS node can be running the process that reads out of the Pi camera, then subsequently publish the images as an “image” message type out on a “foo” topic channel. Another ROS node who’s responsible for detecting cats in incoming images will subscribe to the “foo” topic channel. Like the standard pub/sub model, publishers and subscribers do not care about who has subscribed or who is publishing upstream and downstream.

If you are not familiar with this ROS architecture, you can have a look at our ROS Basics in 5 Days Course, where will learn everything you need in order to understand how ROS works.

In summary, any ROS node can be a publisher and/or subscriber of a specific message type communicated through a named channel known as a ROS topic.

In our example here, we wrote some Arduino sketch code to be a ROS node that subscribes to two topics: wheel_power_right and wheel_power_left. The message on these channels will be Float32 types (versus “images” in the hypothetical example above).

Take a look at this piece of the Arduino sketch code you compiled and uploaded:

ros::Subscriber<std_msgs::Float32> sub_right("wheel_power_right",
                                            &rightWheelCb );
ros::Subscriber<std_msgs::Float32> sub_left("wheel_power_left",
                                           &leftWheelCb );
and

nh.subscribe(sub_right);
nh.subscribe(sub_left);
You’ll notice that our UNO board subscribing to the two topic channels, wheel_power_right and *wheel_power_left * with these 2 blocks of code.

When we get a message on either of these topics, we call the rightWheelCb(…) and leftWheelCb(…) callback respectively. That callback, if you recall, implemented the code to drive the L9110’s input pins.

If you remember from the previous Exercise 1.2, when you typed rostopic list, the topic names wheel_power_right and wheel_power_left were listed.

rosserial
The sketch code we have just seen it's on our UNO board. But how does our UNO board communicate to the rest of the ROS system running on the Raspberry Pi?

The UNO board is connected to our RPi via the USB and communicates to it via serial port. That’s all fine and dandy except ROS does not directly talk “serial port” language!?

If you remember again, from the Exercise 1.2, when you typed rosnode list, you saw an /uno_serial_node node listed as one of the running ROS nodes. In order for ROS to be able to talk “serial port”, we need a bridge that serializes the ROS messages into serial port packets sent to our UNO to process. This bridge application is the ROS node known as rosserial, although we named the node /uno_serial_node. You can check for more information about rosserial in the following links:

http://wiki.ros.org/rosserial
http://wiki.ros.org/rosserial_arduino/Tutorials
The /uno_serial_node job is to open a serial port connection to the UNO board and then serialize/deserialize ROS messages to and from the UNO board.

Type the following command:

Execute in WebShell #1

rostopic info /wheel_power_left
And you’ll see /uno_serial_node as the subscriber. Why doesn’t it say our UNO code?

Well remember our UNO board cannot talk to the ROS system directly. It only can talk with the /uno_serial_node via the serial port. Our UNO code does specify to the /uno_serial_node that it wants to subscribe to the /wheel_power_left topic and the /uno_serial_node subscribes to that topic on the UNO’s behalf.

You can think of the rosserial /uno_serial_node as the mediator between the ROS world and the Arduino Sketch world.
Freat! So the UNO board is a subscriber to the topics /wheel_power_left and /wheel_power_right (via rosserial /uno_serial_node), which will drive the wheels depending on the Float32 message received. But who is the publisher?

Well, it's no one for now. Instead we will use the ROS rostopic tool to manually publish some ROS messages on these topic channels to drive the wheels.

**Exercise 1.3**


a) From any directory, it doesn’t matter, type the following:

Execute in WebShell #1

rostopic pub -1 /wheel_power_left std_msgs/Float32 '{data: 1.0}'
Your left wheel should spin forward

b) Now, type the following:

Execute in WebShell #1

rostopic pub -1 /wheel_power_left std_msgs/Float32 '{data: 0.0}'
Your left wheel should stop.

c) Now, type the following:

Execute in WebShell #1

rostopic pub -1 /wheel_power_left std_msgs/Float32 '{data: -1.0}' 
Your left wheel should spin backwards

d) Now, type the following:

Execute in WebShell #1

rostopic pub -1 /wheel_power_left std_msgs/Float32 '{data: -0.5}'
Your left wheel should spin backwards slower.

**End of Exercise 1.3**

The rostopic pub command tells ROS we want to publish. If you type the following command:

Execute in WebShell #1

rostopic pub --help
You’ll see a couple of options.

The -1 option says just publish a message once versus continuously.
The next option is the topic channel name. std_msgs/Float32 is the message type. '{data: 1.0}' is the actual payload data. It’s in a JSON-like curly brace syntax because ROS generalizes messages as data structures. So the Float32 ROS message type defined from a bunch of standard ROS package of std_msgs (notice Float32 listed in the link). The Float32 message type is actually a data structure of one entity, a 32-bit float.
from IPython.display import YouTubeVideo
# Cubli Robot created By ETHZurich
# Video credit: William Stein.
YouTubeVideo('-baQFAxz-Ss')

Understanding differential drive motion
In the following section, we will describe the kinematics and the dynamics of the ROSbots’ differential drive system. We’ll then apply this understanding and implement the ROS code to predictably drive the robot in a remote-control (RC) tele-operational manner.

Since a wheeled robot cannot fly, we only care about these 3 states that define it’s pose:

x - position on the x-axis (in meters)
y - position on the y-axis (in meters)
φ - phi - angle of the unicycle counter clockwise from x-axis (in radians)
For a unicycle model, there are 2 inputs that affect these 3 states of the robot:

v = forward velocity (ie meters per second)
w = angular velocity (ie radians per second)


While wheeled robot can have any number of wheels and more complicated factors that can affect its pose, we conveniently use a unicycle model to describe the dynamics of most wheeled robot since it is easy and intuitive to understand. Specifically, we intuitively use v** and **w to describe how the unicycle will move.

This is all fine and dandy, but our ROSbots robot is a differential drive robot with 2 inputs for each of its two wheels:

v_r = clockwise angular velocity of right wheel (in radians per second)
v_l = counter-clockwise angular velocity of left wheel (im radians per second)
Fortunately for us, we can convert unicycle inputs into differential drive inputs. Before we describe the equations to do the conversion, there are a couple of measurements we need to make with a straight edge.



L = wheelbase (in meters per radian)
R = wheel radius (in meters per radian)
Because R is a measurement of the radius of a wheel, it makes sense to think of R as meters per radian.

For L, it is not as intuitive. In the differential drive kinematics model, you can think of L as the radius of the circle drawn by one wheel spinning while holding the other wheel still. So L is also in the units of meters per radian where the radius is of that circle’s.



We won’t go into excruciating details (since this is not a post on kinematics), but in summary, we can use the kinematics for a unicycle model and kinematics for a differential drive model to come up with the following equations to convert unicycle v and w inputs into v_r and v_l differential drive inputs for our ROSbots robot.

v_r = ((2 * v) + (w * L)) / (2 * R)
v_l = ((2 * v) - (w * L)) / (2 * R)
The numerator for both equations are in meters per second. The denominator is in meters per radian. Both v_r and v_l result in radians per second, clock-wise and counter-clock-wise respectively— what we would expect.

Unicycle to Differential Drive Conversion in ROS
The two equations above are our sweetheart equations which we implement in a ROS node on our ROSbots robot.

To take our robot on a test spin, we will drive our robot in a remote-control (RC) tele-operational manner, with v and w inputs you specify through keyboard commands. The keyboard interface is through a teleop_twist_keyboard ROS node application created by the ROS community.

**Exercise 1.4**


a) SSH into your Raspberry Pi (RPi) on your ROSbots robot. From your RPi run:

Execute in WebShell #1

update_rosbots_git_repos
That should pick up the latest ROSbots repository source files from Github.

b) We’ll also assume you are picking up right where we left off from the Part 1 post. Specifically, you have the motor_driver firmware compiled and uploaded on the ROSBots’ UNO board via:

Execute in WebShell #1

upload_firmware ~/gitspace/rosbots_driver/platformio/rosbots_firmware/examples/motor_driver/
As a sanity check, here are the running ROS nodes if you run rosnode list:

/rosout
/uno_serial_node
And here are the ROS topics if you run rostopic list:

/diagnostics
/rosout
/rosout_agg
/wheel_power_left
/wheel_power_right
c) Now run let’s run the unicycle to diff drive model robot code. Again, from your RPi SSH terminal run:

Execute in WebShell #1

rosrun rosbots_driver part2_cmr.py
If you should see the following outputs, the ROS node is working:

[INFO] [1521228417.529282]: /part2_cmr RCTeleop initialized
[INFO] [1521228417.595446]: /part2_cmr wheelbase: 0.14 wheel radius: 0.035
[INFO] [1521228417.596918]: /part2_cmr v: 0 w: 0
[INFO] [1521228417.598303]: /part2_cmr vl: 0.0 vr: 0.0
[INFO] [1521228417.605237]: /part2_cmr right power: data: 0.0 left power: data: 0.0
[INFO] [1521228418.098403]: /part2_cmr v: 0 w: 0
[INFO] [1521228418.101346]: /part2_cmr vl: 0.0 vr: 0.0
[INFO] [1521228418.104728]: /part2_cmr right power: data: 0.0 left power: data: 0.0
...
d) Now open another SSH terminal for the RPi on your robot.

Let’s take a look at what additional ROS topics are published. From the 2nd SSH terminal on your RPi run $ rosnode list and you should see the new /part2_cmr ROS node listed:

Execute in WebShell #1

rosnode list
You should get the following nodes:

/part2_cmr
/rosout
/uno_serial_node
e) Now let’s see what other topics are added to the ROS system from running the /part2_cmr ROS node. From your 2nd SSH RPi terminal, type the following:

Execute in WebShell #1

rostopic list
You should get the following topics:

/part2_cmr
/part2_cmr/cmd_vel
/rosout
/uno_serial_node
Now type the following command:

Execute in WebShell #1

rostopic info /part2_cmr/cmd_vel
and you’ll get more info about the /part2_cmr/cmd_vel topic:

Type: geometry_msgs/Twist
Publishers: None
Subscribers: 
 * /part2_cmr (http://192.168.0.22:33183/)
This topic transports a ROS Twist message type. The Twist message is part of the standard geometry_msgs ROS package which includes a number of common messages used to describe robot geometry and kinematics.

If you look at the definition of the ROS Twist message, you’ll notice it actually can encode motion in 6 degrees of freedom (DOF) (3 as linear motion and 3 as angular). We actually only need 2 DOF’s (linear.x which is our v** and **angular.z which is our w.

Type the following command:

Execute in WebShell #1

rosmsg show geometry_msgs/Twist
and you’ll get more info about the Twist message type:

geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
f) From the previous step, you can notice that our /part2_cmr ROS node is a subscriber to the /part2_cmr/cmd_vel topic, but currently there are no publishers! No one is telling the robot what to do. Let’s add a publisher by running the teleop_twist_keyboard ROS node. From the 2nd SSH RPi terminal, type:

Execute in WebShell #1

rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/part2_cmr/cmd_vel
By default, the teleop_twist_keyboard ROS node publishes Twist messages to a topic named /cmd_vel. The /cmd_vel:=/part2_cmr/cmd_vel argument tells rosrun to remap the /cmd_vel name to /part2_cmr/cmd_vel so now the published topic name is the same for both the publishing teleop_twist_keyboard ROS node and the subscribing /part2_cmr ROS node.

Once you have the teleop_twist_keyboard ROS node running, let’s tune the linear and angular speed factors first. Hit the “x” button until you get to around speed 0.174 and hit the “e” key util you get turn 2.59. If you’re a 3 to 4 decimal places off, that’s fine:

...
currently: speed 0.17433922005 turn 2.5937424601
As described by the output from running teleop_twist_keyboard, use the lower case u i o j k l… keys to drive your ROSbots robot.

from IPython.display import YouTubeVideo
# Cubli Robot created By ETHZurich
# Video credit: William Stein.
YouTubeVideo('MIIfUBhKoxk')

**End of Exercise 1.4**

Code Analysis
Let's now have a look at the code we've using in the previous section

The main() function for the /part2_cmr ROS node is in the part2_cmr.py file, at the top level. Inside the main(), we create a Supervisor object and call its execute() function twice a second (2Hz) via:

rate = rospy.Rate(2)
​
while not rospy.is_shutdown():
    supervisor.execute()
    rate.sleep()
When the Supervisor is created, it does 3 things.

Creates a Robot object
Creates a DifferentialDrive object.
Creates all the Controller objects it needs for the defined objectives. In our case, we only need one Controller object, the RCTeleop controller.
Here you can have a look at the full code:

**part2_cmr.py**

#!/usr/bin/env python
​
import rospy
​
from controller.supervisor import Supervisor
    
def main():
    rospy.init_node('part2_cmr', anonymous=False)
    
    supervisor = Supervisor()
    
    rate = rospy.Rate(2)
​
    while not rospy.is_shutdown():
        supervisor.execute()
        rate.sleep()
​
​
if __name__ == '__main__':
    main()
**part2_cmr.py**

Controllers dictate what the robot does. For instance, you can create a Go-To-Goal controller to plan a path to go to a specific waypoint. You can create a Go-To-Angle controller to steer to and track at a specific angle orientation phi. Modern day cars have a Go-To-Velocity controller, aka cruise controller to accelerate/decelerate and track a specific velocity.

For the purposes of this section post, our supervisor will have a single RCTeleop controller that does whatever a teleoperational unicycle command dictates. Here you can have a look at the full rc_teleop.py file:

**rc_teleop.py**

#!/usr/bin/env python
​
import rospy
from geometry_msgs.msg import Twist
​
from controller import Controller
​
class RCTeleop(Controller):
    def __init__(self):
        rospy.loginfo(rospy.get_caller_id() + " RCTeleop initialized")
        rospy.Subscriber(rospy.get_name() + "/cmd_vel", Twist, self.twist_cb)
​
        self.v = 0
        self.w = 0
​
    def execute(self):
        #rospy.loginfo(rospy.get_caller_id() + " RCTeleop execute")
        output = {"v": self.v, "w": self.w}
        return output
​
    def shutdown(self):
        rospy.loginfo(rospy.get_caller_id() + " RCTeleop shutdown")
​
    def twist_cb(self, data):
        rospy.loginfo(rospy.get_caller_id() + \
                      ": Linear.x: %f -- Angular.z: %f", \
                      data.linear.x, data.angular.z)
        self.v = data.linear.x
        self.w = data.angular.z
**rc_teleop.py**

At init time, the RCTeleop subscribes to the /part2_cmr/cmd_vel topic and listens for Twist unicycle commands.

def __init__(self):
        rospy.loginfo(rospy.get_caller_id() + " RCTeleop initialized")
        rospy.Subscriber(rospy.get_name() + "/cmd_vel", Twist, self.twist_cb)
The self.twist_cb() callback function gets called when a Twist message comes in, and stores the v** and **w velocities away.

def twist_cb(self, data):
        rospy.loginfo(rospy.get_caller_id() + \
                      ": Linear.x: %f -- Angular.z: %f", \
                      data.linear.x, data.angular.z)
        self.v = data.linear.x
        self.w = data.angular.z
When the supervisor creates the Robot object, the Robot object fetches the dimensions of its wheelbase and wheel radius as well as the min and max velocities of its motors (yes, real robots have real physical limits) from the ROS parameter server.

self.wheelbase = rospy.get_param("wheelbase", default=0.14)
self.wheel_radius = rospy.get_param("wheel_radius", default=0.035)
​
# Wheel min and max no-load velocities in radians per sec
self.wheel_speed_min = rospy.get_param("wheel_speed/min", default=3.1)
self.wheel_speed_mid = rospy.get_param("wheel_speed/mid", default=4.4)
self.wheel_speed_max = rospy.get_param("wheel_speed/max", default=5.48)
Here you have a lookmat the full robots.py file:

**robot.py**

#!/usr/bin/env python
​
import rospy
from std_msgs.msg import Float32
​
class Robot:
    def __init__(self):
        # Diff drive robot attributes can be stored in parameter server
        # but otherwise a ROSbots dimensions are measured as the defaults
        # wheelbase of 140mm and wheel diameter of 70mm
        self.wheelbase = rospy.get_param("wheelbase", default=0.14)
        self.wheel_radius = rospy.get_param("wheel_radius", default=0.035)
​
        # Wheel min and max no-load velocities in radians per sec
        self.wheel_speed_min = rospy.get_param("wheel_speed/min", default=3.1)
        self.wheel_speed_mid = rospy.get_param("wheel_speed/mid", default=4.4)
        self.wheel_speed_max = rospy.get_param("wheel_speed/max", default=5.48)
        self.wheel_speed_min_power = \
            rospy.get_param("wheel_speed/min_power", default=0.5)
        self.wheel_speed_mid_power = \
            rospy.get_param("wheel_speed/mid_power", default=0.75)
        self.wheel_speed_max_power = \
            rospy.get_param("wheel_speed/max_power", default=1.0)
​
        # Publish out wheel power
        self.cur_wheel_power_right = Float32()
        self.cur_wheel_power_left = Float32()
        self.cur_wheel_power_right.data = 0.0
        self.cur_wheel_power_left.data = 0.0
        
        self.pub_power_right = \
            rospy.Publisher('/wheel_power_right', Float32, queue_size=10)
        self.pub_power_left = \
            rospy.Publisher('/wheel_power_left', Float32, queue_size=10)
        self.pub_power_right.publish(self.cur_wheel_power_right)
        self.pub_power_left.publish(self.cur_wheel_power_left)
​
        
    def shutdown(self):
        rospy.loginfo(rospy.get_caller_id() + " Robot shutdown")
        self.cur_wheel_power_right.data = 0.0
        self.cur_wheel_power_left.data = 0.0
        self.pub_power_right.publish(self.cur_wheel_power_right)
        self.pub_power_left.publish(self.cur_wheel_power_left)
        
​
    def velocity_to_power(self, v):
        av = abs(v)
​
        # If velocity is below minimum velocity turnable by PWM, then
        # just set to zero since the wheels won't spin anyway
        if av < self.wheel_speed_min:
            return 0.0
​
        a = b = a_pow = b_pow = None
        nnn = None
        if av >= self.wheel_speed_min and av < self.wheel_speed_mid:
            a = self.wheel_speed_min
            a_pow = self.wheel_speed_min_power
            b = self.wheel_speed_mid
            b_pow = self.wheel_speed_mid_power
        elif av >= self.wheel_speed_mid and av <= self.wheel_speed_max:
            a = self.wheel_speed_mid
            a_pow = self.wheel_speed_mid_power
            b = self.wheel_speed_max
            b_pow = self.wheel_speed_max_power
        
        # Linearly interpolate a and b
        nnn = ((av - a)/(b - a))
        wheel_power = ((nnn * (b_pow - a_pow)) + a_pow)
​
        if False:
            rospy.loginfo(rospy.get_caller_id() + ": " + str(a) + "," + str(b) +
                          "," + str(a_pow) + "," + str(b_pow))
            rospy.loginfo(rospy.get_caller_id() + " av: " + str(av))
            rospy.loginfo(rospy.get_caller_id() + " nnn: " + str(nnn))
            rospy.loginfo(rospy.get_caller_id() +
                          " wheel_power: " + str(wheel_power))
​
        assert(wheel_power <= 1.0)
        assert(wheel_power >= 0.0)
​
        # Negate if necessary
        if v < 0:
            wheel_power *= -1.0
​
        return wheel_power
​
    def set_wheel_speed(self, vr, vl):
        # Clamp the wheel speeds to actuator limits
        vr = max(min(vr, self.wheel_speed_max), self.wheel_speed_max * -1.0)
        vl = max(min(vl, self.wheel_speed_max), self.wheel_speed_max * -1.0)
​
        # Convert to power norms
        self.cur_wheel_power_right.data = self.velocity_to_power(vr)
        self.cur_wheel_power_left.data = self.velocity_to_power(vl)
​
        # Publish out
        if self.cur_wheel_power_right.data != 0.0 or \
           self.cur_wheel_power_left.data != 0.0:
            rospy.loginfo(rospy.get_caller_id() +
                          " right power: " + str(self.cur_wheel_power_right) +
                          " left power: " + str(self.cur_wheel_power_left))
        self.pub_power_right.publish(self.cur_wheel_power_right)
        self.pub_power_left.publish(self.cur_wheel_power_left)
**robot.py**

When the supervisor creates a DifferentialDrive object, it passes the wheelbase and wheel radius measurements so the DifferentialDrive object can accurately convert unicycle forward and angular velocity inputs to differential drive wheel velocity inputs when asked to do so.

Basically, each time the supervisor.execute() gets called, the supervisor does the following:

Executes the current controller (the RCTeleop controller) to get the unicycle v** and **w velocities.
Calls the DifferentialDrive to convert the unicycle v*, *w velocities to the v_r and v_l wheel velocities.
Passes the v_r and v_l velocities to the Robot who publishes the Float32 ROS messages to the /wheel_power_left and /wheel_power_right ROS topics (directing our UNO board to turn ROSbots’ respective wheels) as described in the previous section of this notebook.
def execute(self):
        # Get commands in unicycle model
        ctrl_output = self.current_controller.execute()
​
        # Convert unicycle model commands to differential drive model
        diff_output = self.dd.uni_to_diff(ctrl_output["v"], ctrl_output["w"])
​
        if ctrl_output["v"] != 0.0 or ctrl_output["w"] != 0.0:
            rospy.loginfo(rospy.get_caller_id() + " v: " +
                          str(ctrl_output["v"]) +
                          " w: " + str(ctrl_output["w"]))
            rospy.loginfo(rospy.get_caller_id() + " vl: " +
                          str(diff_output["vl"]) +
                          " vr: " + str(diff_output["vr"]))
        
        # Set the wheel speeds
        self.robot.set_wheel_speed(diff_output["vr"], diff_output["vl"])
**supervisor.py**

#!/usr/bin/env python
​
import rospy
​
from robot import Robot
from rc_teleop import RCTeleop
from dynamics.differential_drive import DifferentialDrive
​
class Supervisor:
    def __init__(self):
        rospy.on_shutdown(self.shutdown_cb)
​
        self.controllers = {"rc": RCTeleop()}
        self.current_state = "rc"
        self.current_controller = self.controllers[self.current_state]
​
        self.robot = Robot()
        rospy.loginfo(rospy.get_caller_id() +
                      " wheelbase: " + str(self.robot.wheelbase) +
                      " wheel radius: " + str(self.robot.wheel_radius))
        
        self.dd = DifferentialDrive(self.robot.wheelbase,
                                    self.robot.wheel_radius)
​
    def execute(self):
        # Get commands in unicycle model
        ctrl_output = self.current_controller.execute()
​
        # Convert unicycle model commands to differential drive model
        diff_output = self.dd.uni_to_diff(ctrl_output["v"], ctrl_output["w"])
​
        if ctrl_output["v"] != 0.0 or ctrl_output["w"] != 0.0:
            rospy.loginfo(rospy.get_caller_id() + " v: " +
                          str(ctrl_output["v"]) +
                          " w: " + str(ctrl_output["w"]))
            rospy.loginfo(rospy.get_caller_id() + " vl: " +
                          str(diff_output["vl"]) +
                          " vr: " + str(diff_output["vr"]))
        
        # Set the wheel speeds
        self.robot.set_wheel_speed(diff_output["vr"], diff_output["vl"])
        
    def shutdown_cb(self):
        for ctrl in self.controllers.values():
            ctrl.shutdown()
​
        self.robot.shutdown()
**supervisor.py**

The RCTeleop controller’s execute function simply returns back the current v** and **w it has stored away. Nothing exciting here.

def execute(self):
    #rospy.loginfo(rospy.get_caller_id() + " RCTeleop execute")
    output = {"v": self.v, "w": self.w}
    return output
The DifferentialDrive’s uni_to_diff(.) function implements the unicycle to differential drive model conversion. This is where our 2 sweetheart functions get implemented.

The Robot’s set_wheel_speed(.) function has a couple of important responsibilities.

It clamps the wheel velocity to the motor’s physical limitations.
Converts wheel velocity to wheel power. If you recall from Part 1, our motor_driver firmware only supports wheel power normalized between -1.0 and 1.0 — full speed back to full speed forward. The set_wheel_speed(.) function converts velocity into power using a lookup table of no-load speeds we measured on our ROSbots robot prior.
Our motors also has a minimum speed velocity. There is a minimum pulse width threshold where the motors just won’t turn. If the controller wants a speed lower than this threshold, the motors won’t be able to support it so currently we just set the motor speed to zero to save power.
