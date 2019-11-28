
ROS IN 5 DAYS
Course Project




ROSject Link: http://bit.ly/2LO3XZf

Package Name: sphero_gazebo

Launch File: main.launch

Estimated time to completion: 10 hours

What will you learn in this unit?

Practice everything you learn through the course
Put together everything you learn into a big project
Create a main program
Win the Sphero Race!
In this project, you will have to make a Sphero robot move through a maze faster than the other students. The fastest one will win a prize.

For this goal, you will have to apply all the things that you have learned throughout the course. It is really important that you complete this race because all of the structures that you create for this project will be asked about in our official exam.

Note: Our offical exam is only available for on-site and virtual classes at present.

Get the robot out of the maze as fast as possible and you will get the prize. The ideal would be for Spheros to come out cleanly, but it is possible that your robot may collide with the maze. You can use the collision detection to get data and help you in your strategy to get out.

Basically, in this project, you will have to:

Apply all the theory that is given in the course
Decide on a strategy to solve the problem
Implement this strategy in the simulation environment
Do as many tests as required in the simulation environment until it works
To achieve success in this project, please follow the 5 steps below that will provide clear instructions, and even solutions.

Also, remember to:

Create your packages and code in the simulation environment as you have been doing throughout the course.
Use the consoles to gather information about the status of the simulated robot.
Use the IDE to create your programs and execute them through the consoles, observing the results on the simulation screen. You can use other consoles to watch calls to topics, services, or action servers.
Everything that you create in this unit will be automatically saved in your space. You can come back to this unit at any time and continue with your work from where you left off.
Every time you need to reset the position of the robot, just press the restart button in the simulation window.
Use the debugging tools to try to find what is not working and why (for instance, the rviz tool is very useful for this purpose).
One final note: Because the program that you create should work with a real robot, if needed, you can't move the Sphero in a closed loop. This is because in reality, the circuit could be different, the robot could not be as responsive, there might be errors in the readings, and so on. So in this project, you should create a program that can cope with all of those uncertainties.

With "closed loop", we reffer that you cannot send some hard-coded velocity commands in order to make Sphero get out of the maze. You need to use its sensors in order to be aware of the environment it's in and, with that data, find out how to get out of the maze.

What does Sphero Provide to Program It?
So the main question is, what can you do with Sphero from a ROS programming point of view? Which sensors and actuators does Sphero provide that will allow you to do the maze test?

Good question! Sphero provides the following sensors and actuators:

Sensors
IMU sensor: Sphero has an Inertial Measurement Unit (IMU), which provides information about acceleration and orientation. The value of the sensor is provided through the /sphero/imu/data3 topic.
Odometry: The odometry of the robot can be accessed through the /odom topic.
Actuators
Speed: You can send speed commands to move the robot through the /cmd_vel topic.
Now that you know the relevant topics of the robot, it is your job to figure out the types of messages, and how to use them, in order to make the robot do what you want it to do.

Ideas to Start Working On
This is a list of things that you can start with. You do not have to follow them. They are provided just in case you don't know where to start.

Start watching some of the messages the sensor topics are publishing. Try to get an idea about the information that they are providing. Move the robot in the simulation and see how those messages change their values. It is very important that you understand how changes in the robot produce changes in the topics.
Try to move the robot by sending messages to the /cmd_vel (either through the console or through python programs).
Observe how the topic messages change when the robot hits an obstacle.
Is the odometry trustworthy? Can you move the robot the exact amount, even when it collides with something?
Steps You Should Cover
These are the steps that you should follow throughout the duration of the project. These steps will ensure that you have practised and created all of the structures that will be asked about in the final exam of this course. If you perform all of the steps mentioned here, you will find completion of the exam to be achievable.

Step 1: Read and Write Topics (Dedicate 2 hours)
Step 2: Use topics through Services (Dedicate 3 hours)
Step 3: Use topics through Actions (Dedicate 4 hours)
Step 4: Create a main program to manage everything (Dedicate 1 hour)
EXTRA Step: How to use python modules from different packages (Not required, included here for information purposes only)
NOTE 1: We provide the solutions for each step. Do not watch unless you are really stuck.

NOTE 2: The fifth step may not be required, but we have found that some students organize their code in such a way that they need this step. We have provided it here just in case you need it, but you don't have to use it.

Step 1: Read and Write Topics
This step requires 3 actions:

Create a package called my_sphero_topics that will contain all of the programs related to the topics
Create a topic publisher that allows you to move the Sphero.
Create two topic subscribers that extract the data that you need from Odometry and the IMU.
So, let's get start on them!

1. Create package my_sphero_topics, with rospy as dependency.

2. Create the Topic Publisher to move Sphero.

To move Sphero, you need to publish in the topic /cmd_vel.

It is important that you always encapsulate your topic subscribers and publishers inside classes. This will allow you to store values and manage the publishing callbacks easily.

First, you have to see if there is a topic like /cmd_vel running.

Note: It will not always be this simple. With real robots, you will need to access the code to see what the name of the topic is that moves the robot, or even use rostopic info /name_of_topic to know which one it could be.

Execute in WebShell #1

rostopic list
As you can see, there is a /cmd_vel topic.
You, then, have to extract the type of message that /cmd_vel uses.

Execute in WebShell #1

rostopic info /cmd_vel
Test it to see if it works py publishing different values:

Execute in WebShell #1

rostopic pub /cmd_vel message_type_of_cmd_vel [TAB][TAB]
Once you have the information, you are ready to create the python class.
Create a python file in the src folder of the package that you just created, "my_sphero_topics".
This file has to have not only the class, but also a way of testing that the class works.

Here is an example of a way in which this could be done (DO NOT watch unless you are stuck):

#! /usr/bin/env python
​
import rospy
from geometry_msgs.msg import Twist
​
​
class CmdVelPub(object):
    def __init__(self):
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._twist_object = Twist()
        self.linearspeed = 0.2
        self.angularspeed = 0.5
        
    def move_robot(self, direction):
        if direction == "forwards":
            self._twist_object.linear.x = self.linearspeed
            self._twist_object.angular.z = 0.0
        elif direction == "right":
            self._twist_object.linear.x = 0.0
            self._twist_object.angular.z = self.angularspeed
        elif direction == "left":
            self._twist_object.linear.x = 0.0
            self._twist_object.angular.z = -self.angularspeed
        elif direction == "backwards":
            self._twist_object.linear.x = -self.linearspeed
            self._twist_object.angular.z = 0.0
        elif direction == "stop":
            self._twist_object.linear.x = 0.0
            self._twist_object.angular.z = 0.0
        else:
            pass
        
        self._cmd_vel_pub.publish(self._twist_object)
​
​
if __name__ == "__main__":
    rospy.init_node('cmd_vel__publisher_node')
    cmd_publisher_object = CmdVelPub()
    
    rate = rospy.Rate(1)
    
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        global ctrl_c
        global twist_object
        global pub
        
        rospy.loginfo("shutdown time!")
        
        ctrl_c = True
        cmd_publisher_object.move_robot(direction="stop")
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        cmd_publisher_object.move_robot(direction="forwards")
        rate.sleep()
3. Create Two Topic Subscribers that extract the data that you need from the Odometry and the IMU.

To get the Odometry data and the IMU data, you need to read the appropriate topics. Try to find them on your own by typing:

Execute in WebShell #1

rostopic list
Have you found them? What type of message do they use? Are they the same as the ones listed here?

/sphero/imu/data3, type = sensor_msgs/Imu
/odom, type = nav_msgs/Odometry

Are they publishing? What's the data like?

Execute in WebShell #1

rostopic echo name_of_topic
Once you have the information, you are ready to create the python classes for each one.

Create two different python files in the src folder of the package that you just created, "my_sphero_topics."
These files have to have not only the class, but also a way of testing the class objects.

Remember that you need to move the Sphero to see the changes in both the /odom and the /sphero/imu/data3 topics. Use the previously created program to move it.

Here is an example of how this could be done (DO NOT watch unless you are stuck):

#! /usr/bin/env python
​
import rospy
from nav_msgs.msg import Odometry
​
class OdomTopicReader(object):
    def __init__(self, topic_name = '/odom'):
        self._topic_name = topic_name
        self._sub = rospy.Subscriber(self._topic_name, Odometry, self.topic_callback)
        self._odomdata = Odometry()
    
    def topic_callback(self, msg):
        self._odomdata = msg
        rospy.logdebug(self._odomdata)
    
    def get_odomdata(self):
        """
        Returns the newest odom data
​
        std_msgs/Header header                                                                                                                 
          uint32 seq                                                                                                                           
          time stamp                                                                                                                           
          string frame_id                                                                                                                      
        string child_frame_id                                                                                                                  
        geometry_msgs/PoseWithCovariance pose                                                                                                  
          geometry_msgs/Pose pose                                                                                                              
            geometry_msgs/Point position                                                                                                       
              float64 x                                                                                                                        
              float64 y                                                                                                                        
              float64 z                                                                                                                        
            geometry_msgs/Quaternion orientation                                                                                               
              float64 x                                                                                                                        
              float64 y                                                                                                                        
              float64 z                                                                                                                        
              float64 w                                                                                                                        
          float64[36] covariance                                                                                                               
        geometry_msgs/TwistWithCovariance twist                                                                                                
          geometry_msgs/Twist twist                                                                                                            
            geometry_msgs/Vector3 linear                                                                                                       
              float64 x                                                                                                                        
              float64 y                                                                                                                        
              float64 z                                                                                                                        
            geometry_msgs/Vector3 angular                                                                                                      
              float64 x                                                                                                                        
              float64 y                                                                                                                        
              float64 z                                                                                                                        
          float64[36] covariance                                                                                                               
        
        """
        return self._odomdata
    
if __name__ == "__main__":
    rospy.init_node('odom_topic_subscriber', log_level=rospy.INFO)
    odom_reader_object = OdomTopicReader()
    rospy.loginfo(odom_reader_object.get_odomdata())
    rate = rospy.Rate(0.5)
    
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        global ctrl_c
        print "shutdown time!"
        ctrl_c = True
​
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        data = odom_reader_object.get_odomdata()
        rospy.loginfo(data)
        rate.sleep()
The /sphero/imu/data3 topic will be done in exactly the same way. So try it yourself!

Step 2: Use Topics through Services
Now you need to take it a step further. Instead of having a topic subscriber/publisher on its own, you need to create a service that reads from topics.
You have to do the following:

Create a service that, when called, tells you if the robot has crashed or not, through the imu data. It also has to return some extra information, such as in what direction to move now that it has crashed.

We divided this into 3 tasks:

Determine the data
Modify the subscriber
Create the Server and Client
1. Determine data

Determine what data input you need ( request )
Determine what data you want the service to return ( response )
Then, you have to search for a service message that is already done in the system. You can find them in the std_srvs or rospy_tutorials package. You can also find other service messages created in non-standard packages. Bear in mind that using packages that might not be installed in ROS systems, or third party packages, is not recommened at this point. In this case, it is better to just generate your own service messages and use the std_msgs message types.
It is always preferable to use messages that are already done, just because it's faster and you don't have to deal with compilation.

In this case, you need a service message with the following structure (DO NOT watch unless you are stuck):

# request, Empty because no data is needed
---
#response
bool movement_successfull
string extra_data
In this case, you have a service message that has this exact structure.
It's in the std_srvs package and is called Trigger.srv. This is no coincidence. This service structure is very useful because normally you would ask a service to give you data, without providing any input.

So you just need to create a service that uses this Trigger.srv, that reads from the imu topic and tells you if you have crashed or not. It will also tell you based in the imu crash data, in which direction to move to now.


2. Modify the /sphero/imu/data3 topic susbcriber

Now you have to modify the /sphero/imu/data3 topic susbcriber for it to be able to tell you from which direction the crash occurred. Here is how it could be done (DO NOT watch unless you are stuck):

When you need to use an object from another python file, it has to be in the same package. Using python modules from other packages is not as easy as it may seem.

#! /usr/bin/env python
​
import rospy
from sensor_msgs.msg import Imu
​
​
class ImuTopicReader(object):
    def __init__(self, topic_name = '/sphero/imu/data3'):
        self._topic_name = topic_name
        self._sub = rospy.Subscriber(self._topic_name, Imu, self.topic_callback)
        self._imudata = Imu()
        self._threshhold = 7.00
    def topic_callback(self, msg):
        self._imudata = msg
        rospy.logdebug(self._imudata)
    
    def get_imudata(self):
        """
        Returns the newest imu data
​
        std_msgs/Header header                                                                                                          
          uint32 seq                                                                                                                    
          time stamp                                                                                                                    
          string frame_id                                                                                                               
        geometry_msgs/Quaternion orientation                                                                                            
          float64 x                                                                                                                     
          float64 y                                                                                                                     
          float64 z                                                                                                                     
          float64 w                                                                                                                     
        float64[9] orientation_covariance                                                                                               
        geometry_msgs/Vector3 angular_velocity                                                                                          
          float64 x                                                                                                                     
          float64 y                                                                                                                     
          float64 z                                                                                                                     
        float64[9] angular_velocity_covariance                                                                                          
        geometry_msgs/Vector3 linear_acceleration                                                                                       
          float64 x                                                                                                                     
          float64 y                                                                                                                     
          float64 z                                                                                                                     
        float64[9] linear_acceleration_covariance                                                                                                              
        
        """
        return self._imudata
    
    def four_sector_detection(self):
        """
        Detects in which four directions there is an obstacle that made the robot crash
        Based on the imu data
        Axis:
         ^y
         |
        zO-->x
        
        """
        x_accel = self._imudata.linear_acceleration.x
        y_accel = self._imudata.linear_acceleration.y
        z_accel = self._imudata.linear_acceleration.z
        
        
        axis_list = [x_accel, y_accel, z_accel]
        
        max_axis_index = axis_list.index(max(axis_list))
        positive = axis_list[max_axis_index] >= 0
        significative_value = abs(axis_list[max_axis_index]) > self._threshhold
        
        
        if significative_value:
            if max_axis_index == 0:
                # Winner is in the x axis, therefore its a side crash left/right
                rospy.logwarn("[X="+str(x_accel))
                rospy.loginfo("Y="+str(y_accel)+", Z="+str(z_accel)+"]")
                if positive:
                    message = "right"
                else:
                    message = "left"
            
            elif max_axis_index == 1:
                # Winner is the Y axis, therefore its a forn/back crash
                rospy.logwarn("[Y="+str(y_accel))
                rospy.loginfo("X="+str(x_accel)+", Z="+str(z_accel)+"]")
                if positive:
                    message = "front"
                else:
                    message = "back"
            elif max_axis_index == 2:
                # Z Axixs is the winner, therefore its a crash that made it jump
                rospy.logwarn("[Z="+str(z_accel))
                rospy.loginfo("X="+str(x_accel)+", Y="+str(y_accel)+"]")
                
                if positive:
                    message = "up"
                else:
                    message = "down"
            else:
                message = "unknown_direction"
        else:
            rospy.loginfo("X="+str(x_accel)+"Y="+str(y_accel)+", Z="+str(z_accel)+"]")
            message = "nothing"
        
        return self.convert_to_dict(message)
        
    def convert_to_dict(self, message):
        """
        Converts the fiven message to a dictionary telling in which direction there is a detection
        """
        detect_dict = {}
        # We consider that when there is a big Z axis component there has been a very big front crash
        detection_dict = {"front":(message=="front" or message=="up" or message=="down"),
                          "left":message=="left",
                          "right":message=="right",
                          "back":message=="back"}
        return detection_dict
        
    
if __name__ == "__main__":
    rospy.init_node('imu_topic_subscriber', log_level=rospy.INFO)
    imu_reader_object = ImuTopicReader()
    rospy.loginfo(imu_reader_object.get_imudata())
    rate = rospy.Rate(0.5)
    
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        global ctrl_c
        print "shutdown time!"
        ctrl_c = True
​
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        data = imu_reader_object.get_imudata()
        rospy.loginfo(data)
        rate.sleep()
A way of evaluating the threshold that you consider relevant crash, or how the axis may be placed is by executing the code, and then controlling the Sphero with the:

roslaunch sphero_gazebo keyboard_teleop.launch
3. Create the Server and client that tells you if there was a crash and what direction to move

Why do you need to create a client,too? Well, this is not needed for your core program to run, but its highly recommended because it allows you to test the server. So one way that it could be done is the following. Bear in mind that there is not only one way of doing things, and this is just a basic example (DO NOT watch unless you are stuck):

The Server:

#! /usr/bin/env python
​
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from imu_topic_susbcriber import ImuTopicReader
import time
​
​
class CrashDirectionService(object):
    def __init__(self, srv_name='/crash_direction_service'):
        self._srv_name = srv_name
        self._imu_reader_object = ImuTopicReader()
        self.detection_dict = {"front":False, "left":False, "right":False, "back":False}
        self._my_service = rospy.Service(self._srv_name, Trigger , self.srv_callback)
​
    def srv_callback(self, request):
        self.detection_dict = self._imu_reader_object.four_sector_detection()
        
        message = self.direction_to_move()
        
        rospy.logdebug("[LEFT="+str(self.detection_dict["left"])+", FRONT="+str(self.detection_dict["front"])+", RIGHT="+str(self.detection_dict["right"])+"]"+", BACK="+str(self.detection_dict["back"])+"]")
        rospy.logdebug("DIRECTION ==>"+message)
        
        response = TriggerResponse()
        """
        ---                                                                                                 
        bool success   # indicate if crashed                                       
        string message # Direction
        """
        response.success = self.has_crashed()
        response.message = message
        
        return response
​
    
    def has_crashed(self):
        for key, value in self.detection_dict.iteritems():
            if value:
                return True
        
        return False
    
    def direction_to_move(self):
​
        if not self.detection_dict["front"]:
            message = "forwards"
        
        else:
            if not self.detection_dict["back"]:
                    message = "backwards"
            else:
                if not self.detection_dict["left"]:
                    message = "left"
                else:
                    if not self.detection_dict["right"]:
                        message = "right"
                    else:
                        message = "un_stuck"
​
        
        return message
​
if __name__ == "__main__":
    rospy.init_node('crash_direction_service_server', log_level=rospy.INFO) 
    dir_serv_object = CrashDirectionService()
    rospy.spin() # mantain the service open.
The Client:

#! /usr/bin/env python
​
import rospy
from std_srvs.srv import Trigger, TriggerRequest
import sys 
​
rospy.init_node('crash_direction_service_client') # initialise a ROS node with the name service_client
service_name = "/crash_direction_service"
rospy.wait_for_service(service_name) # wait for the service client /gazebo/delete_model to be running
direction_service = rospy.ServiceProxy(service_name, Trigger) # create the connection to the service
request_object = TriggerRequest()
​
rate = rospy.Rate(5)
​
ctrl_c = False
def shutdownhook():
    # works better than the rospy.is_shut_down()
    global ctrl_c
    print "shutdown time!"
    ctrl_c = True
​
rospy.on_shutdown(shutdownhook)
​
while not ctrl_c:
    result = direction_service(request_object) # send through the connection the request
    """
    ---                             
    bool success   # indicate succes
    string message # informational, 
    """
    if result.success:
        rospy.logwarn("Success =="+str(result.success)) # print the result given by the service called
        rospy.logwarn("Direction To Go=="+str(result.message)) # print the result given by the service called
    else:
        rospy.loginfo("Success =="+str(result.success)) # print the result given by the service called
        rospy.loginfo("Direction To Go=="+str(result.message)) # print the result given by the service called
    rate.sleep()
Step 3: Use Topics through Actions
Now you need to create an action that, when called, starts to save odometry data and checks if the robot has exited the maze.
To accomplish that, you have to measure the distance from the starting point to the current position. If the distance is bigger that the maze distance, you are out. A more elaborate one would be to consider the vector, and therefore know if you exited correctly or you just jumped over a wall.

The Action should also stop if a certain amount of time has passed without the robot exiting the maze. The task is then:

Create an action server that finishes when it has detected that the robot has exited the maze, or has been working for a certain period of time. Use only the /odom topic subscriber.

We divided it into 3 subtasks:

Define the action message
Create the action server, action client, and algorithm to exit the maze
Test it
1. The first thing you have to think about is what kind of message you need for this action to work as intended.

You need to call this action, without any input.

It doesn't need to give feedback because the only thing that matters is that it returns the needed data to evaluate the distance. It needs to return the data used to calculate the distance, for post-completion calculations.

So the action message should look something like this (DO NOT watch this unless you are stuck):

#goal, empty                
---                             
#result, Odometry array             
nav_msgs/Odometry[] result_odom_array                
---                             
#feedback, empty
This message is, as you can see, customized. Therefore, you will need to compile the package.
The steps to do this are as follows:

Step 1: Create a new package called my_sphero_actions to store all of the action servers and the message.
Step 2: Create an action directory, and within, an action message called record_odom.action.
Step 3: Make all of the needed changes to the package.xml and CMakeLists.txt files in order to correctly compile the action message. These are the two files as they should be, if the only external dependency of your my_sphero_actions package is:

CMakeLists.txt

cmake_minimum_required(VERSION 2.8.3)
project(my_sphero_actions)
​
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
## Here go the packages needed to COMPILE the messages of topic, services and actions.
## in package.xml you have to state them as build
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  nav_msgs
)
​
## Generate actions in the 'action' folder
add_action_files(
   FILES
   record_odom.action
)
​
## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs
   nav_msgs
 )
​
###################################
## catkin specific configuration ##
###################################
## Declare things to be passed to dependent projects
## State here all the packages that will be needed by someone that executes something from your package
## All the packages stated here must be in the package.xml as exec_depend
catkin_package(
  CATKIN_DEPENDS rospy nav_msgs
)
​
###########
## Build ##
###########
​
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
​
​
package.xml

<?xml version="1.0"?>
<package format="2">
  <name>my_sphero_actions</name>
  <version>0.0.0</version>
  <description>The my_sphero_actions package</description>
​
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO</license>
​
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>actionlib_msgs</build_depend>
  <build_depend>nav_msgs</build_depend>
  
  <build_export_depend>rospy</build_export_depend>
  <exec_depend>rospy</exec_depend>
  <build_export_depend>nav_msgs</build_export_depend>
  <exec_depend>nav_msgs</exec_depend>
​
  <export>
  </export>
</package>
Once you think you have it, execute the following commands in the WebShell:

Execute in WebShell #1

roscd;cd ..
catkin_make
source devel/setup.bash
rosmsg list | grep record_odom
Output in WebShell #1

my_sphero_actions/record_odomAction
my_sphero_actions/record_odomActionFeedback
my_sphero_actions/record_odomActionGoal
my_sphero_actions/record_odomActionResult
my_sphero_actions/record_odomFeedback
my_sphero_actions/record_odomGoal
my_sphero_actions/record_odomResult
rosmsg list | grep record_odom: This command is listing all of the rosmsgs defined in the system and in your devel folder, and only filtering with grep command the ones with the name record_odom. All of the messages compiled from your packages are stored in the devel folder.
This is one of the best ways to know that your action message has been correctly compiled and is accesible for all of your ROS system.

2. Create the action server, the client, and the program to compute out of the maze.

This action server has to start recording the /odom topic and stop when a certain time period has passed or the distance moved reaches a certain value.

When you need to use an object from another python file, it has to be in the same package. Using python modules from other packages is not as easy as it may seem.

Move your /odom topic susbcriber to your my_sphero_actions package. This way, your server can use it easily.
Now create the action server and action client. It is the same here as it is in services: you don't need the client, but it's very useful to test the server and it also gives you a template for how to use it later in the core program.

Note: It may happen that when you test it, you get the following error:

ImportError: No module named my_sphero_actions.msg





This error is quite common when you generate your own messages. It doesn't find the my_sphero_actions.msg. But you have compiled it and doing the rosmsg list returns the correct output. Then, why? Because in order to allow your program to find the messages, you have to compile them and execute the source devel/setup.bash. This script sets not only the ROS environment, but also other systems related to message generation.
So, in order to always make your messages work, do the following:

Execute in WebShell #1

catkin_make
source devel/setup.bash
Now you are ready to work on your action server. This is an example of how it could be done (DO NOT watch unless you are stuck):

#! /usr/bin/env python
​
import rospy
import actionlib
from my_sphero_actions.msg import record_odomFeedback, record_odomResult, record_odomAction
from nav_msgs.msg import Odometry
from odom_topic_subscriber import OdomTopicReader
from odometry_analysis import check_if_out_maze
​
​
class RecordOdomClass(object):
    
    def __init__(self, goal_distance):
        """
        It starts an action Server. To test it was created correctly, just rostopic the list and search for /rec_odom_as/...
        When launching, bear in mind that you should have:
        $catkin_make
        $source devel/setup.bash
        """
        # creates the action server
        self._as = actionlib.SimpleActionServer("/rec_odom_as", record_odomAction, self.goal_callback, False)
        self._as.start()
        
        # Create an object that reads from the topic Odom
        self._odom_reader_object = OdomTopicReader()
        
        # create messages that are used to publish result
        self._result   = record_odomResult()
        
        self._seconds_recording = 120
        self._goal_distance = goal_distance
    
    def goal_callback(self, goal):
    
        success = True
        rate = rospy.Rate(1)
        
        for i in range(self._seconds_recording):
            rospy.loginfo("Recording Odom index="+str(i))
            # check that the preempt (cancelation) has not been requested by the action client
            if self._as.is_preempt_requested():
                rospy.logdebug('The goal has been cancelled/preempted')
                # the following line sets the client in a preempted state (goal cancelled)
                self._as.set_preempted()
                success = False
                # we end the action loop
                break
            
            else:# builds the next feedback msg to be sent
                if not self.reached_distance_goal():
                    rospy.logdebug('Reading Odometry...')
                    self._result.result_odom_array.append(self._odom_reader_object.get_odomdata())
                else:
                    rospy.logwarn('Reached distance Goal')
                    # we end the action loop
                    break
            rate.sleep()
        
        # at this point, either the goal has been achieved (success==true)
        # or the client preempted the goal (success==false)
        # If successful, then we publish the final result
        # If not successful, we do not publish anything in the result
        if success:
            self._as.set_succeeded(self._result)
            # Clean the Result Variable
        
        self.clean_variables()
    
    def clean_variables(self):
        """
        Cleans variables for the next call
        """
        self._result   = record_odomResult()
    
    def reached_distance_goal(self):
        """
        Returns True if the distance moved from the first instance of recording until now has reached the self._goal_distance
        """
        return check_if_out_maze(self._goal_distance, self._result.result_odom_array)
    
    
      
if __name__ == '__main__':
  rospy.init_node('record_odom_action_server_node')
  RecordOdomClass(goal_distance=2.0)
  rospy.spin()
Here is an example of how the odometry could be processed to know if the Sphero has exited the maze:

#! /usr/bin/env python
​
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
import math 
​
​
class OdometryAnalysis(object):
    def __init__(self):
        pass
    
    def get_distance_moved(self, odmetry_data_list):
        
        distance = None
        
        if len(odmetry_data_list) >= 2 :
            start_odom = odmetry_data_list[0]
            end_odom = odmetry_data_list[len(odmetry_data_list)-1]
            
            start_position = start_odom.pose.pose.position
            end_position = end_odom.pose.pose.position
            
            rospy.loginfo("start_position ==>"+str(start_position))
            rospy.loginfo("end_position ==>"+str(end_position))
            
            
            distance_vector = self.create_vector(start_position, end_position)
            rospy.loginfo("Distance Vector ==>"+str(distance_vector))
            
            distance = self.calculate_legth_vector(distance_vector)
            rospy.loginfo("Distance ==>"+str(distance))
        
        else:
            rospy.logerr("Odom array doesnt have the minimum number of elements = "+str(len(odmetry_data_list)))
        
        return distance
        
    def create_vector(self, p1, p2):
        
        distance_vector = Vector3()
        distance_vector.x = p2.x - p1.x
        distance_vector.y = p2.y - p1.y
        distance_vector.z = p2.z - p1.z
        
        return distance_vector
    
    def calculate_legth_vector(self,vector):
        
        length = math.sqrt(math.pow(vector.x,2)+math.pow(vector.y,2)+math.pow(vector.z,2))
        return length
​
​
def check_if_out_maze(goal_distance, odom_result_array):
    odom_analysis_object = OdometryAnalysis()
    distance = odom_analysis_object.get_distance_moved(odom_result_array)
    rospy.loginfo("Distance Moved="+str(distance))
    # To exit we consider that each square in the floor is around 0.5m, there fore to exit correctly
    # distance has to be sqrt (6*5 + 5*4) = 7.8
    return distance > goal_distance
And here is the client example:

#! /usr/bin/env python
​
import rospy
import time
import actionlib
from my_sphero_actions.msg import record_odomGoal, record_odomFeedback, record_odomResult, record_odomAction
from nav_msgs.msg import Odometry
​
​
# definition of the feedback callback. This will be called when feedback
# is received from the action server
# it just prints a message indicating a new message has been received
def feedback_callback(feedback):
    rospy.loginfo("Rec Odom Feedback feedback ==>"+str(feedback))
​
​
def count_seconds(seconds):
    for i in range(seconds):
        rospy.loginfo("Seconds Passed =>"+str(i))
        time.sleep(1)
​
# initializes the action client node
rospy.init_node('record_odom_action_client_node')
​
# create the connection to the action server
client = actionlib.SimpleActionClient('/rec_odom_as', record_odomAction)
​
rate = rospy.Rate(1)
​
# waits until the action server is up and running
rospy.loginfo('Waiting for action Server')
client.wait_for_server()
rospy.loginfo('Action Server Found...')
​
# creates a goal to send to the action server
goal = record_odomGoal()
​
# sends the goal to the action server, specifying which feedback function
# to call when feedback received
client.send_goal(goal, feedback_cb=feedback_callback)
​
​
​
# simple_state will be 1 if active, and 2 when finished. It's a variable, better use a function like get_state.
#state = client.simple_state
# state_result will give the FINAL STATE. Will be 1 when Active, and 2 if NO ERROR, 3 If Any Warning, and 3 if ERROR
state_result = client.get_state()
​
"""
class SimpleGoalState:
    PENDING = 0
    ACTIVE = 1
    DONE = 2
    WARN = 3
    ERROR = 4
​
"""
​
rospy.loginfo("state_result: "+str(state_result))
​
while state_result < 2:
    rospy.loginfo("Waiting to finish: ")
    rate.sleep()
    state_result = client.get_state()
    rospy.loginfo("state_result: "+str(state_result))
    
​
state_result = client.get_state()
rospy.loginfo("[Result] State: "+str(state_result))
if state_result == 4:
    rospy.logerr("Something went wrong in the Server Side")
if state_result == 3:
    rospy.logwarn("There is a warning in the Server Side")
​
rospy.loginfo("[Result] State: "+str(client.get_result()))
3. Test it.

Launch it:

rosrun my_sphero_actions rec_odom_action_server.py
Check that it's working:

rosnode list | grep record_odom_action_server_node
rostopic list | grep rec_odom_as
Launch the client to test if it really works.

Leave it running until the time runs out. It should return to the client side all of the /odom recordings until then.
Force that the distance goal gets reached. Use the roslaunch sphero_gazebo keyboard_teleop.launch to move the robot. It should also return the /odom topics recorded until then.
Step 4: Create a Main Program to Manage Everything
So, you finally have all the tools needed to create a main program that does the following:

Calls a service that tells you if it has crashed and in what direction you should move.
Moves the sphero based on the service response.
Checks if it has exited the maze or the time given has run out. If so, the program ends.
We have divided this into 3 sub-steps:

1. Create a package called my_sphero_main.

This package has to contain the main program and the python files that it needs. You might need to copy some files from other packages.

2. Create a launch file that launches the Action Server, the Service Server, and the main program.

Try it first with the node tags, and if it works, then generate a launch for the action_server and the service_server and use the include tag. Here is an example of how it could be done (DO NOT watch unless you are stuck):

<launch>
​
  <node pkg ="my_sphero_actions"
        type="rec_odom_action_server.py"
        name="record_odom_action_server_node"
        output="screen">
  </node>
  
  <node pkg ="my_sphero_services"
        type="direction_service_server.py"
        name="crash_direction_service_server"
        output="screen">
  </node>
​
  <node pkg ="my_sphero_main"
        type="sphero_main.py"
        name="sphero_main_node"
        output="screen">
  </node>
​
​
</launch>
3. Create the main program.

Use all of the data and knowledge that you have extracted from the clients of the server and action to reuse as much code as possible. This is an example of how it could be done: (DO NOT watch unless you are stuck)

#! /usr/bin/env python
​
import rospy
import actionlib
from std_srvs.srv import Trigger, TriggerRequest
from exam_action_rec_odom.msg import record_odomGoal, record_odomFeedback, record_odomResult, record_odomAction
from cmd_vel_publisher import CmdVelPub
from odometry_analysis import OdometryAnalysis
from odometry_analysis import check_if_out_maze
​
class ControlSphero(object):
    def __init__(self, goal_distance):
        self._goal_distance = goal_distance
        self.init_direction_service_client()
        self.init_rec_odom_action_client()
        self.init_move_sphero_publisher()
        
    def init_direction_service_client(self, service_name = "/crash_direction_service"):
        rospy.loginfo('Waiting for Service Server')
        rospy.wait_for_service(service_name) # wait for the service client /gazebo/delete_model to be running
        rospy.loginfo('Service Server Found...')
        self._direction_service = rospy.ServiceProxy(service_name, Trigger) # create the connection to the service
        self._request_object = TriggerRequest()
        
    def make_direction_request(self):
        
        result = self._direction_service(self._request_object) # send the name of the object to be deleted by the service through the connection
        return result.message
    
    def init_rec_odom_action_client(self):
        self._rec_odom_action_client = actionlib.SimpleActionClient('/rec_odom_as', record_odomAction)
        # waits until the action server is up and running
        rospy.loginfo('Waiting for action Server')
        self._rec_odom_action_client.wait_for_server()
        rospy.loginfo('Action Server Found...')
        self._rec_odom_action_goal = record_odomGoal()
    
    def send_goal_to_rec_odom_action_server(self):
        self._rec_odom_action_client.send_goal(self._rec_odom_action_goal, feedback_cb=self.rec_odom_feedback_callback)
        
    def rec_odom_feedback_callback(self,feedback):
        rospy.loginfo("Rec Odom Feedback feedback ==>"+str(feedback))
    
    def rec_odom_finished(self):
        
        has_finished = ( self._rec_odom_action_client.get_state() >= 2 )
        
        return has_finished
    
    def get_result_rec_odom(self):
        return self._rec_odom_action_client.get_result()
        
    def init_move_sphero_publisher(self):
        self._cmdvelpub_object = CmdVelPub()
​
    def move_sphero(self, direction):
        self._cmdvelpub_object.move_robot(direction)
​
    def got_out_maze(self, odom_result_array):
        return check_if_out_maze(self._goal_distance, odom_result_array)
​
rospy.init_node("sphero_main_node", log_level=rospy.INFO)
controlsphero_object = ControlSphero(goal_distance=2.0)
rate = rospy.Rate(10)
​
controlsphero_object.send_goal_to_rec_odom_action_server()
​
while not controlsphero_object.rec_odom_finished():
    direction_to_go = controlsphero_object.make_direction_request()
    rospy.loginfo(direction_to_go)
    controlsphero_object.move_sphero(direction_to_go)
    rate.sleep()
​
​
odom_result = controlsphero_object.get_result_rec_odom()
odom_result_array = odom_result.result_odom_array
​
if controlsphero_object.got_out_maze(odom_result_array):
    rospy.loginfo("Out of Maze")
else:
    rospy.loginfo("In Maze")
​
rospy.loginfo("Sphero Maze test Finished")
Step 5: How to Use Python Modules from Different Packages
As you might have noticed, if you wanted to use something declared in a python module that was from another package, you had to copy it into your code. This is because in ROS, python module importing from other packages is not as easy as it might seem.

To learn how to do it, go through this example, divided in 4 sub-steps:

Create packages
Prepare common_pkg
Import in testing
Test everything
Let's say that you have a package named common_pkg and another package named testing.

1. Create those 2 packages.

Once done, edit both of them to be able to use a python class defined in common_pkg, in a python program located in the package testing.

2. Prepare common_pkg.

To prepare common_pkg so that anyone can use the python files in it, follow these steps:

Execute the following commands on WebShell #1:
Execute in WebShell #1

roscd; cd ..
cd src/common_pkg
mkdir src/common_dir
touch src/common_dir/_\_\_init\_\_.py
This will generate an extra directory named common_dir inside a special python file, which you are able to find through the python named init.py,.

Once done, you create your python file inside the common_dir folder.
Here is an example of what you could put in it:
**Python Program {common_things.py}: common_things.py**


#! /usr/bin/env python
​
import rospy
from geometry_msgs.msg import Twist
import time
​
def cool(name):
    print('Cool ' + name)
​
class CmdVelPub(object):
    def __init__(self):
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._twist_object = Twist()
        self.linearspeed = 0.2
        self.angularspeed = 0.5
        
    def move_robot(self, direction):
        if direction == "forwards":
            self._twist_object.linear.x = self.linearspeed
            self._twist_object.angular.z = 0.0
        elif direction == "right":
            self._twist_object.linear.x = 0.0
            self._twist_object.angular.z = self.angularspeed
        elif direction == "left":
            self._twist_object.linear.x = 0.0
            self._twist_object.angular.z = -self.angularspeed
        elif direction == "backwards":
            self._twist_object.linear.x = -self.linearspeed
            self._twist_object.angular.z = 0.0
        elif direction == "stop":
            self._twist_object.linear.x = 0.0
            self._twist_object.angular.z = 0.0
        else:
            pass
        
        self._cmd_vel_pub.publish(self._twist_object)
**END Python Program {common_things.py}: common_things.py**


Go to the root of the common_pkg
Execute in WebShell #1

roscd common_pkg
Create a file named setup.py, with the following content:
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
​
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
​
# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['common_dir'],
    package_dir={'': 'src'},
)
​
setup(**setup_args)
Now edit the CMakeLists.txt file, and uncomment the following line:
## Uncomment if the package has a setup.py
catkin_python_setup()
Compile and check that there is no error:
Execute in WebShell #1

roscd; cd ..
catkin_make
source devel/setup.bash
3. Import in testing.

Once you have set up the common_pkg, you just have to import the elements in the following way:

from common_dir.common_things import cool, CmdVelPub
Note that you have imported with the name of the directory inside the src directory of your package.
name of the package = common_pkg
name of the directory and where you import = common_dir
This is very important because it can lead to errors.

So here you have an example of the python file in the src directory of the testing package:

**Python Program {test_import.py}: test_import.py**


#! /usr/bin/env python
import rospy
import time
​
from common_dir.common_things import cool, CmdVelPub
​
​
if __name__ == '__main__':
    cool('TheConstruct')
    
    stop_time = 1
    move_time = 3
    
    rospy.init_node('test_import', log_level=rospy.INFO)
    
    move_object = CmdVelPub()
    rospy.loginfo("Starting...")
    move_object.move_robot(direction="stop")
    time.sleep(stop_time)
    
    rospy.loginfo("Forwards...")
    move_object.move_robot(direction="forwards")
    time.sleep(move_time)
    
    rospy.loginfo("Stopping...")
    move_object.move_robot(direction="stop")
    time.sleep(stop_time)
    
    rospy.loginfo("Forwards...")
    move_object.move_robot(direction="backwards")
    time.sleep(move_time)
    
    rospy.loginfo("Stopping...")
    move_object.move_robot(direction="stop")
    
**END Python Program {test_import.py}: test_import.py**


4. Test everything.

Now execute the test_import.py file:

Execute in WebShell #1



rosrun testing test_import.py
You should see the robot moving.

Conclusion
Now try to optimise your system. Play with the rates and the detection strategy. Play with the movement and the "AI" used to decide what to do.
You especially need to know this project (except STEP 5) by heart because the exam will be very similar.

