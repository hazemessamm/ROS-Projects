
ROS IN 5 DAYS
Course Project




ROSject Link: http://bit.ly/2LQpSzi

Package Name: turtlebot_project

Launch File: main.launch

Estimated time to completion: 10 hours

What you will learn with this unit?

Practice everything you learn through the course
Put together everything you learn into a big project
Create a main program
Win the TurtleBot Race!
In this project, you will have to make a Turtlebot 2 Robot move along a maze faster than the other students. The fastest one will win a prize.

For this goal, you will have to apply all of the things that you are learning along the course. It's really important that you complete it because all of the structures that you create for this project will be asked about in our Official Exam.

Get the robot out of the maze as fast as possible and you will get the prize. The ideal would be that Turtlebot goes out cleanly, but it may happen that you collide with the maze. You can use the collision detection to get data and help you in your strategy to get out.

Basically, in this project you will have to:

Apply all of the theory given in the course
Decide on a strategy to solve the problem
Implement this strategy in the simulation environment
Make as many tests as required in the simulation environment until it works
To achieve success in this project, we provide 5 steps below that you should follow with clear instructions and even solutions.

Also, remember to:

Create your packages and code in the simulation environment, as you have been doing throughout the course.
Use the consoles to gather information about the status of the simulated robot.
Use the IDE to create your programs and execute them through the consoles, observing the results on the simulation screen. You can use other consoles to watch calls to topics, services, or action servers.
Everything that you create in this unit will be automatically saved in your space. You can come back to this unit at any time and continue with your work from the point that you left it.
Every time you need to reset the position of the robot just press the restart button in the simulation window.
Use the debugging tools to try to find what is not working and why (for instance, the rviz tool is very useful for this purpose).
One final note: Because the program that you create should work with a real robot, if needed, you can't move the Turtlebot in a closed loop. This is because, in reality, the circuit could be different, the robot could not be as responsive, there might be errors in the readings, and so on. So in this project, you should create a program that can cope with all of these uncertainties.

What does Turtlebot Provide to Program It?
So the main question is, what can you do with Turtlebot from a ROS programming point of view? Which sensors and actuators does Turtlebot provide that will allow you to do the maze test?

Good question! Turtlebot provides the following sensors and actuators:

Sensors
Laser sensor: Turtlebot has a 2D Laser which provides information about the environment you are in. The value of the sensor is provided through the topic /kobuki/laser/scan.
Odometry: The odometry of the robot can be accessed through the /odom topic.
Actuators
Speed: You can send speed commands to move the robot through the /cmd_vel topic.
Now that you know the relevant topics of the robot, it is your job to figure out the types of messages and how to use them in order to make the robot do the strategy you want it to do.

Ideas to Start Working On
Here is a list of things that you can start from. You do not have to follow them. They are provided just in case you don't know how to start.

Start watching some of the messages that the sensor topics are publishing. Try to get an idea about the information they are providing. Move the robot in the simulation and see how those messages change their values. It is very important that you understand how changes in the robot produce changes in the topics.
Try to move the robot sending messages to the /cmd_vel (either through the console or through python programs).
Observe how the messages of the topics change when the robot moves around the environment, or when it crashes into a wall.
Is the odometry trustworthy? Can you move the robot the exact amount even when it collides with something?
Steps You Should Cover
These are the steps that you should follow throughout the duration of the project. These steps will ensure that you have practised and created all of the structures asked for in the final exam of this course. If you perform all of the steps mentioned here, you will find the exam feasible.

Step 1: Read and Write Topics (Dedicate 2 hours)
Step 2: Use topics through Services (Dedicate 3 hours)
Step 3: Use topics through Actions (Dedicate 4 hours)
Step 4: Create a main program to manage everything (Dedicate 1 hour)
EXTRA Step: How to use python modules from different packages (Not required, included here for information purposes only)
NOTE: The 5th Step may not be required, but we have found some students that organize their code in such a way that they need this step. We have provided it here just in case you need it, but you don't have to use it.

Step 1: Read and Write Topics
This step has 3 actions for you to do:

Create a package called my_turtlebot_topics that will contain all of the programs related to topics
Create a topic publisher that allows you to move the Turtlebot.
Create two topic subscribers that extract the data that you need from the Odometry and the Laser.
So, let's get started.

1. Create package my_turtlebot_topics, with rospy as dependency.

2. Create the Topic Publisher to move Turtlebot.

To move Turtlebot, you need to publish in the topic /cmd_vel.

It's important that you always encapsulate your topic subscribers and publishers inside classes. This will allow you to store values and manage the publishing callbacks easily.

First, you have to see if there is a topic like /cmd_vel running.

Note: It will not always be this simple. In real robots, you will need to access the code to see what is the name of the topic that moves the robot, or even use rostopic info /name_of_topic to know which one it could be.

Execute in WebShell #1

rostopic list
As you may see, there is a /cmd_vel.
You, then, have to extract the type of message that /cmd_vel uses.

Execute in WebShell #1

rostopic info /cmd_vel
Test that it works by publishing different values:

Execute in WebShell #1

rostopic pub /cmd_vel message_type_of_cmd_vel [TAB][TAB]
Once you have the information, you are ready to create the python class.
Create a python file in the src folder of the package you just created, "my_turtlebot_topics".
This file has to have not only the class, but also a way of testing that the class works.

3. Create Two Topic Subscribers that extract the data that you need from the Odometry and the Laser.

To get the Odometry data and the Laser data, you need to read the appropriate topics. Try finding them by yourself, first by typing:

Execute in WebShell #1

rostopic list
Have you found them? What type of message do they use? Are they the same as the ones listed here?
/kobuki/laser/scan, type = sensor_msgs/LaserScan
/odom, type = nav_msgs/Odometry

Are they publishing? What's the data like?

Execute in WebShell #1

rostopic echo name_of_topic
Once you have the information, you are ready to create the python classes for each one.

Create two different python files in the src folder of the package that you just created, "my_turtlebot_topics".
These files have to have not only the class, but also a way of testing the class objects.

Remember that you need to move the Turtlebot to see the changes in both the /odom and the /kobuki/laser/scan topics. Use the previously created program to move it.

Step 2: Use Topics through Services
Now you need to take it a step further. Instead of having a topic subscriber/publisher on its own, you need to create a service that reads from the topics.
You have to do the following:

Create a service that, when called, it tells you if the robot is about to hit an obstacle, through the laser data. It also has to return some extra information, such as the direction that the robot should move in next.

We divided this into 3 tasks:

Determine data
Modify the subscriber
Create the Server and Client
1. Determine data

Determine what data input you need ( request )
Determine what data you want the service to return ( response )
Then, you have to search for a previously done service message in the system. You can find them in the std_srvs or rospy_tutorials package. You can also find other service messages created in non-standard packages. Bear in mind that using packages that might not be installed in ROS systems or from third party packages is not recommened at this point. In this case, it's better to just generate your own service messages and use the std_msgs message types.
It's always preferable to use previously done messages, just because it's faster and you don't have to deal with compilation.

In this case, you need a service message of the following structure (DO NOT watch unless you are stuck):

# request, Empty because no data is needed
---
#response
bool movement_successfull
string extra_data
In this case, you have a service message that has this exact structure.
It's in the std_srvs package and is called Trigger.srv. This is no coincidence. This service structure is very useful because, normally, you ask a service to give you data without providing any input.

So you just need to create a service that uses this Trigger.srv that reads from the laser topic and tells you if you are about to crash or not. It will also tell you, based on the laser data, in which direction to move in now.


2. Modify the /kobuki/laser/scan topic subscriber

Now you have to modify the /kobuki/laser/scan topic susbcriber to be able to tell you in what direction it is going to be the crash.

When you need to use an object from another python file, it has to be in the same package. Using python modules from other packages is not as easy as it may seem.

A way of evaluating the threshold that you consider it may be a potential crash, is by executing the code, and then controlling the Turtlebot with the:

roslaunch turtlebot_teleop keyboard_teleop.launch

3. Create the Server and client that tell you if there is a potential crash, and in what direction to move

Why do you need to create a client, too? Well, this is not needed for your core program to run, but it's highly recommended because it allows you to test the server.

Step 3: Use Topics through Actions
Now you need to create an action that, when called, will start to save odometry data and check if the robot has exited the maze.
To accomplish that, you have to measure the distance from the starting point to the current position. If the distance is larger than the maze distance, you are out. A more elaborate one would be to consider the vector, and therefore, know if you exited correctly or you just jumped over a wall.

The Action should also stop in case a certain period of time has passed without the robot exiting the maze. The task is then:

Create an action server that finishes when it has detected that the robot has exited the maze, or has been working for a certain period of time. Use only the /odom topic subscriber.

We divided it into 3 subtasks:

Define the action message
Create the action server, action client, and algorithm to exit the maze
Test it
1. The first thing you have to think about is what kind of message you need for this action to work as intended.

You need to call this action, without any input.

It doesn't need to give feedback because the only thing that matters is that it returns the needed data to evaluate the distance. It needs to return the data used to calculate the distance for post-completion calculations.

So the action message should look something like this (DO NOT watch this unless you are stuck):

#goal, empty                
---                             
#result, Odometry array             
nav_msgs/Odometry[] result_odom_array                
---                             
#feedback, empty
This message is, as you can see, custom. Therefore, you will need to compile the package.
The steps to do this are as follows:

Step 1: Create a new package called my_turtlebot_actions to store all the action servers and the message.
Step 2: Create an action directory, and within, an action message called record_odom.action.
Step 3: Make all of the needed changes to the package.xml and CMakeLists.txt files, in order to correctly compile the action message. These are the two files as they should be, if the only external dependency of your my_turtlebot_actions package is:

CMakeLists.txt

cmake_minimum_required(VERSION 2.8.3)
project(my_turtlebot_actions)
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
  <name>my_turtlebot_actions</name>
  <version>0.0.0</version>
  <description>The my_turtlebot_actions package</description>
​
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO</license>
​
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>actionlib_msgs</build_depend>
  <build_depend>nav_msgs</build_depend>
  
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>nav_msgs</build_export_depend>
  
  <exec_depend>rospy</exec_depend>
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

my_turtlebot_actions/record_odomAction
my_turtlebot_actions/record_odomActionFeedback
my_turtlebot_actions/record_odomActionGoal
my_turtlebot_actions/record_odomActionResult
my_turtlebot_actions/record_odomFeedback
my_turtlebot_actions/record_odomGoal
my_turtlebot_actions/record_odomResult
rosmsg list | grep record_odom: This command lists all of the rosmsgs defined in the system and in your devel folder, and only filters with the grep command the ones with the name record_odom. All of the messages compiled from your packages are stored in the devel folder.
This is one of the best ways to know that your action message has been correctly compiled and is accessible for your entire ROS system.

2. Create the action server, the client, and the program to compute out of the maze.

This action server has to start recording the /odom topic and stop when a certain time period has passed, or the distance moved reaches a certain value.

When you need to use an object from another python file, it has to be in the same package. Using python modules from other packages is not as easy as it may seem.

Move your /odom topic subscriber to your my_turtlebot_actions package. This way, your server can use it easily.
Now create the action server and action client. It is the same here as in services: you don't need the client but it's very useful to test the server, and it also gives you a template for how to use it later in the core program.

Note: It may happen that, when you test it, you get the following error:

ImportError: No module named my_turtlebot_actions.msg





This error is quite common when you generate your own messages. It doesn't find the my_turtlebot_actions.msg. But you have compiled it and doing the rosmsg list returns the correct output. Then, why? Because in order to allow your program to find the messages, you have to compile them and execute the source devel/setup.bash. This script sets not only the ROS environment, but also other systems related to message generation.
So, in order to always make your messages work, do the following:

Execute in WebShell #1

catkin_make
source devel/setup.bash
Now you are ready to work on your action server.

3. Test it.

Launch it:

rosrun my_turtlebot_actions rec_odom_action_server.py

Check that it's working:

rosnode list | grep record_odom_action_server_node
rostopic list | grep rec_odom_as

Launch the client to test that it really works.

Leave it running until the time runs out. It should return in the client side all of the /odom recordings up until then.
Force the distance goal to be reached. Use the roslaunch turtlebot_teleop keyboard_teleop.launch. It should return the /odom topics recorded up until then as well.
Step 4: Create a Main Program to Manage Everything
So, finally, you have all of the tools needed to create a main program that does the following:

Calls a service that tells you if it is going to crash and in what direction you should move.
Moves the turtlebot based on the service response.
Checks if it has exited the maze or the time given has run out. If so, the program ends.
We have divided this into 3 sub-steps:

1. Create a package called my_turtlebot_main.

This package has to contain the main program and the python files that it needs. You might need to copy some files of other packages.

2. Create a launch file that launches the Action Server, the Service Server, and the main program.

Try it first with the node tags and, if it works, then generate launched for the action_server and the service_server and use the include tag. Here is an example of how it could be done (DO NOT watch unless you are stuck):

<launch>
​
  <node pkg ="my_turtlebot_actions"
        type="rec_odom_action_server.py"
        name="record_odom_action_server_node"
        output="screen">
  </node>
  
  <node pkg ="my_turtlebot_services"
        type="direction_service_server.py"
        name="crash_direction_service_server"
        output="screen">
  </node>
​
  <node pkg ="my_turtlebot_main"
        type="turtlebot_main.py"
        name="turtlebot_main_node"
        output="screen">
  </node>
​
​
</launch>
3. Create the main program.

Use all of the data and knowledge that you have extracted from the clients of the server and action, to reuse as much code as possible.

Step 5: How to Use Python Modules from Different Packages
As you might have noticed, if you wanted to use something declared in a python module that was from another package, you had to copy it into your code. This is because in ROS, python module importing from other packages is not as easy as it might seem.

To learn how to do it, you will go through this example, divided into 4 sub-steps:

Create the packages
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
touch src/common_dir/__init__.py
This will generate an extra directory named common_dir, and inside a special python file, to be able to find it through python, named init.py,.

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
Note that you import with the name of the directory inside the src directory of your package.
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

Now execute the test_import.py:

Execute in WebShell #1

rosrun testing <i>test_import.py
You should see the robot moving.

Conclusion
Now try to optimise your system. Play with the rates and the detection strategy. Play with the movement and the "AI" used to decide what to do.
You especially have to know this project (except STEP 5) by heart because the exam will be very similar.

Solutions

Please Try to do it by yourself unless you get stuck or need some inspiration. You will learn much more if you fight for each exercise.



Follow this link to open the solutions for the Turtlebot Project:Turtlebot Project Solutions

