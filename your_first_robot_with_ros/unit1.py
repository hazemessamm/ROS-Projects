


Building and Rosifying a Robot from Scratch
Unit 0: Introduction to the Course
SUMMARY

Estimated time to completion: 10 minutes

This unit is an introduction to the Building and Rosifying a Robot from Scratch Course. You'll have a quick preview of the contents you are going to cover during the course, and you will also view a practical demo.

END OF SUMMARY

What's this course about?
It's a fact that each year the number of people interested in learning robotics increases, and so does the need to make robotics more accessible to everybody. With this in mind, a good number of robots that are more affordable have appeared so that anybody can use them to learn and train their robotic skills.

One of these robots is the ROSbots, created by Jack Pien. ROSbots is a 2-wheeled robot, which can be easily purchased and assembled since it's based on affordable pieces, like a Raspberry Pi.

With this in mind, we decided to create a Course that will drive our ROS students through the complete process of building a physical robot from scratch, and addings ROS to it in order to be able to control it and create nice applications for it.

So basically, within this course you are going to go through the whole process of mounting the robot, creating a simulation for the robot so that you can test your ROS programs there, building all the controllers for the robot, and finally, getting it to autonomously navigate using ROS tools.

Do you want to have a taste?
With the proper introductions made, it is time to actually start. And... as we always do in the Robot Ignite Academy, let's start with practice! In the following example, you will be using a simulated version of the robot you are going to build from scratch in this course. So... let's go!

Demo 1.1


a) For this demo, we are going to be using a simple Lane Following algorithm that you will develop yourself in the Navigation chapters of this course.

Execute in WebShell #1

rosrun line_follower line_follow_test.py
You should now see the robot trying to follow the yellow lanes on the ground.



End of Demo 1.1

What will you learn with this course?
Basically, during this course, you will address the following topics:

Build a Basic 2-Wheeled Robot with a Camera Sensor and Raspberry Pi
How to Communicate with the Robot (via WiFi and Ethernet)
Build a Gazebo Simulation of the 2-Wheeled Robot
Control the 2-Wheeled Robot (Create the Drivers)
Navigate with the 2-Wheeled Robot (OpenCV)
Apply Deep Learning Trainings to our 2-Wheeled Robot (openai_ros package)
How will you learn all this?
You will learn through hands-on experience from day one! Besides building the robot from scratch, you will create a simulation of the robot, like the below one.

Simulated RIABot:



Real RIABot:



Minimum requirements for the course
In order to be able to fully understand the contents of this course, it is highly recommended that you have the following knowledge:

Basic ROS. You can get this knowledge by following the ROS in 5 Days course.
Basic Python.
Basic Unix shell knowledge.
Basic OpenAI knowledge. You can get this knowledge by following the OpenAI Gym for Robotics 101 course.
Some basic electronics knowledge.
Shopping List
As you already know, this Course will be based both on a physical and a simulated robot. So, in order to build the physical robot, you will need to purchase the following list of items.

**Product**	**Purchase Link**
Magician Chassis	https://www.amazon.com/Karlsson-Robotics-SX10825-Magician-Chassis/dp/B007R9U5CU/
Raspberry Pi 3 Model B	https://www.amazon.com/Raspberry-Pi-MS-004-00000024-Model-Board/dp/B01LPLPBS8
16GB Class 10 MicroSD Card	https://www.amazon.com/SanDisk-microSDHC-Standard-Packaging-SDSQUNC-032G-GN6MA/dp/B010Q57T02/
Generic Raspberry Pi Lipo Battery Power Pack Board	https://www.amazon.com/GAOPAN-Lithium-Battery-Expansion-Raspberry/dp/B07QWRN4V6/
Adafruit Powerboost 1000c Board	https://www.amazon.com/Adafruit-PowerBoost-1000-Charger-Rechargeable/dp/B01BMRBTH2/
Lipo Battery for the Powerboost	https://www.amazon.com/1800mAh-battery-Rechargeable-Lithium-Connector/dp/B07BTTKM8H/
Breadboard (1 piece)	https://www.amazon.com/Points-Solderless-Bread-Board-Breadboard/dp/B07JXYBT7N/
Wires	https://www.amazon.com/LANDZO-Multicolored-Breadboard-Raspberry-Arduino/dp/B01IB7UOFE/
H-Bridge L9110 DC/Stepper 2 Way Motor Driver Module	https://www.amazon.com/Super-H-Bridge-Driver-Module-Arduino/dp/B00NN6EB3U/
Arduino UNO Board	https://www.amazon.com/KAILEDI-Development-Microcontroller-ATmega328-ATMEGA16U2/dp/B07V9VGXFS/
Raspberry Pi Camera Module	https://www.amazon.com/Raspberry-Pi-Camera-Module-Megapixel/dp/B01ER2SKFS/
Micro USB Breakout Board	https://www.amazon.com/Adafruit-Micro-B-Breakout-Board-ADA1833/dp/B00KLDPZVU/


Please bear in mind that these purchase links are from Amazon Spain. If you are from any other part of the world, we recommend you to try to find these products from your local resellers. For instance, if you are from the States, it is very likely that you will find all these products in Amazon US (amazon.com).

Special Thanks
This course wouldn't have been possible without the amazing work done by Jack Pien, who created the magnificent ROSbots. For more info about him, please follow the links below:

ROSBots Official Page: https://www.rosbots.com/
ROSBots Github: https://github.com/rosbots
ROSBots Blog: https://medium.com/@rosbots
This course also wouldn't have been possible without the knowledge and work of the ROS Community, OSRF, and Gazebo Team.
