
Unit 1: URDF Intro


NOTE: Bear in mind that the full simulation (with the 3 robots on it) may take some time to load completely. Just be patient!

Here you have three simulated robots:

JIBO: This is a real robot created by Cynthia Breazeal and her Team. It's a social robot with 3 main movement axis. You are going to learn how to create its simulation in this course.

Mira: The physical version of this very cute and tiny robot was created by Alonso Martinez, a worker at Pixar. It's designed to demonstrate that with very simple elements, you can give life to a robot, as only Pixar people know how. The physical version can play Peekaboo with humans by using facial recognition through its single camera. It has a very ingenious roll, pitch, and yaw movement system. You will know how to create this simulation when you finish this course and do facial recognition with it, if you so desire.

Gurdy: Its physical version was also created by Alonso Martinez. In this case, it's a movement study robot based on the bouncing ball animation exercise. This simulation has been changed from the original, giving more freedom to its three legs. You will learn how to create it and move it around, and also use its IMU.

To make the three robots in the simulation move, just execute the following command in the WebShell:

Execute in WebShell #1

roslaunch urdf_demo_pkg start_robot_party.launch 
You should see something similar to this:



Want to see the Jibo or Mira's camera? Just type the following in another WebShell and then select the camera topic in the graphical interface. Then, select "/jibo/jibo/camera1/image_raw" or "/mira/mira/camera1/image_raw."

Execute in WebShell #2

rosrun rqt_image_view rqt_image_view
And click on the Graphical Interface icon to be able to visualize it:



Bear in mind that it can take up to 20 seconds to establish a video flow, so just be patient once the first image appears. Afterwards, switching to other cameras will be much faster.



So, what's in this course for you?

In this course, you will learn how to go from a physical robot, or even a robot drawing, to a full-fledged simulation with physics, actuators, and sensors.
For this, you will learn how to create a file that defines your robot in the Gazebo-ROS ecosystem. In this file, you will define weights, inertias, joints, links, sensor plugins, and all that is needed to simulate a robot.

These files are called URDF files. They are based on XML language, so it's simple to grasp.

Hit that NEXT button to start right now or keep on reading for more details.
Why do I need to simulate robots?
There are many reasons why someone would need to simulate a robot:

You need some way to describe your physical robot in ROS: The URDF models are not only needed for simulation, they are also used for having a virtual description of the real robot that is publishing data in ROS. Through this representation, ROS programs like RVIZ can represent the robot based on the real joint values, for example.
It's impossible to work with the real robot:
There are loads of robots that aren't available to the general public, like ASIMO, VALKIRIA, ROBONAUT, and ATLAS, to name a few. They are very well designed robots, but very few people can use them; and if they can, they are subjected to very strict conditions. So, why don't you create your own virtual version of it?
I want to create a physical robot that doesn't exist yet:
Building a robot is expensive, so you might as well be sure that there are no big design flaws.
By creating it first in a simulation, you will be able to iterate much faster in the design and detect any errors before you even start the 3D printing.
You will also be able to do any test and make any change in the design without any real cost. For example, what would happen if you were to add a leg, or two cameras, or use a more powerful servers, or make it heavier...The list is endless.
It's too risky to do my preliminary tests on the physical robot:
You do have the robot, but you may want to test something that's too dangerous for the integrity of the robot.
Maybe you want to try an algorithm for HumanRobotInteraction for the first time, and you don't want anyone to get hurt in the tests.
How will you learn about URDF files
As with all of the courses in RobotIgnite, this course will be based on practice from day one.
Minimal theory and loads of practice.
You will have to create three different robots, with different dynamics, design, controls, and sensors.
Through these exercises, you will learn:

How to import your 3D models so that they are in the correct format: Some tips and tricks to make that transition easier.
Creating a Visual Model of the robot in URDF: No physics and no sensors. Just the links, joints, and 3D models.
Adding it to the Gazebo simulator: How to add all the physics so that your URDF can be simulated.
Adding Sensors: How to add some basic sensors like cameras and imus.
How to go from a monolithic URDF file to the XACROS file system. This will make URDFs much simpler and more flexible towards changes.
Requirements
It's essential that before starting this course, you know the following:

ROS Basics: You need to know all the basics of ROS to be able to follow this course. If you don't, please do our ROS Basics in Five Days course.
Basic understanding of how XML files work.
Basic knowledge of Python.
Some basic mechanical knowledge of what Torques are and how Forces work, although you will be taught what you need to know to create your robot.
Some knowledge on how PIDs work, although you will be taught what you need to know to create your robot.
Go to the next unit if you want to know how to create a simulation of any robot you want
Special thanks
We would like to give our biggest thanks to:

Alonso Martinez for his amazing robots. We decided to create a simulated version of them in this course because they are so simple and yet so rich in their abilities and expressions that we couldn't resist. They are a perfect example of the simplicity in design that makes the teaching of URDF structure so much simpler. We loved them from the first time we set eyes on them. You can see a full video where Alonso Martinez shows them on the great Tested YouTube channel. Highly advisable if you want to feel inspired to create your own robots.
Also, a big shout out to michaelrod77 for his fantastic Jibo model that was used for this simulation.
Thanks also to JiboTeam for creating this amazing robot that we are using as a model to teach you URDF creation.They have created an SDK to program the physical robot, so don't hesitate to give it a try if you are into SocialRobotics.
This course wouldn't have been possible without the knowledge and work of the ROS Community, OSRF, and Gazebo Team
