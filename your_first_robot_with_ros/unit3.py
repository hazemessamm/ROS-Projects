Loading [MathJax]/jax/output/HTML-CSS/fonts/STIX-Web/Main/Italic/Main.js
Building and Rosifying a Robot from Scratch


Unit 2: Creating a simulation of the robot
SUMMARY

Estimated time to completion: 2 hours

In this unit, you are going to see how to create a simulation of your RIAbot robot, so you can test everything you want in the simulated environment, without worrying about damaging anything on the real robot.

END OF SUMMARY

So, in the previous unit, you were building your robot from the hardware side. This means, assembling all the different pieces and connecting all the electronic parts. At this point, then, you should already have your robot built and ready to be used.

Of course, you could already connect to your robot, and test your code there to see how it performs. But we don't recommend doing this. Why? Well, there could be many reasons, but the most important one, probably, is that your robot could get damaged or, even worse, it could damage someone. Of course, in the case of our robot, it's unlikely that something like this could happen since it is quite a simple (and small) robot.

Anyway, we think that the best practice when working with robots is to always work first in a simulated environment. This will allow you to do all the tests you want, without worrying about the physical robot. And afterwards, when you are confident enough that your code will work correctly, you can test it on the real robot.

There are already many robot simulations created by companies or by the community. You can check some of them out in our ROSDS (ROS Development Studio) . For some cases, though, you might need to build the simulation for your robot yourself, such as in this case. So, in this unit, we are going to build, step by step, a simulation of our robot.

Creating the basic URDF files
In order to have a Gazebo simulation where we can test our ROSbots robot, the first thing we'll need to do is create the URDF file for the robot. As you may already know, URDF files are the files that describe a robot. If you are interested in this topic, you can take a deeper look into it with the Robot Creation with URDF Course at our online Academy.

Let's follow the next exercise in order to start creating the URDF files for our robot.

**Exercise 2.1**


a) Create a new package named rosbots_description.

Execute in WebShell #1

roscd; cd ..; cd src;
catkin_create_pkg rosbots_description rospy
b) Inside the package, create a new folder named urdf. And inside this folder, create a new file named rosbots.xacro

Execute in WebShell #1

roscd rosbots_description
mkdir urdf; cd urdf;
touch rosbots.xacro
chmod +x rosbots.xacro
Next, you'll place the following code into the rosbots.xacro file.

**rosbots.xacro**

<?xml version="1.0" encoding="utf-8"?>
<robot name="rosbots" xmlns:xacro="http://www.ros.org/wiki/xacro">
​
    <link name="base_footprint"/>
​
    <joint name="base_joint" type="fixed">
      <origin xyz="0 0 0.05" rpy="0 0 0" />
      <parent link="base_footprint"/>
      <child link="base_link" />
    </joint>
    
    <link name="base_link">
      <visual>
        <geometry>
          <mesh filename="package://rosbots_description/meshes/base.dae"/>
        </geometry>
        <origin xyz="-0.52 -0.4 0.43" rpy="0 0 0"/>
      </visual>
      <collision>
        <geometry>
          <mesh filename="package://rosbots_description/meshes/base.dae"/>
        </geometry>
        <origin xyz="-0.52 -0.4 0.43" rpy="0 0 0"/>
      </collision>
      <inertial>
        <origin xyz="0.0 0 0."/>
        <mass value="0.5"/> 
        <inertia ixx="0.01" ixy="0.0" ixz="0.0"
                 iyy="0.01" iyz="0.0" 
                 izz="0.03" />
      </inertial>
    </link>
    
    <joint name="second_level_joint" type="fixed">
      <origin xyz="0 0 0.68" rpy="0 0 0" />
      <parent link="base_link"/>
      <child link="base_second_link" />
    </joint>
    <link name="base_second_link">
      <visual>
        <geometry>
          <mesh filename="package://rosbots_description/meshes/base.dae"/>
        </geometry>
        <origin xyz="-0.52 -0.4 0.0" rpy="0 0 0"/>
      </visual>
      <collision>
        <geometry>
          <mesh filename="package://rosbots_description/meshes/base.dae"/>
        </geometry>
        <origin xyz="-0.52 -0.4 0.0" rpy="0 0 0"/>
      </collision>
      <!--inertial>
        <origin xyz="0.01 0 0.7"/>
        <mass value="1.0"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0"
                 iyy="0.01" iyz="0.0" 
                 izz="0.03" />
      </inertial-->
    </link>
​
    <joint name="mcu_joint" type="fixed">
      <origin xyz="0.02 0.12 0.73" rpy="0 0 0" />
      <parent link="base_link"/>
      <child link="mcu_link" />
    </joint>
    <link name="mcu_link">
      <visual>
        <geometry>
          <mesh filename="package://rosbots_description/meshes/rasp.dae" scale="0.14 0.14 0.14"/>
        </geometry>
        <origin xyz="0.9 -1.25 0" rpy="0 0 1.57"/>
      </visual>
      <collision>
        <geometry>
          <mesh filename="package://rosbots_description/meshes/rasp.dae" scale="0.14 0.14 0.14"/>
        </geometry>
        <origin xyz="0.9 -1.25 0" rpy="0 0 1.57"/>
      </collision>
      <!-- inertial>
        <origin xyz="0.01 0 0"/>
        <mass value="1.0"/>
        <inertia ixx="0.019995" ixy="0.0" ixz="0.0"
                 iyy="0.019995" iyz="0.0" 
                 izz="0.03675" />
      </inertial-->
    </link>
​
   <joint name="mcu_joint_1" type="fixed">
      <origin xyz="0.02 0.12 0.83" rpy="0 0 0" />
      <parent link="base_link"/>
      <child link="mcu_link_1" />
    </joint>
    <link name="mcu_link_1">
      <visual>
        <geometry>
          <mesh filename="package://rosbots_description/meshes/rasp.dae" scale="0.14 0.14 0.14"/>
        </geometry>
        <origin xyz="0.9 -1.25 0" rpy="0 0 1.57"/>
      </visual>
      <collision>
        <geometry>
          <mesh filename="package://rosbots_description/meshes/rasp.dae" scale="0.14 0.14 0.14"/>
        </geometry>
        <origin xyz="0.9 -1.25 0" rpy="0 0 1.57"/>
      </collision>
      <!-- inertial>
        <origin xyz="0.01 0 0"/>
        <mass value="1.0"/>
        <inertia ixx="0.019995" ixy="0.0" ixz="0.0"
                 iyy="0.019995" iyz="0.0" 
                 izz="0.03675" />
      </inertial-->
    </link>
​
​
   <joint name="stand_mcu1_joint" type="fixed">
      <origin xyz="0.02 0.25 0.78" rpy="0 0 1.57" />
      <parent link="base_link"/>
      <child link="stand_mcu1_link" />
    </joint>
    <link name="stand_mcu1_link">
      <visual>
        <geometry>
         <cylinder length="0.1" radius="0.01"/>
        </geometry>
        <origin rpy="0.0 0 0" xyz="0 0 0.0"/>   
      </visual>
      <collision>
        <geometry>
         <cylinder length="0.1" radius="0.01"/>
        </geometry>
        <origin rpy="0.0 0 0" xyz="0 0 0.0"/>
      </collision>
    </link>
​
   <joint name="stand_mcu2_joint" type="fixed">
      <origin xyz="0.02 -0.1125 0.78" rpy="0 0 1.57" />
      <parent link="base_link"/>
      <child link="stand_mcu2_link" />
    </joint>
    <link name="stand_mcu2_link">
      <visual>
        <geometry>
         <cylinder length="0.1" radius="0.01"/>
        </geometry>
        <origin rpy="0.0 0 0" xyz="0 0 0.0"/>   
      </visual>
      <collision>
        <geometry>
         <cylinder length="0.1" radius="0.01"/>
        </geometry>
        <origin rpy="0.0 0 0" xyz="0 0 0.0"/>
      </collision>
    </link>
​
​
   <joint name="stand_mcu3_joint" type="fixed">
      <origin xyz="0.25 0.25 0.78" rpy="0 0 1.57" />
      <parent link="base_link"/>
      <child link="stand_mcu3_link" />
    </joint>
    <link name="stand_mcu3_link">
      <visual>
        <geometry>
         <cylinder length="0.1" radius="0.01"/>
        </geometry>
        <origin rpy="0.0 0 0" xyz="0 0 0.0"/>   
      </visual>
      <collision>
        <geometry>
         <cylinder length="0.1" radius="0.01"/>
        </geometry>
        <origin rpy="0.0 0 0" xyz="0 0 0.0"/>
      </collision>
    </link>
​
   <joint name="stand_mcu4_joint" type="fixed">
      <origin xyz="0.25 -0.1125 0.78" rpy="0 0 1.57" />
      <parent link="base_link"/>
      <child link="stand_mcu4_link" />
    </joint>
    <link name="stand_mcu4_link">
      <visual>
        <geometry>
         <cylinder length="0.1" radius="0.01"/>
        </geometry>
        <origin rpy="0.0 0 0" xyz="0 0 0.0"/>   
      </visual>
      <collision>
        <geometry>
         <cylinder length="0.1" radius="0.01"/>
        </geometry>
        <origin rpy="0.0 0 0" xyz="0 0 0.0"/>
      </collision>
    </link>
​
    <joint name="batery_joint" type="fixed">
      <origin xyz="1.2 0.2 0.43" rpy="0 0 1.57" />
      <parent link="base_link"/>
      <child link="batery_link" />
    </joint>
    <link name="batery_link">
      <visual>
        <geometry>
          <mesh filename="package://rosbots_description/meshes/battery.dae" scale="13.0 6.0 6.0"/>
        </geometry>
        <origin xyz="0 4.5 0.05" rpy="1.57 0 1.57"/>    
      </visual>
      <collision>
        <geometry>
          <mesh filename="package://rosbots_description/meshes/battery.dae" scale="13.0 6.0 6.0"/>
        </geometry>
        <origin xyz="0 4.5 0.05" rpy="1.57 0 1.57"/>
      </collision>
      <!-- inertial>
        <origin xyz="0.01 0 0"/>
        <mass value="1.0"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0"
                 iyy="0.01" iyz="0.0" 
                 izz="0.03" />
      </inertial-->
    </link>
​
   <joint name="stand_1_joint" type="fixed">
      <origin xyz="0.5 0.4125 0.58" rpy="0 0 1.57" />
      <parent link="base_link"/>
      <child link="stand_1_link" />
    </joint>
    <link name="stand_1_link">
      <visual>
        <geometry>
         <cylinder length="0.20" radius="0.03"/>
        </geometry>
        <origin rpy="0.0 0 0" xyz="0 0 0.0"/>   
      </visual>
      <collision>
        <geometry>
         <cylinder length="0.20" radius="0.03"/>
        </geometry>
        <origin rpy="0.0 0 0" xyz="0 0 0.0"/>
      </collision>
      <!--inertial>
        <origin xyz="0.0 0 0"/>
        <mass value="1.0"/>
        <inertia ixx="0.019995" ixy="0.0" ixz="0.0"
                 iyy="0.019995" iyz="0.0" 
                 izz="0.03675" />
      </inertial-->
    </link>
​
   <joint name="stand_2_joint" type="fixed">
      <origin xyz="0.5 -0.2625 0.58" rpy="0 0 1.57" />
      <parent link="base_link"/>
      <child link="stand_2_link" />
    </joint>
    <link name="stand_2_link">
      <visual>
        <geometry>
         <cylinder length="0.20" radius="0.03"/>
        </geometry>
        <origin rpy="0.0 0 0" xyz="0 0 0.0"/>   
      </visual>
      <collision>
        <geometry>
         <cylinder length="0.20" radius="0.03"/>
        </geometry>
        <origin rpy="0.0 0 0" xyz="0 0 0.0"/>
      </collision>
      <!--inertial>
        <origin xyz="0.0 0 0"/>
        <mass value="1.0"/>
        <inertia ixx="0.019995" ixy="0.0" ixz="0.0"
                 iyy="0.019995" iyz="0.0" 
                 izz="0.03675" />
      </inertial-->
    </link>
​
    <joint name="wheel_left_joint" type="continuous">
      <parent link="base_link"/>
      <child link="wheel_left_link"/>
      <origin xyz="0.15 0.4125 0.30" rpy="-1.57 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>
    <link name="wheel_left_link">
      <collision>
        <geometry>
         <cylinder length="0.20" radius="0.23"/>
        </geometry>
        <origin rpy="0.0 0.0 0" xyz="0 0 0.1"/>
      </collision>
      <visual name="visual">
        <geometry>
          <!-- cylinder length="0.0206" radius="0.0550"/-->
          <!--cylinder length="0.20" radius="0.26"/-->
          <mesh filename="package://rosbots_description/meshes/wheel.dae" scale="8.0 8.0 8.0"/>
        </geometry>
        <origin rpy="0.0 0 0" xyz="0 0 0"/>
      </visual>
      <inertial>
        <mass value="0.4" />
        <origin xyz="0 0 0" />
        <inertia ixx="0.001" ixy="0.0" ixz="0.0"
                 iyy="0.001" iyz="0.0" 
                 izz="0.001" />
      </inertial>
    </link>
    
    <joint name="wheel_right_joint" type="continuous">
      <parent link="base_link"/>
      <child link="wheel_right_link"/>
      <origin xyz="0.15 -0.5625 0.30" rpy="-1.57 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>
    <link name="wheel_right_link">
      <collision>
        <geometry>
         <cylinder length="0.20" radius="0.23"/>
        </geometry>
        <origin rpy="0.0 0 0" xyz="0 0 0.195"/>
      </collision>
      <visual name="visual">
        <geometry>
      <!--cylinder length="0.20" radius="0.26"/-->
          <mesh filename="package://rosbots_description/meshes/wheel.dae" scale="8.0 8.0 8.0"/>
        </geometry>
        <origin rpy="0.0 0 0" xyz="0 0 0"/>
      </visual>
      <inertial>
        <mass value="0.4" />
        <origin xyz="0 0.0 0.3" />
        <inertia ixx="0.001" ixy="0.0" ixz="0.0"
                 iyy="0.001" iyz="0.0" 
                 izz="0.001" />
      </inertial>
    </link>
   
    <joint name="caster_back_joint" type="fixed">
      <parent link="base_link"/>
      <child link="caster_back_link"/>
      <origin xyz="-0.4 0.1 0.26" rpy="0 0 0"/>
    </joint>
    <link name="caster_back_link">
      <collision>
        <geometry>
          <!-- cylinder length="0.05" radius="0.19"/-->
          <sphere radius="0.19"/>
        </geometry>
        <origin rpy="0 0 0" xyz="0 0 0"/>
      </collision>    
      <visual>
        <geometry>
          <!--cylinder length="0.05" radius="0.19"/-->
          <sphere radius="0.19"/>
        </geometry>
        <origin rpy="0 0 0" xyz="0 0 0"/>
      </visual>   
      <inertial>
        <mass value="0.1" />
        <origin xyz="0 0 0" />
        <inertia ixx="0.001023539" ixy="0.0" ixz="0.0"
                 iyy="0.001023539" iyz="0.0" 
                 izz="0.001023539" />
      </inertial>
    </link>
​
   <joint name="stand_3_joint" type="fixed">
      <origin xyz="-0.4 0.4125 0.58" rpy="0 0 1.57" />
      <parent link="base_link"/>
      <child link="stand_3_link" />
    </joint>
    <link name="stand_3_link">
      <visual>
        <geometry>
         <cylinder length="0.20" radius="0.03"/>
        </geometry>
        <origin rpy="0.0 0 0" xyz="0 0 0.0"/>   
      </visual>
      <collision>
        <geometry>
         <cylinder length="0.20" radius="0.03"/>
        </geometry>
        <origin rpy="0.0 0 0" xyz="0 0 0.0"/>
      </collision>
    </link>
​
   <joint name="stand_4_joint" type="fixed">
      <origin xyz="-0.4 -0.2625 0.58" rpy="0 0 1.57" />
      <parent link="base_link"/>
      <child link="stand_4_link" />
    </joint>
    <link name="stand_4_link">
      <visual>
        <geometry>
         <cylinder length="0.20" radius="0.03"/>
        </geometry>
        <origin rpy="0.0 0 0" xyz="0 0 0.0"/>   
      </visual>
      <collision>
        <geometry>
         <cylinder length="0.20" radius="0.03"/>
        </geometry>
        <origin rpy="0.0 0 0" xyz="0 0 0.0"/>
      </collision>
    </link>
​
</robot>
**rosbots.xacro**

c) As you can see from the file above, we are loading different meshes (.dae files) that contain the shapes and textures of some of the elements of the URDF file. In order to have these meshes available in your package, just execute the following command:

Execute in WebShell #1

roscd duckietown_description;
cp -rf meshes ~/catkin_ws/src/rosbots_description/
Now, you should have the meshes folder inside your rosbots_description package.

**End of Exercise 2.1**

So, in the xacro file above, we have basically defined the following:

Robot Chasis
Wheels (left + right) and Caster Wheel
Joints
Some extra elements, like the battery or the Raspberry
All of these elements have some common properties, like inertial, collision, and visual. The inertial and collision properties enable physics simulations, and the visual property controls the appearance of the robot.

The joints help in defining the relative motion between links, such as the motion of the wheel with respect to the chassis. The wheels are links of cylindrical geometry, and they are oriented (using rpy property) in a way that makes rolling possible. The placement is controlled via the xyz property. For joints, we have specified the values for damping and friction as well.

Great! So all of this is very nice, but... where's my robot? I don't see it! Well, in order to visualize our robot, we are going to use the RViz tool. So, let's use the following exercise to see how we can visualize our robot!

**Exercise 2.2**

a) To visualize the robot we just defined, we will create a launch file named rviz.launch inside the launch folder. This launch file will start RViz. We will populate it with the following content:

**rviz.launch**

<?xml version="1.0"?>
<launch>
 
  <param name="robot_description" command="cat '$(find rosbots_description)/urdf/rosbots.xacro'"/>
 
  <!-- send fake joint values -->
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <param name="use_gui" value="False"/>
  </node>
 
  <!-- Combine joint values -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher"/>
 
  <!-- Show in Rviz   -->
  <node name="rviz" pkg="rviz" type="rviz" />
 
</launch>
**rviz.launch**

b) Now to launch it, use the following command:

Execute in WebShell #1

roslaunch rosbots_description rviz.launch
Once the node is launched, we need to open the Graphical Tools window, which will help us see the RViz screen.

Hit the icon with a screen in the top-right corner of the IDE windowto open the Graphic Interface window.

Once the RViz window loads, do the following:

Change the frame to base_link in the Fixed Frame option
Add a Robot Description display
You should get something like this:



**End of Exercise 2.2**

Awesome! So now we have created an URDF file that defines how our robot looks. Now, all we have done so far is just visualize our robot. But at this point, we can't yet test anything on it. For that, we'll need to simulate our robot in Gazebo, which is the simulation environment.

In order to load our robot into Gazebo, we will need to create another launch file. Let's follow the next exercise to see how to do that!

**Exercise 2.3**

a) To spawn the robot we have just created into the Gazebo simulation, we will create a launch file named spawn.launch inside the launch folder. We will populate it with the following content:

**spawn.launch**

<?xml version="1.0" encoding="UTF-8"?>
<launch>
    <param name="robot_description" command="cat '$(find rosbots_description)/urdf/rosbots.xacro'" />
 
    <arg name="x" default="0"/>
    <arg name="y" default="0"/>
    <arg name="z" default="0.5"/>
 
    <node name="mybot_spawn" pkg="gazebo_ros" type="spawn_model" output="screen"
          args="-urdf -param robot_description -model rosbots -x $(arg x) -y $(arg y) -z $(arg z)" />
 
</launch>
**spawn.launch**

b) So, in order to spawn our robot within the Gazebo simulation, use the following command:

Execute in WebShell #1

roslaunch rosbots_description spawn.launch
You should now see the robot spawning in the Gazebo simulation window:



**End of Exercise 2.3**

Great! So now we are able to spawn our robot into the simulation! That's really nice, but let me tell you that you aren't capable of controlling your robot just yet. You can't move it or drive around, for instance. For that, you will need to add control to the robot, by adding some elements called plugins.

Let's follow the next exercise to see how to add control to our robot!

**Exercise 2.4**

a) Add the following code snippet into your rosbots.xacro file, inside the <robot> tag.

<gazebo>
  <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
    <legacyMode>false</legacyMode>
    <alwaysOn>true</alwaysOn>
    <publishWheelTF>true</publishWheelTF>
    <publishTf>1</publishTf>
    <publishWheelJointState>true</publishWheelJointState>
    <updateRate>100.0</updateRate>
    <leftJoint>wheel_left_joint</leftJoint>
    <rightJoint>wheel_right_joint</rightJoint>
    <wheelSeparation>1.1</wheelSeparation>
    <wheelDiameter>0.52</wheelDiameter>
    <wheelAcceleration>1.0</wheelAcceleration>
    <torque>20</torque>
    <commandTopic>/part2_cmr/cmd_vel</commandTopic>
    <odometryTopic>odom</odometryTopic>
    <odometryFrame>odom</odometryFrame>
    <robotBaseFrame>base_link</robotBaseFrame>
  </plugin>
</gazebo>
b) Now, let's spawn our robot into the simulation again. To be able to spawn it again, though, we will need to first remove the existing one, which we added in the previous exercise. To do this, you can execute the following command.

Execute in WebShell #1

rosservice call /gazebo/delete_model "model_name: 'rosbots'"
Now, with an empty simulation again, we can spawn the robot with the updated URDF file.

Execute in WebShell #1

roslaunch rosbots_description spawn.launch
Now, if everything goes as expected, we will be able to control the robot. We can check this by listing the available topics using the following command:

Execute in WebShell #1

rostopic list
You should now see the following topic in your topics list:

/part2_cmr/cmd_vel
c) Finally, to control the motion of the robot, we can use the keyboard_teleop program, which allows you to publish motion commands using the keyboard. To launch it, execute the following command:

rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/part2_cmr/cmd_vel
Now we can make the robot navigate with keyboard keys.

IMPORTANT NOTE!!: In the current Exercise you have just added a Gazebo plugin to your robot, which will allow it to move around. This is pretty cool, but there is one inconvenient. Whenever you delete a model that has a Gazebo plugin, the Gazebo simulation will crash. This is a known issue with Gazebo, but it is quite incovenient for this Unit, since we have been spawning/deleting our robot several times.

So, from now on, in order to delete the model so that you can spawn an updated one, you will have to change to another Unit (ie, Unit 0) and come back to this one. This way, the Gazbo simulation will be reset and you will be able to spawn again your robot on it.

**Exercise 2.4**

Using the XACRO Macros
So far, we've achieved the following:

Created a xacro file that contains the urdf description of our robot
Created a launch file to spawn the robot in the Gazebo simulation environment
Controlled the simulated robot using keyboard teleoperation
In this part, we will organize the existing project to make it more readable and modular. Even though our robot description only had a few components, we had written a lengthy xacro file. So, let's start doing some modifications to our files in the following exercise!

**Exercise 2.5**

a) In our robot spawn launch file, we used the following line:

<param name="robot_description" command="cat '$(find rosbots_description)/urdf/rosbots.xacro'" />
Here we are using the cat command to load the contents of the rosbtos.xacro file into the robot_description parameter. However, to really use the features of the xacro file, we need to parse and execute the xacro file. And to achieve that, we will modify the above line to:

<param name="robot_description" command="$(find xacro)/xacro.py '$(find rosbots_description)/urdf/rosbots.xacro'" />
The above command uses the xacro.py file to execute the instructions of the rosbots.xacro file.

Modify the spawn.launch file so that it uses the xacro.py file.

b) A similar edit is needed for the rviz.launch file as well. So, modify this file so that it also uses the xacro.py file.

<param name="robot_description" command="cat '$(find rosbots_description)/urdf/rosbots.xacro'" />
to

<param name="robot_description" command="$(find xacro)/xacro.py  '$(find rosbots_description)/urdf/rosbots.xacro'"/>
c) Test your launch files again and check that everything is still working fine. You might get some warnings like the following ones:

NOTE: Remember that, in order to delete the model spawned in the previous Exercise, you will have to switch to another Unit and come back to this one.



**End of Exercise 2.5**

At the moment, our xacro file does not contain any instructions. So, we will now split the rosbots.xacro file into smaller files, and using the features of xacro, we will assimilate the smaller files.

Basically, we will move all the gazebo tags from the rosbots.xacro file to a new file.

**Exercise 2.6**

a) Create a new xacro file named rosbots.gazebo.xacro inside the URDF folder. You will populate it with the following content, which basically is the <gazebo> tag that you added to the rosbots.xacro file in Exercise 2.4:

**rosbots.gazebo.xacro**

<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="rosbots" >
  
  <gazebo>
    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <legacyMode>false</legacyMode>
      <alwaysOn>true</alwaysOn>
      <publishWheelTF>true</publishWheelTF>
      <publishTf>1</publishTf>
      <publishWheelJointState>true</publishWheelJointState>
      <updateRate>100.0</updateRate>
      <leftJoint>wheel_left_joint</leftJoint>
      <rightJoint>wheel_right_joint</rightJoint>
      <wheelSeparation>1.1</wheelSeparation>
      <wheelDiameter>0.52</wheelDiameter>
      <wheelAcceleration>1.0</wheelAcceleration>
      <torque>20</torque>
      <commandTopic>/part2_cmr/cmd_vel</commandTopic>
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <robotBaseFrame>base_link</robotBaseFrame>
    </plugin>
  </gazebo>
​
</robot>
**rosbots.gazebo.xacro**

Also, remember to remove this <gazebo> tag from the main rosbots.xacro file.

b) We will now have to include this new file in our rosbots.xacro file. We can do that by using the following line:

<xacro:include filename="$(find rosbots_description)/urdf/rosbots.gazebo.xacro" />
Add this line at the beginning of your rosbots.xacro file, inside the <robot> tag.

c) To see whether everything works, let's spawn the robot again using the updated Xacro files.

NOTE: Remember that, in order to delete the model spawned in the previous Exercise, you will have to switch to another Unit (ie Unit 0) and come back to this one so that the Gazebo simulation is reset.

Execute in WebShell #1

roslaunch rosbots_description spawn.launch
You should see the robot being spawned:



Also, if executing a rostopic list, you should see the /part2_cmr/cmd_vel topic among the listed topics.

**End of Exercise 2.6**

Adding the sensors to the robot
Now, we will add all the sensors to our robot's URDF model. For that, we will need to modify both rosbots.xacro and rosbots.gazebo.xacro files. For our robot, we will be adding two sensors: a camera and an IMU.

Let's start by adding the camera. For that, we'll need to do several things. First, in the rosbots.xacro file, we will do the following:

Add a link element to our robot. This link will be represented by a dae file.
Add a joint element to our robot. This will connect the sensor to the robot body rigidly.
Also, in the rosbots.gazebo.xacro file, we will do the following:

Add a plugin element that will add sensing ability to the link that we created (the cylinder representing the sensor).
Let's follow the next exercises to see how we can do that!

**Exercise 2.7**

a) Open the rosbots.xacro file and add the following elements into the <robot> tag:

<joint name="camera_joint" type="fixed">
  <origin xyz="0.49 -0.03 0.75" rpy="0 0.21 0" />
  <parent link="base_link"/>
  <child link="camera_link" />
</joint>
    
<link name="camera_link">
  <visual>
    <geometry>
      <mesh filename="package://duckietown_description/meshes/camera.dae" scale="4.0 4.0 4.0"/>
    </geometry>
    <origin xyz="0.0 0 0" rpy="0 0 0"/>
  </visual>
  <collision>
    <geometry>
      <mesh filename="package://duckietown_description/meshes/camera.dae" scale="4.0 4.0 4.0"/>
    </geometry>
    <origin xyz="0.0 0 0" rpy="0 0 0"/>
  </collision>
</link>
The above code will add the camera joint and link, and will allow us to visualize the camera on the robot.

b) Let's test everything by spawning our robot into the simulation again.

NOTE: Remember that, in order to delete the model spawned in the previous Exercise, you will have to switch to another Unit (ie Unit 0) and come back to this one so that the Gazebo simulation is reset.

Execute in WebShell #1

roslaunch rosbots_description spawn.launch
You should now see the robot with a camera mounted on top of it:



**End of Exercise 2.7**

Great! So now we have added a camera to our robot. Well, actually, what we have really done is add the visualization of the camera. But it won't behave as a camera just yet. At this point, the camera is not able to take images or record videos.

So next, we need to add the camera behavior to the link. To do so, we will use the camera gazebo plugin. You can check out more information about this plugin here.

**Exercise 2.8**

a) Open the rosbots.gazebo.xacro file and add the following plugin element:

<gazebo reference="camera_link">
    <sensor type="camera" name="camera1">
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.04</horizontal_fov>
        <image>
          <width>320</width>
          <height>240</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>50</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>0</updateRate>
        <cameraName>camera1</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>camera</frameName>
        <hackBaseline>0.07</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
      </plugin>
    </sensor>
  </gazebo>
As you can see, within this code we specify some parameters related to the camera's behavior:

update_rate: Controls how often (how fast) the camera data is captured.
width/height: Defines the resolution of the images taken. In this case, 320x240.
format: Defines the format of the textures.
imageTopicName: Defines the topic name, which is used for publishing the camera data.
frameName: Defines the link to which the plugin has to be applied.
b) With this plugin incorporated in our URDF file, we are now ready to simulate and visualize the camera's behavior. Let's spawn the robot again by launching the spawn.launch file:

NOTE: Remember that, in order to delete the model spawned in the previous Exercise, you will have to switch to another Unit (ie Unit 0) and come back to this one so that the Gazebo simulation is reset.

Execute in WebShell #1

roslaunch rosbots_description spawn.launch
c) To verify that the camera is actually working correctly, check the list of topics with the following command:

Execute in WebShell #1

rostopic list
You should get the following topics now:

/rosbots/camera1/camera_info
/rosbots/camera1/image_raw
/rosbots/camera1/image_raw/compressed
/rosbots/camera1/image_raw/compressed/parameter_descriptions
/rosbots/camera1/image_raw/compressed/parameter_updates
/rosbots/camera1/image_raw/compressedDepth
/rosbots/camera1/image_raw/compressedDepth/parameter_descriptions
/rosbots/camera1/image_raw/compressedDepth/parameter_updates
/rosbots/camera1/image_raw/theora
/rosbots/camera1/image_raw/theora/parameter_descriptions
/rosbots/camera1/image_raw/theora/parameter_updates
/rosbots/camera1/parameter_descriptions
/rosbots/camera1/parameter_updates
c) We will now visualize the camera data with RViz. First, we will populate the simulated environment with an obstacle, to better see the camera's result. Use the following commands to do so:

Execute in WebShell #1

cp /home/simulations/public_sim_ws/src/all/turtlebot/turtlebot_navigation_gazebo/urdf/object.urdf /home/user/catkin_ws/src
rosrun gazebo_ros spawn_model -file /home/user/catkin_ws/src/object.urdf -urdf -x 1 -y 0 -z 1 -model my_object
After the obstacle is spawned, you should get something like this:



To start RViz visualization, launch the rviz.launch using the following command:

Execute in WebShell #1

roslaunch rosbots_description rviz.launch
After starting RViz, you will need to apply the following settings:

Select base_link in the Fixed Frame field.
Add two new displays using the Add button on the bottom-left of the RViz screen. The first display should be RobotModel and the other one should be Image.
Expand the image display by double-clicking on its name and set the Image Topic to /rosbots/camera1/image_raw (as shown in the image below).


**End of Exercise 2.8**

Great! So at this point, we have our robot with a camera incorporated on it, which provides us with an image feed. That's great! But let's add still one last sensor, the IMU.

**Exercise 2.9**

a) Open the rosbots.gazebo.xacro file and add the following plugin element:

<gazebo>
    <plugin name="gazebo_ros_imu_controller" filename="libgazebo_ros_imu.so">
      <!-- <robotNamespace></robotNamespace> -->
      <topicName>imu/data</topicName>
      <serviceName>imu/service</serviceName>
      <bodyName>base_link</bodyName>
      <gaussianNoise>0</gaussianNoise>
      <rpyOffsets>0 0 0</rpyOffsets>
      <updateRate>30.0</updateRate>
      <alwaysOn>true</alwaysOn>
      <gaussianNoise>0</gaussianNoise>
    </plugin>
  </gazebo>
As you can see, we are going to publish the IMU data into a topic called /imu/data.

b) With this plugin incorporated in our URDF file, we are now ready to simulate the IMU behavior. Let's spawn the robot again by launching the spawn.launch file:

NOTE: Remember that, in order to delete the model spawned in the previous Exercise, you will have to switch to another Unit (ie Unit 0) and come back to this one so that the Gazebo simulation is reset.

Execute in WebShell #1

roslaunch rosbots_description spawn.launch
c) To verify that the IMU sensor is actually working correctly, check the list of topics with the following command:

Execute in WebShell #1

rostopic list
You should get the following topics now:

/imu/data
d) We can get more information about this topic by using the following command:

Execute in WebShell #1

rostopic info /imu/data
You should see something like this:

Type: sensor_msgs/Imu
​
Publishers:
 * /gazebo (http://10.8.0.1:43002/)
​
Subscribers: None
As you can see, our IMU topic uses messages of the sensor_msgs/Imu type, and we can also see that the gazebo node is publishing data into it.

You can also have a look at how the IMU data looks by using the following command:

Execute in WebShell #1

rostopic echo /imu/data -n1
You should get something like this:

header:
  seq: 0
  stamp:
    secs: 503
    nsecs: 649000000
  frame_id: "base_footprint"
orientation:
  x: 3.02585927105e-05
  y: 0.000935318146935
  z: 0.0058563840343
  w: 0.999982413361
orientation_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity:
  x: -2.41480080677e-05
  y: -7.96171695717e-05
  z: -8.40822726502e-06
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration:
  x: 1.29284634024e-07
  y: -2.05529541187e-08
  z: 5.84459362599e-05
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
So basically, the IMU provides us with the following data:

orientation:: Orientation of the robot in x, y, z, and w.
angular_velocity: Angular velocity of the robot.
linear_acceleration: Linear acceleration of the robot.
The covariance indicates the error on those measures.

**End of Exercise 2.9**

Awesome! So, now we have our simulated robot fully functional, able to move around, capture images using its camera, and provide us with data from its IMU sensor. Let's just do one last exercise, where we are going to complete the URDF files with some specific data.

**Exercise 2.10**

a) Open the rosbots.gazebo.xacro file and add the following elements at the beginning of the file, inside the <robot> tag:

<gazebo reference="wheel_left_link">
  <mu1>1.0</mu1>
  <mu2>1.0</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
  <minDepth>0.001</minDepth>
  <maxVel>1.0</maxVel>
</gazebo>
​
<gazebo reference="wheel_right_link">
  <mu1>1.0</mu1>
  <mu2>1.0</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
  <minDepth>0.001</minDepth>
  <maxVel>1.0</maxVel>
</gazebo>
​
<gazebo reference="base_link">
  <material>Gazebo/Blue</material>
  <mu1>0.3</mu1>
  <mu2>0.3</mu2>
  <sensor type="contact" name="bumpers">
    <always_on>1</always_on>
    <update_rate>50.0</update_rate>
    <visualize>true</visualize>
    <contact>
      <collision>base_footprint_collision_base_link</collision>
    </contact>
  </sensor>
</gazebo>
​
<gazebo reference="camera_link">
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>
The above code will specify some extra parameters, like the frictions between some of the links, or the maximum velocity of the wheels.

b) Let's spawn our robot one last time by launching the spawn.launch file:

NOTE: Remember that, in order to delete the model spawned in the previous Exercise, you will have to switch to another Unit (ie Unit 0) and come back to this one so that the Gazebo simulation is reset.

Execute in WebShell #1

roslaunch rosbots_description spawn.launch
Check that everything works correctly:

Move the robot around
Visualize the images captured by the camera
Check that the IMU data is accurate
**End of Exercise 2.10**

