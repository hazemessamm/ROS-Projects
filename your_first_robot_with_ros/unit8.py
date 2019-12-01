Loading [MathJax]/extensions/Safe.js
Building and Rosifying a Robot from Scratch


Unit 6: Deep Learning with RIAbot
SUMMARY

Estimated time to completion: 4 hours

In this unit, you are going to be following, step by step, the full workflow of training your ROSbots robot into following lanes, including all the environments and scripts involved in that training.

END OF SUMMARY

The main goal of this chapter will be to try to explain, in the simplest way possible, how a training session using OpenAI, ROS, and Gazebo will be structured and what the full workflow will be. And our main goal will be getting the ROSbots to learn how to follow the lane(s) detected from the image stream from its camera using Deep Reinforcement Learning.



We will make use of the openai_ros package to train the robot within the simulation, and TensorFlow.

The openai_ros package provides a common structure for organizing everything you need to create your robot training from zero, requiring very little implementation. It is basically composed of the following elements:

It contains the Gazebo Environment class that connects your OpenAI programs to Gazebo.
It provides a group of already made Robot Environments for the most popular ROS-based robots. The Robot Environments provide the complete integration between the Gazebo simulation of the robot and the OpenAI algorithm environments, so obtaining sensor information from the robot or sending actions to it are ROS transparent to the OpenAI algorithms and to you, the developer.
It provides a group of already made Task Environments that you can use together with the RobotEnvironments and GazeboEnvironment to train a robot in the task defined in the TaskEnvironment.
It provides a set of templates to help you create your own Robot Environments and Task Environments, which directly connect to Gazebo (because they inherit from GazeboEnvironment)


So, summarizing, we can divide everything into the following two groups:

Training Script: The training script will define and set up the learning algorithm that you are going to use to train your agent.
Training Environments: The training environments will be the ones in charge of providing all the needed data to your learning algorithm in order to make the agent learn. There are different types of Training Environments: Gazebo Environment, Robot Environment, and Task Environment.
Training Environments
An environment is, basically, a problem (like the CartPole) with a minimal interface with which an agent (a robot) can interact. The environments in the OpenAI Gym are designed to allow objective testing and bench-marking of an agent's abilities.

In basic OpenAI systems, you can usually define everything with just one environment. But this is a more complex situation, since we want to integrate OpenAI with ROS and Gazebo. We could probably define everything in just one environment, but it would be a complete mess, and the code would be incomprehensible.

That's why we have created a structure to organize and split these environments in the same way every time, so that it is easier to understand and to work with them. This structure divides the OpenAI environment into three different types:

Task Environment: This environment is dedicated to implementing all the functions that a specific training session will use. For instance, using the same Robot Environment, you could create different Task Environments in order to train your robot in different tasks, or using different methods (actions).
Robot Environment: This environment is dedicated to implementing all the functions that a specific robot will use during its training. It will not include, though, the specific tasks that depend on the training that you are going to implement.
Gazebo Environment: This environment is common for any training or robot that you want to implement. This environment will generate all the connections between your robot and Gazebo, so that you don't have to worry about it.
It's very important that you understand that this environment structure will always be the same, regardless of the problem you want to solve or the learning algorithm that you want to use. Also, these three environments will be connected to each other in the sense that each environment will inherit from the previous one. It goes like this:

Task Environment inherits from the Robot Environment.
Robot Environment inherits from the Gazebo Environment.
Gazebo Environment inherits from the Gym Environment. The Gym Environment (gym.Env) is the most basic environment structure provided by OpenAI.
Task Environment
The Task Environment is the highest one in the inheritance structure, which also means that it is the most specific one. This environent will fully depend on the kind of training we want to create.

The Task Environment will tell the following to the openai_ros package:

How to get an observation for the Reinforcement Learning algorithm
How to execute an action
How to compute the reward
How to detect when the task is done
Next, you can have a look at an example Task Environment that can be used for training the ROSbots to follow lanes:

**rosbots_lane_follow_env.py**

import rospy
import numpy
import time
from scipy.interpolate import interp1d
from gym import spaces
import rosbots_env
from gym.envs.registration import register
​
import sys
import rospkg
rospack = rospkg.RosPack()
detection_pkg_path = rospack.get_path('line_follower')
sys.path.insert(0, detection_pkg_path + "/scripts")
​
from line_follow import LineFollower
​
# The path is __init__.py of openai_ros, where we import the TurtleBot2MazeEnv directly
timestep_limit_per_episode = 20000  # Can be any Value
​
register(
    id='RosBotsEnv-v0',
    entry_point='rosbots_lane_follow_env:RosBotsEnv',
    timestep_limit=timestep_limit_per_episode,
)
​
​
class RosBotsEnv(rosbots_env.RosBotsEnv):
    def __init__(self):
        """
        This Task Env is designed for allowing the ROSbots to navigate following lanes. Using the robot's visual sensor.
        It will learn how to move around the town without goin off-course.
        """
        # Only variable needed to be set here
        number_actions = rospy.get_param('/rosbots/n_actions')
        self.action_space = spaces.Discrete(number_actions)
​
        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-numpy.inf, numpy.inf)
​
        # number_observations = rospy.get_param('/turtlebot2/n_observations')
        """
        We set the Observation space
        cube_observations = [
            round([camera_feed])
        ]
        """
​
        # Actions and Observations
        self.linear_forward_speed = rospy.get_param(
            '/rosbots/linear_forward_speed')
        self.linear_turn_speed = rospy.get_param('/rosbots/linear_turn_speed')
        self.angular_speed = rospy.get_param('/rosbots/angular_speed')
        self.init_linear_forward_speed = rospy.get_param(
            '/rosbots/init_linear_forward_speed')
        self.init_linear_turn_speed = rospy.get_param(
            '/rosbots/init_linear_turn_speed')
​
        # Control Parameters
        self.dMax = rospy.get_param('/rosbots/max_lane_offset')
        self.dSafe = rospy.get_param('/rosbots/safe_lane_offset')
        self.look_ahead_distance = rospy.get_param(
            '/rosbots/look_ahead_distance')
​
        # Rewards
        self.follow_lane_reward = rospy.get_param(
            "/rosbots/follow_lane_reward")
        self.left_right_reward = rospy.get_param("/rosbots/left_right_reward")
        self.veer_off_reward = rospy.get_param("/rosbots/veer_off_reward")
        self.end_episode_points = rospy.get_param(
            "/rosbots/end_episode_points")
​
        self.action_taken = None
​
        self.cumulated_steps = 0.0
​
        # Here we will add any init functions prior to starting the MyRobotEnv
        super(RosBotsEnv, self).__init__()
​
        # Instantiate the image processing class
        self.line_follow = LineFollower()
​
        # low = numpy.array([0.0])
        # high = numpy.array([19200.0])
​
        # We only use two integers #CHANGED THE OBSERVATION SPACE
        # self.observation_space = spaces.Box(low, high) #observation space boundary
        self.observation_space = spaces.Box(0, 255, shape=(80, 80, 3))
​
        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>" +
                       str(self.observation_space))
​
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        self.line_follow.clean_up()
        self.move_base(self.init_linear_forward_speed,
                       self.init_linear_turn_speed,
                       epsilon=0.05,
                       update_rate=10,
                       use_offset=False)  # Do not check for lane
​
        return True
​
    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        # For Info Purposes
        self.cumulated_reward = 0.0
        # Set to false Done, because its calculated asyncronously
        self._episode_done = False
​
        # We wait a small ammount of time to start everything because in very fast resets, laser scan values are sluggish
        # and sometimes still have values from the prior position that triguered the done.
        time.sleep(0.2)
​
    def _set_action(self, action):
        """
        This set action will Set the linear and angular speed of the ROSbots
        based on the action number given.
        :param action: The action integer that set s what movement to do next.
        """
​
        rospy.logdebug("Start Set Action ==>"+str(action))
        self.action_taken = action
        # We convert the actions to speed movements to send to the parent class CubeSingleDiskEnv
        if action == 0:  # FORWARD
            linear_speed = self.linear_forward_speed
            angular_speed = 0.0
            self.last_action = "FORWARD"
        elif action == 1:  # LEFT
            linear_speed = self.linear_turn_speed
            angular_speed = self.angular_speed
            self.last_action = "TURN_LEFT"
        elif action == 2:  # RIGHT
            linear_speed = self.linear_turn_speed
            angular_speed = -1*self.angular_speed
            self.last_action = "TURN_RIGHT"
​
        # We tell ROSbots the linear and angular speed to set to execute
        self.move_base(linear_speed,
                       angular_speed,
                       epsilon=0.05,
                       update_rate=10)
​
        rospy.logdebug("END Set Action ==>"+str(action))
​
    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have acces to, we need to read the
        RosBotsEnv API DOCS
        :return:
        """
        rospy.logdebug("Start Get Observation ==>")
​
        image_1 = self.get_camera_rgb_image_raw()
        image_2 = self.get_camera2_rgb_image_raw()
​
        discretized_observations = self.discretize_observation(
            image_1, image_2)
        # rospy.logerr("Observations==>"+str(discretized_observations))
        rospy.logdebug("END Get Observation ==>")
        # rospy.logerr(discretized_observations.shape)
        return discretized_observations
​
    def _is_done(self, observations):
​
        if self._episode_done:
            rospy.logerr("ROSBots veered OFF-COURSE ==>")
            return self._episode_done
        else:
            rospy.logerr("ROSBots is Ok ==>")
​
        return self._episode_done
​
    def _compute_reward(self, observations, done):
        """
        Our reward system will be based on how far or near the robot's forward with respect to the lane
        E.g 0(centered) = 100 and 1(off-course) = -10
        """
​
        if not done:
            if (self.action_taken != None):
                if(self.action_taken == 0):
                    reward = self.follow_lane_reward  # Favour going foward to turning left/right
                else:
                    reward = self.left_right_reward
        else:
            reward = -1*self.follow_lane_reward
​
        rospy.logdebug("reward=" + str(reward))
        self.cumulated_reward += reward
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))
​
        return reward
​
    # Internal TaskEnv Methods
​
    def discretize_observation(self, image_data,  image2_data):
        """
        Discards all the laser readings that are not multiple in index of look_ahead_distance
        value.
        """
​
        # discretized_ranges = []
​
        # 1. Process image
        # 2. Reduce the 'cur_offset' to 2dp
        # 3. Create a mapping for the ranging aspect
​
        # Results returned is a tuple (detection:boolean, angular_z:float, observation_image: numpy.array())
        self.detection_results = self.line_follow.process_data(observation_image=image_data,
                                                               detection_image=image2_data,
                                                               rows_to_watch=self.look_ahead_distance,
                                                               show_window=True)
​
        self.cur_offset = float(
            "%.4f" % self.detection_results[1])  # 0.5  and 1.52
​
        no_line_detected = self.detection_results[0]
​
        if ((self.cur_offset < self.dSafe) or (self.cur_offset > self.dMax) or no_line_detected):
            self._episode_done = True
        else:
            self._episode_done = False
​
        # return the image converted to numpy array as observation data
        return self.detection_results[2]
​
**rosbots_lane_follow_env.py**

Code Analysis
First of all, at the top section of the script, we import various classes that will be required for the functionalities we want to define for the robot in question.

Then, we register our class with Open AI Gym and set the entry point. Take note of the id used. That is how we will refer to this file later on in the training script.

register(
        id='RosBotsEnv-v0',
        entry_point='rosbots_lane_follow_env:RosBotsEnv',
        timestep_limit=timestep_limit_per_episode,
    )
You may also notice that our current class inherits from the Robot Environment class (in this case, it is called RosBotsEnv) and implements the functions defined in it. We will see this class in the next section.

class RosBotsEnv(rosbots_env.RosBotsEnv):
Let's now center our focus on these five(5) functions:

def setaction(self, action):

def getobs(self):

def isdone(self, observations):

def computereward(self, observations, done):

def discretize_observation(self, image_data):

setaction
def _set_action(self, action):
    """
    This set action will Set the linear and angular speed of the ROSBots
    based on the action number given.
    :param action: The action integer that set s what movement to do next.
    """
​
    rospy.logdebug("Start Set Action ==>"+str(action))
    self.action_taken = action
    # We convert the actions to speed movements to send to the parent class CubeSingleDiskEnv
    if action == 0: #FORWARD
        linear_speed = self.linear_forward_speed
        angular_speed = 0.0
        self.last_action = "FORWARD"
    elif action == 1: #LEFT
        linear_speed = self.linear_turn_speed
        angular_speed = self.angular_speed
        self.last_action = "TURN_LEFT"
    elif action == 2: #RIGHT
        linear_speed = self.linear_turn_speed
        angular_speed = -1*self.angular_speed
        self.last_action = "TURN_RIGHT"
​
    # We tell ROSBot the linear and angular speed to set to execute
    self.move_base( linear_speed,
                    angular_speed,
                    epsilon=0.05,
                    update_rate=10)
​
    rospy.logdebug("END Set Action ==>"+str(action))
The set action (self, action) function defines what each action will do. Each action will be represented by an integer between 0 and 2. And depending on the action number that this function receives, the ROSbots will do one thing or another.

So basically, what we are doing here is the following: based on the action chosen by the training script, we tell the robot to move FORWARD, LEFT, or RIGHT.

getobs
def _get_obs(self):
    """
    Here we define what sensor data defines our robot's observations
    To know which Variables we have access to, we need to read the
    RosBotsEnv API DOCS
    :return:
    """
    rospy.logdebug("Start Get Observation ==>")
​
    image_1 = self.get_camera_rgb_image_raw()
    image_2 = self.get_camera2_rgb_image_raw()
​
    discretized_observations = self.discretize_observation(image_1, image_2)
    #rospy.logerr("Observations==>"+str(discretized_observations))
​
    rospy.logdebug("END Get Observation ==>")
    rospy.logerr(discretized_observations.shape)
    return discretized_observations
When this function is called:

We get the current image from the stream
We pass the current image to a discretizer function. The discretizer function returns a list containing our observations.
You might be wondering, WHAT ARE WE CONSIDERING TO BE AN OBSERVATION IN THIS SCENARIO? The answer is simple: we simply want to enable the ROSBots to be able to drive on its own by just using the feed from its camera, right? In that case, we return the current offset of the lane relative to the center of the image as our observation.

Simply put: Observation = [current_lane_offset]

NOTE: offset here refers to the centroid of the image after it has been stripped of all other colors, relative to the center of the original image used.

discretize_observation
def discretize_observation(self, image_data,  image2_data):
        """
        Discards all the laser readings that are not multiple in index of look_ahead_distance
        value.
        """
​
        # discretized_ranges = []
​
        # 1. Process image
        # 2. Reduce the 'cur_offset' to 2dp
        # 3. Create a mapping for the ranging aspect
​
        # Results returned is a tuple (detection:boolean, angular_z:float, observation_image: numpy.array())
        self.detection_results = self.line_follow.process_data(observation_image=image_data,
                                                               detection_image=image2_data,
                                                               rows_to_watch=self.look_ahead_distance,
                                                               show_window=True)
​
        self.cur_offset = float(
            "%.4f" % self.detection_results[1])  # 0.5  and 1.52
​
        no_line_detected = self.detection_results[0]
​
        if ((self.cur_offset < self.dSafe) or (self.cur_offset > self.dMax) or no_line_detected):
            self._episode_done = True
        else:
            self._episode_done = False
​
        # return the image converted to numpy array as observation data
        return self.detection_results[2]
​
This is where we make a call to the image processor function, get the current_lane_offset, and scale it up using the interpolation function self.observation_mapper()

We do the scaling in order to have our observations returned to us on a scale of 0-100.

isdone
def _is_done(self, observations):
        
        if self._episode_done:
            rospy.logerr("ROSBots veered OFF-COURSE ==>")
            return self._episode_done
        else:
            rospy.logerr("ROSBots is Ok ==>")
​
        return self._episode_done
The is done (self, observations) function decides whether the current step has finished or not, and returns a variable (boolean) that says so. For this case, the is done function checks if the value of the observation is still within range in order to decide when to end an episode.

computereward
def _compute_reward(self, observations, done):
        """
        Our reward system will be based on how far or near the robot's forward is with respect to the lane
        E.g 0(centered) = 100 and 1(off-course) = -10
        """
        
        if not done:
            if (self.action_taken != None):
                if(self.action_taken == 0):
                    reward = self.follow_lane_reward #Favour going foward to turning left/right
                else:
                    reward = self.left_right_reward
        else:
            reward = -1*self.follow_lane_reward
​
​
        rospy.logdebug("reward=" + str(reward))
        self.cumulated_reward += reward
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))
        
        return reward
The compute reward (self, observations, done) function decides the reward to give for each step. In this specific case, the compute reward() function works with the following logic: Since we want the robot to drive as close to the lane as possible without going overboard, the function assigns a maximum of follow lane reward, which is set in our parameter file to when the robot is moving in the safezone (dSafe >> min_offset_boundary) and a minimum of 0 for when the robot approaches the outer edge or boundary (dMax >> max_offset_boundary)

We also penalize the robot when it goes out of range with a value of veer_off_reward set in the parameters file.

**Exercise 6.1**

a) Create a new pacakge, where you will place all the files related to the ROSbots training.

Execute in WebShell #1

catkin_create_pkg my_rosbots_training rospy openai_ros
b) Inside the my_rosbots_training package, create the following folders:

scripts
launch
config
c) Inside the scripts folder, add a new script called rosbots_lane_follow_env.py. In this new file, copy the contents of the rosbots_lane_follow_env.py script that you have just reviewed.

**End of Exercise 6.1**

Robot Environment
The robot environment will then contain all the functions associated with the specific "robot" that you want to train. This means that it will contain all the functionalities that your robot will need in order to be controlled.

Summarizing, in order to allow our ROSbots robot to be able to learn using the OpenAI Gym, we will need a file that tells openai_ros:

How our robot works
How to access our robot's sensor(s)
What topic(s) our robot responds to
Any other thing we think is necessary about the robot environment that the learning algorithm is going to use to learn.
And this file is, of course, the Robot Environment. Below you can have a look at an example Robot Environment file for our ROSbots.

**rosbots_env.py**

import numpy
import rospy
from openai_ros import robot_gazebo_env
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
​
​
class RosBotsEnv(robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all CubeSingleDisk environments.
    """
​
    def __init__(self):
        """
        Initializes a new RosBots environment.
​
        To check any topic we need to have the simulations running, we need to do two things:
        1) Unpause the simulation: without that the stream of data doesn't flow. This is for simulations
        that are paused for whatever reason
        2) If the simulation was running already for some reason, we need to reset the controllers.
        This has to do with the fact that some plugins with tf don't understand the reset of the simulation
        and need to be reset to work properly.
​
        The Sensors: The sensors accessible are the ones considered useful for AI learning.
​
        Sensor Topic List:
        * /odom : Odometry readings of the Base of the Robot
        * /camera/rgb/image_raw: RGB camera
​
        Actuators Topic List: /cmd_vel,
​
        Args:
        """
        rospy.logdebug("Start RosBotsEnv INIT...")
        # Variables that we give through the constructor.
        # None in this case
​
        # Internal Vars
        # Doesnt have any accesibles
        self.controllers_list = []
​
        # Using namespace
        self.robot_name_space = "/robot1"
​
        # Control parameters
        self.odom_topic = self.robot_name_space + "/odom"
        self.camera2_topic = self.robot_name_space + "/camera1/image_raw"
        self.camera_topic = self.robot_name_space + "/camera1/image_raw_crop"
​
        self.detection_results = None
​
        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(RosBotsEnv, self).__init__(controllers_list=self.controllers_list,
                                         robot_name_space=self.robot_name_space,
                                         reset_controls=False,
                                         start_init_physics_parameters=False,
                                         reset_world_or_sim="WORLD")
​
        self.gazebo.unpauseSim()
        # self.controllers_object.reset_controllers()
        self._check_all_sensors_ready()
​
        # We Start all the ROS related Subscribers and publishers
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback)
        rospy.Subscriber(self.camera_topic, Image,
                         self._camera_rgb_image_raw_callback)
        rospy.Subscriber(self.camera2_topic, Image,
                         self._camera2_rgb_image_raw_callback)
​
        self._cmd_vel_pub = rospy.Publisher(
            self.robot_name_space + '/cmd_vel', Twist, queue_size=1)
​
        self._check_publishers_connection()
​
        self.gazebo.pauseSim()
​
        rospy.logdebug("Finished RosBotsEnv INIT...")
​
    # Methods needed by the RobotGazeboEnv
    # ----------------------------
​
    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers, and other simulation systems are
        operational.
        """
        self._check_all_sensors_ready()
        return True
​
    # CubeSingleDiskEnv virtual methods
    # ----------------------------
​
    def _check_all_sensors_ready(self):
        rospy.logdebug("START ALL SENSORS READY")
        self._check_odom_ready()
        self._check_camera_rgb_image_raw_ready()
        self._check_camera2_rgb_image_raw_ready()
        rospy.logdebug("ALL SENSORS READY")
​
    def _check_odom_ready(self):
        self.odom = None
        rospy.logdebug("Waiting for " + self.odom_topic + " to be READY...")
        while self.odom is None and not rospy.is_shutdown():
            try:
                self.odom = rospy.wait_for_message(
                    self.odom_topic, Odometry, timeout=5.0)
                rospy.logdebug("Current " + self.odom_topic + " READY=>")
​
            except:
                rospy.logerr("Current " + self.odom_topic +
                             " not ready yet, retrying to get odom")
​
        return self.odom
​
    def _check_camera_rgb_image_raw_ready(self):
        self.camera_rgb_image_raw = None
        rospy.logdebug("Waiting for " + self.camera_topic + " to be READY...")
        while self.camera_rgb_image_raw is None and not rospy.is_shutdown():
            try:
                self.camera_rgb_image_raw = rospy.wait_for_message(
                    self.camera_topic, Image, timeout=5.0)
                rospy.logdebug("Current " + self.camera_topic + " READY=>")
​
            except:
                rospy.logerr("Current " + self.camera_topic +
                             " not ready yet, retrying for getting camera_rgb_image_raw")
        return self.camera_rgb_image_raw
​
    def _check_camera2_rgb_image_raw_ready(self):
        self.camera2_rgb_image_raw = None
        rospy.logdebug("Waiting for " + self.camera2_topic + " to be READY...")
        while self.camera2_rgb_image_raw is None and not rospy.is_shutdown():
            try:
                self.camera2_rgb_image_raw = rospy.wait_for_message(
                    self.camera2_topic, Image, timeout=5.0)
                rospy.logdebug("Current " + self.camera2_topic + " READY=>")
​
            except:
                rospy.logerr("Current " + self.camera2_topic +
                             " not ready yet, retrying for getting camera2_rgb_image_raw")
        return self.camera2_rgb_image_raw
​
    def _odom_callback(self, data):
        self.odom = data
​
    def _camera_rgb_image_raw_callback(self, data):
        self.camera_rgb_image_raw = data
​
    def _camera2_rgb_image_raw_callback(self, data):
        self.camera2_rgb_image_raw = data
​
    def _check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while self._cmd_vel_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug(
                "No susbribers to _cmd_vel_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_cmd_vel_pub Publisher Connected")
​
        rospy.logdebug("All Publishers READY")
​
    # Methods that the TrainingEnvironment will need to define here as virtual
    # because they will be used in RobotGazeboEnv GrandParentClass and defined in the
    # TrainingEnvironment.
    # ----------------------------
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        raise NotImplementedError()
​
    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()
​
    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()
​
    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()
​
    def _get_obs(self):
        raise NotImplementedError()
​
    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()
​
    # Methods that the TrainingEnvironment will need.
    # ----------------------------
    def move_base(self, linear_speed, angular_speed, epsilon=0.05, update_rate=10, use_offset=True):
        """
        It will move the base based on the linear and angular speeds given.
        It will wait until those twists are achieved reading from the odometry topic.
        :param linear_speed: Speed in the X axis of the robot base frame
        :param angular_speed: Speed of the angular turning of the robot base frame
        :param epsilon: Acceptable difference between the speed asked and the odometry readings
        :param update_rate: Rate at which we check the odometry.
        :return:
        """
        cmd_vel_value = Twist()
        cmd_vel_value.linear.x = linear_speed
        cmd_vel_value.angular.z = angular_speed
        rospy.logdebug("ROSBots Base Twist Cmd>>" + str(cmd_vel_value))
        self._check_publishers_connection()
        self._cmd_vel_pub.publish(cmd_vel_value)
​
    def get_odom(self):
        return self.odom
​
    def get_camera_rgb_image_raw(self):
        return self.camera_rgb_image_raw
​
    def get_camera2_rgb_image_raw(self):
        return self.camera2_rgb_image_raw
​
---------------------------------------------------------------------------
ModuleNotFoundError                       Traceback (most recent call last)
<ipython-input-4-0f053a26e59f> in <module>()
      1 import numpy
----> 2 import rospy
      3 from openai_ros import robot_gazebo_env
      4 from std_msgs.msg import Float64
      5 from sensor_msgs.msg import JointState

ModuleNotFoundError: No module named 'rospy'

**rosbots_env.py**

Code Analysis
First of all, you can see that this class inherits from the RobotGazeboEnv class:

class RosBotsEnv(robot_gazebo_env.RobotGazeboEnv):
This means that all the functions that are defined in the Gazebo Environment will also be accessible from this class.

In the _init_() function of the class, we are basically creating all the publishers and subscribers required for this specific ROSbots environment:

self.odom_topic = self.robot_name_space + "/odom"
self.camera2_topic = self.robot_name_space + "/camera1/image_raw"
self.camera_topic = self.robot_name_space + "/camera1/image_raw_crop"
​
rospy.Subscriber( self.odom_topic, Odometry, self._odom_callback)
rospy.Subscriber( self.camera_topic, Image, self._camera_rgb_image_raw_callback)
rospy.Subscriber( self.camera2_topic, Image, self._camera2_rgb_image_raw_callback)
​
self._cmd_vel_pub = rospy.Publisher(self.robot_name_space + '/cmd_vel', Twist, queue_size=1)
These are the publishers needed to move the ROSbots robot around, and the subscribers needed to get data about the odometry, and the RGB cameras input.

Here, we are basically initializing the Gazebo Environment, passing it some variables it needs.

super(RosBotsEnv, self).__init__(controllers_list=self.controllers_list,
                                robot_name_space=self.robot_name_space,
                                reset_controls=False,
                                start_init_physics_parameters=False,
                                reset_world_or_sim="WORLD")
Let's now center our focus on these three(3) functions:

def move_base(self, linear_speed, angular_speed, epsilon=0.05, update_rate=10, use_offset=True):

def get_camera_rgb_image_raw(self):

def get_camera2_rgb_image_raw(self):

move_base
def move_base(self, linear_speed, angular_speed, epsilon=0.05, update_rate=10, use_offset=True):
    """
    It will move the base based on the linear and angular speeds given.
    It will wait until those twists are achieved reading from the odometry topic.
    :param linear_speed: Speed on the X axis of the robot base frame
    :param angular_speed: Speed of the angular turning of the robot base frame
    :param epsilon: Acceptable difference between the speed asked and the odometry readings
    :param update_rate: Rate at which we check the odometry.
    :return: 
    """
    cmd_vel_value = Twist()
    cmd_vel_value.linear.x = linear_speed
    cmd_vel_value.angular.z = angular_speed
    rospy.logdebug("ROSBots Base Twist Cmd>>" + str(cmd_vel_value))
    self._check_publishers_connection()
    self._cmd_vel_pub.publish(cmd_vel_value)
The move_base (...) function will be in charge of moving the robot based on the linear and angular speeds given (linear_speed and angular_speed input variables). It is very important because it will be the function responsible for moving our ROSbots around during the training session.

get_camera_rgb_image_raw
def get_camera_rgb_image_raw(self):
        return self.camera_rgb_image_raw
The get_camera_rgb_image_raw (self) basically returns the RGB Image of one of the camera topics being processed. The topic it gets the RGB Image from is /rosbots/camera1/image_raw.

get_camera2_rgb_image_raw
def get_camera2_rgb_image_raw(self):
        return self.camera2_rgb_image_raw
The get_camera2_rgb_image_raw (self) function does exactly the same as the previous functions, but gets the RGB Image from a different topic. In this case, from /rosbots/camera1/image_raw_rect.

**Exercise 6.2**

a) Inside the scripts folder, add a new script called rosbots_env.py. In this new file, copy the contents of the rosbots_env.py script that you have just reviewed.

**End of Exercise 6.2**

MULTIPLE CAMERAS?
From the previous Robot Environment, you may have noticed that we are using two different image streams. But in our ROsbots, we only have one camera mounted, don't we? Then... what's going on here?

Well. Basically, and for purposes of reducing the image matrix used and speeding up the training process, we are going to use two camera inputs for this training.

Camera Input n° 1:

Image dimensions: 80 x 80
Topic: /robot1/camera1/image_raw_crop
Uses: Returned as the robot's observation
Camera Input n° 2:
Image dimensions: 320 x 240
Topic: /robot1/camera1/image_raw
Uses: By the detection algorithm, to keep the robot within bounds
That's why you saw in the Robot Environment the following two functions:

get_camera_rgb_image_raw()
get_camera2_rgb_image_raw()
Let's clarify that the Camera Input nº 2, which has image dimensions of 320 x 240, is the current image stream that we have with the camera mounted on our ROSbots. Then, where does the other camera input come from? Well, it comes from the same camera, but we are just resizing the image dimensions in order to make the training process a little bit faster.

Resizing Camera Image
In order to have an extra camera input, we are going to use the ros_imresize package, developed by Jeremie Deray.

This package takes an already existing RGB camera stream, and provides two new topics (image_raw_rect and camera_info_rect) as an output, with a resized image.

In order to have it working, let's follow the next steps:

Step 1
First, let's download the package from the official repository into our workspace:

Execute in WebShell #1

roscd; cd ../src;
git clone https://github.com/artivis/ros_imresize.git
Step 2
Second, let's modify the example launch file, imresize.launch, so that it fits our needs. Basically, we'll need to change the names of the topics and the image size we want.

From:

<launch>
​
 <arg name="width" default="640" />
 <arg name="height" default="480" />
​
 <arg name="camera_info" default="/stereo/left/camera_info" />
 <arg name="camera_topic" default="/stereo/left/image" />
​
 <arg name="undistord" default="true"/>
 
 <node name="ros_imresize" pkg="ros_imresize" type="ros_imresize" output="screen">
  <param name="camera_info" value="$(arg camera_info)" />
  <param name="topic_crop" value="$(arg camera_topic)" />
  <param name="resize_width" value="$(arg width)" />
  <param name="resize_height" value="$(arg height)" />
  <param name="undistord" value="$(arg undistord)" />
 </node>
​
</launch>
To:

<launch>
​
 <arg name="width" default="80" />
 <arg name="height" default="80" />
​
 <arg name="camera_info" default="/robot1/camera1/camera_info" />
 <arg name="camera_topic" default="/robot1/camera1/image_raw" />
​
 <arg name="undistord" default="true"/>
 
 <node name="ros_imresize" pkg="ros_imresize" type="ros_imresize" output="screen">
  <param name="camera_info" value="$(arg camera_info)" />
  <param name="topic_crop" value="$(arg camera_topic)" />
  <param name="resize_width" value="$(arg width)" />
  <param name="resize_height" value="$(arg height)" />
  <param name="undistord" value="$(arg undistord)" />
 </node>
​
</launch>
As you can see, we are changing the input topics to the ones where our RGB camera is publishing, and we are also setting the new image size to 80x80.

Step 3
Finally, you'll need to compile the package.

Execute in WebShell #1

roscd; cd ..;
catkin_make --only-pkg-with-deps ros_imresize
Step 4
So now, let's launch it!

Execute in WebShell #1

roslaunch ros_imresize imresize.launch
If everything goes fine, you should get the following two new topics when doing a rostopic list:

/robot1/camera1/camera_info_crop
/robot1/camera1/image_raw_crop
Also, if you do a rostopic echo of the camera_info_crop topic, you should see that the new size of the image is 80x80.



Training Script
Let's now talk about the Training Script. This script is the one that sets up the algorithm that you are going to use in order to make your robot learn. In this case, we are going to use Deep Q-Network (DQN). DQN is a reinforcement learning algorithm that combines Q-Learning with deep neural networks to let RL work in complex, high-dimensional environments, like robotics. The goal of DQN is to learn a policy, which tells an agent which action to take under which circumstances. But we're not going to enter into details about how DQN works now.

Next, you can see an example of a Training Script that uses DQN. Then, we'll have a look at the most important parts of it.

**start_training.py**

#!/usr/bin/env python
import rospy
import time
# Inspired by https://keon.io/deep-q-learning/
import random
import gym
import math
import numpy as np
from collections import deque
from keras.models import Sequential
from keras.layers import Dense
from keras.optimizers import Adam
​
# import our task environment
import rosbots_lane_follow_env
​
​
class DQNRobotSolver():
    def __init__(self, n_observations, n_actions, n_episodes=1000, n_win_ticks=195, min_episodes=100, max_env_steps=None, gamma=1.0, epsilon=1.0, epsilon_min=0.01, epsilon_log_decay=0.995, alpha=0.01, alpha_decay=0.01, batch_size=64, monitor=False, quiet=False):
        self.memory = deque(maxlen=100000)
        self.env = gym.make('RosBotsEnv-v0')
        if monitor:
            self.env = gym.wrappers.Monitor(
                self.env, '../data/rosbots', force=True)
​
        self.input_dim = n_observations
        self.n_actions = n_actions
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_min = epsilon_min
        self.epsilon_decay = epsilon_log_decay
        self.alpha = alpha
        self.alpha_decay = alpha_decay
        self.n_episodes = n_episodes
        self.n_win_ticks = n_win_ticks
        self.min_episodes = min_episodes
        self.batch_size = batch_size
        self.quiet = quiet
        if max_env_steps is not None:
            self.env._max_episode_steps = max_env_steps
​
        # Init model
        self.model = Sequential()
​
        self.model.add(Dense(200, input_dim=self.input_dim, activation='tanh'))
        self.model.add(Dense(48, activation='tanh'))
        self.model.add(Dense(self.n_actions, activation='linear'))
        self.model.compile(loss='mse', optimizer=Adam(
            lr=self.alpha, decay=self.alpha_decay))
​
    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))
​
    def choose_action(self, state, epsilon):
        return self.env.action_space.sample() if (np.random.random() <= epsilon) else np.argmax(self.model.predict(state))
​
    def get_epsilon(self, t):
        return max(self.epsilon_min, min(self.epsilon, 1.0 - math.log10((t + 1) * self.epsilon_decay)))
​
    def preprocess_state(self, state):
        return np.reshape(state, [1, self.input_dim])
​
    def replay(self, batch_size):
        x_batch, y_batch = [], []
        minibatch = random.sample(
            self.memory, min(len(self.memory), batch_size))
        for state, action, reward, next_state, done in minibatch:
            y_target = self.model.predict(state)
            y_target[0][action] = reward if done else reward + \
                self.gamma * np.max(self.model.predict(next_state)[0])
            x_batch.append(state[0])
            y_batch.append(y_target[0])
​
        self.model.fit(np.array(x_batch), np.array(y_batch),
                       batch_size=len(x_batch), verbose=0)
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay
​
    def run(self):
​
        rate = rospy.Rate(30)
​
        scores = deque(maxlen=100)
​
        for e in range(self.n_episodes):
​
            init_state = self.env.reset()
​
            state = self.preprocess_state(init_state)
            done = False
            i = 0
            while not done:
                # openai_ros doesnt support render for the moment
                # self.env.render()
                action = self.choose_action(state, self.get_epsilon(e))
                next_state, reward, done, _ = self.env.step(action)
                next_state = self.preprocess_state(next_state)
                self.remember(state, action, reward, next_state, done)
                state = next_state
                i += 1
​
            scores.append(i)
            mean_score = np.mean(scores)
            if mean_score >= self.n_win_ticks and e >= min_episodes:
                if not self.quiet:
                    print('Ran {} episodes. Solved after {} trials'.format(
                        e, e - min_episodes))
                return e - min_episodes
            if e % 1 == 0 and not self.quiet:
                print('[Episode {}] - Mean survival time over last {} episodes was {} ticks.'.format(
                    e, min_episodes, mean_score))
​
            self.replay(self.batch_size)
​
        if not self.quiet:
            print('Did not solve after {} episodes'.format(e))
        return e
​
​
if __name__ == '__main__':
    rospy.init_node('rosbots_n1try_algorithm',
                    anonymous=True, log_level=rospy.FATAL)
​
    n_observations = rospy.get_param('/rosbots/n_observations')
    n_actions = rospy.get_param('/rosbots/n_actions')
​
    n_episodes = rospy.get_param('/rosbots/episodes_training')
    n_win_ticks = rospy.get_param('/rosbots/n_win_ticks')
    min_episodes = rospy.get_param('/rosbots/min_episodes')
    max_env_steps = None
    gamma = rospy.get_param('/rosbots/gamma')
    epsilon = rospy.get_param('/rosbots/epsilon')
    epsilon_min = rospy.get_param('/rosbots/epsilon_min')
    epsilon_log_decay = rospy.get_param('/rosbots/epsilon_decay')
    alpha = rospy.get_param('/rosbots/alpha')
    alpha_decay = rospy.get_param('/rosbots/alpha_decay')
    batch_size = rospy.get_param('/rosbots/batch_size')
    monitor = rospy.get_param('/rosbots/monitor')
    quiet = rospy.get_param('/rosbots/quiet')
​
    agent = DQNRobotSolver(n_observations,
                           n_actions,
                           n_episodes,
                           n_win_ticks,
                           min_episodes,
                           max_env_steps,
                           gamma,
                           epsilon,
                           epsilon_min,
                           epsilon_log_decay,
                           alpha,
                           alpha_decay,
                           batch_size,
                           monitor,
                           quiet)
    agent.run()
​
**start_training.py**

Code Analysis
As you can see from the imports at the beginning of the code:

from keras.models import Sequential
from keras.layers import Dense
from keras.optimizers import Adam
We are using Keras to create a deep neural network with Tensorflow that will encode the Q-learning table. If you want to get more information about the Keras library, you can have a look at their official documentation, here: http://keras.io

Specifically, in our case, we are using a Sequential model for the neural network. We are not going to go deep into how it works, so if you are interested, you can have a look at the following link: https://keras.io/getting-started/sequential-model-guide/.

Also, note that we are importing the Task Environment here, which we defined before.

import rosbots_lane_follow_env
In the line above, we are importing the Python file that contains the Task Environment that we want to use to learn.

At the end of the file, you can see the main function.

if __name__ == '__main__':
Inside the main function, you can see the following lines:

rospy.init_node('rosbots_n1try_algorithm', anonymous=True, log_level=rospy.FATAL)
In the line above, we are initializing our ROS node. Since we'll be working with Gazebo + ROS, we need to initialize a ROS node.

n_observations = rospy.get_param('/rosbots/n_observations')
n_actions = rospy.get_param('/rosbots/n_actions')
​
n_episodes = rospy.get_param('/rosbots/episodes_training')
n_win_ticks = rospy.get_param('/rosbots/n_win_ticks')
min_episodes = rospy.get_param('/rosbots/min_episodes')
max_env_steps = None
gamma =  rospy.get_param('/rosbots/gamma')
epsilon = rospy.get_param('/rosbots/epsilon')
epsilon_min = rospy.get_param('/rosbots/epsilon_min')
epsilon_log_decay = rospy.get_param('/rosbots/epsilon_decay')
alpha = rospy.get_param('/rosbots/alpha')
alpha_decay = rospy.get_param('/rosbots/alpha_decay')
batch_size = rospy.get_param('/rosbots/batch_size')
monitor = rospy.get_param('/rosbots/monitor')
quiet = rospy.get_param('/rosbots/quiet')
Here, we are getting all the parameters that have been loaded onto the ROS Parameter Server by the parameters file. This parameter file will be loaded within the launch file, when launching this script. You can have a look at it here:

**rosbots_deepq_params.yaml**

rosbots: #namespace
​
    #qlearn parameters
​
    alpha: 0.01 # Learning Rate
    alpha_decay: 0.01
    gamma: 1.0 # future rewards value 0 none 1 a lot
    epsilon: 1.0 # exploration, 0 none 1 a lot
    epsilon_decay: 0.995 # how we reduse the exploration
    epsilon_min: 0.01 # minimum value that epsilon can have
    batch_size: 64 # maximum size of the batches sampled from memory
    episodes_training: 1000
    n_win_ticks: 250 # If the mean of rewards is bigger than this and have passed min_episodes, the task is considered finished
    min_episodes: 100
    #max_env_steps: None
    monitor: True # stores results of openai gym in a file
    quiet: False # Used to show messages on screen
​
​
    # Follow the line Task Environment Related parameters
    number_decimals_precision_obs: 4
    speed_step: 1.0 # Time to wait in the reset phases
​
    linear_forward_speed: 1.5 # Spawned for going forwards
    linear_turn_speed: 0.1 # Linear speed when turning
    angular_speed: 0.45 # Angular speed when turning Left or Right
    init_linear_forward_speed: 0.0 # Initial linear speed in which we start each episode
    init_linear_turn_speed: 0.0 # Initial angular speed in shich we start each episode
​
    n_observations: 19200 # (80, 80, 3) # Number of pixels used as observations
    n_actions: 3 # Number of actions used by algorithm and task
​
    # How many meters ahead camera-stream readings we use for detection, the smaller the less faster processing
    look_ahead_distance: 100
    cur_lane_offset: 1.51
    max_lane_offset: 1.5 # Value considered max, extreme side of lane relative to the center. Cx = 320 = width cam
    safe_lane_offset: -1.5 # Value considered nominal, around lane center. Cx = 0 => Its on the max left side
​
    follow_lane_reward: 5 # Points Given to go follow lane
    left_right_reward: 2.5 # Points given when a turn action is taken
    veer_off_reward: -10
    end_episode_points: 200 # Points given when ending an episode
**rosbots_deepq_params.yaml**

Finally, we create an instance of the DQN Robot Solver () class, loading all the parameters.

agent = DQNRobotSolver(     n_observations,
                                n_actions,
                                n_episodes,
                                n_win_ticks,
                                min_episodes,
                                max_env_steps,
                                gamma,
                                epsilon,
                                epsilon_min,
                                epsilon_log_decay,
                                alpha,
                                alpha_decay,
                                batch_size,
                                monitor,
                                quiet)
and we call the run () function of the class.

agent.run()
But what does it mean to create an instance of the DQN Robot Solver () class? Let's have a look. At the beginning of the script, you may have seen that we are defining this class with all the parameters loaded from the parameters file. Here:

class DQNRobotSolver():
    def __init__(self, n_observations, n_actions, n_episodes=1000, n_win_ticks=195, min_episodes= 100, max_env_steps=None, gamma=1.0, epsilon=1.0, epsilon_min=0.01, epsilon_log_decay=0.995, alpha=0.01, alpha_decay=0.01, batch_size=64, monitor=False, quiet=False):
Also, we are creating an environment based on our Task Environment (rosbots_lane_follow_env). Here:

self.env = gym.make('RosBotsEnv-v0')
Then, it reads all the parameters that have been previously loaded and puts them into variables of the class.

self.input_dim = n_observations
    self.n_actions = n_actions
    self.gamma = gamma
    self.epsilon = epsilon
    self.epsilon_min = epsilon_min
    self.epsilon_decay = epsilon_log_decay
    self.alpha = alpha
    self.alpha_decay = alpha_decay
    self.n_episodes = n_episodes
    self.n_win_ticks = n_win_ticks
    self.min_episodes = min_episodes
    self.batch_size = batch_size
    self.quiet = quiet
    if max_env_steps is not None: self.env._max_episode_steps = max_env_steps
Finally, here it is where the Sequential model of the Neural Network is created, with all its layers.

self.model = Sequential()
​
self.model.add(Dense(200, input_dim=self.input_dim, activation='tanh'))
self.model.add(Dense(48, activation='tanh'))
self.model.add(Dense(self.n_actions, activation='linear'))
self.model.compile(loss='mse', optimizer=Adam(lr=self.alpha, decay=self.alpha_decay))
Putting it simply, the Sequential model we are using is given three(3) layers:

An Input layer is the dimension of the observation vector (19200).
The First hidden layer has 200 neurons.
The Second hidden layer has 48 neurons.
The Output layer has three neurons, which encode the action to take.
So now, in the self.model variable, we will have the neural network that is going to be trained.

We can also have a look at the run () function:

def run(self):
Inside this function, we are going to basically create a loop for the episodes specified.

for e in range(self.n_episodes):
Also, inside each episode, there is another loop for each one of the steps:

while not done:
        # openai_ros doesnt support render for the moment
        #self.env.render()
        action = self.choose_action(state, self.get_epsilon(e))
        next_state, reward, done, _ = self.env.step(action)
        next_state = self.preprocess_state(next_state)
        self.remember(state, action, reward, next_state, done)
        state = next_state
        i += 1
Each step will finish whenever the variable done is set to True. Remember that this done variable is controlled within the Task Environment we defined previously. You can go have a look at it to refresh your memory.

The rest of the code of this function is the basic structure of any reinforcement learning algorithm, with some particularities:

First, we choose an action to be performed.
action = self.choose_action(state, self.get_epsilon(e))
Then, we pass this action as a step to our Task Environment, and we get a reward based on the step executed.
next_state, reward, done, _ = self.env.step(action)
Finally, we store all the data of the episode in a queue.
self.remember(state, action, reward, next_state, done)
At the end of each episode, this stored data will be used by the neural network to learn.

self.replay(self.batch_size)
And that's basically it. We could go into many more details about the code or about how the algorithm works, but that's not the real concern of this course.

The MOST IMPORTANT THING you need to understand from the training script is that it is totally independent from the environments. The purpose of this training script is to set up the learning algorithm that you want to use in order to make your agent learn, regardless of what is being done in the environments. This means that you can change the algorithm you use to learn in the training script without having to worry about modifying your environment's structure. And this is very powerful!

Let's now test everything within the following exercise.

**Exercise 6.3**

b) Inside the my_rosbots_training package, add the Training Script you have just reviewed inside the scripts folder. Also, inside the config folder, add the Parameters File. Finally, inside the launch folder, create a new file named start_training.launch that will start the Training.

You can have a look at the below launch file:

**start_training.launch**

<launch>
    <rosparam command="load" file="$(find my_rosbots_training)/config/rosbots_deepq_params.yaml" />
    <!-- Launch the training system -->
    <node pkg="my_rosbots_training" name="rosbots_n1try_algorithm" type="start_training.py" output="screen"/> -->
</launch>
**start_training.launch**

As you can see, the launch file is quite self-explanatory. What you are doing is, first, loading some parameters that Qlearn needs in order to train, and second, launching the training script that you've reviewed in the above section.

b) Launch your Training script and check how your robot starts learning how to follow lanes.

Execute in WebShell #1

roslaunch my_rosbots_training start_training.launch
**End of Exercise 6.3**

Visualizing the reward
When working with the OpenAI ROS structure, an important thing to know is that it publishes the rewards of the learning process into a topic named /openai/reward. This is very important because it will allow us to visualize our reward using regular ROS tools. For instance, rqt_multiplot.

rqt_multiplot is similar to rqt_plot, since it also provides a GUI for visualizing 2D plots, but it's a little bit more complex and provides more features. For instance, you can visualize multiple plots at the same time, or you can also customize the axis for your plot.

In the next exercise, we are going to see how to visualize our training reward using the rqt_multiplot tool.

**Exercise 6.4**


a) First of all, let's start the rqt_multiplot tool.

Execute in WebShell #1

rosrun rqt_multiplot rqt_multiplot
b) Hit the icon with a screen in the top-right corner of the IDE windowin order to open the Graphic Interface window.

You should now see something like this in this new window.



c) Next, let's launch our training script so that we start getting the rewards published into the /openai/reward topic.

Execute in WebShell #1

roslaunch my_rosbots_training start_training.launch
d) Next, let's configure our plot. For that, first, you'll need to click on the Configure plot button, at the top-right side of the window.



e) Give a name to your new plot, and click on the Add curve icon.



f) Next, set the topic to /openai/reward. Automatically, the rest of the field will be autocompleted. For the x-axis, select the episode_number field. For the y-axis, select the episode_reward field, like in the picture below.



When you are done, just click on the Enter key on your keyboard.

g) Finally, in the plot visualization window, you will need to click on the Run plot button in order to start visualizing your new plot.



If everything went OK, you should visualize something like this:



You can also save your plot configuration, if you want. This way, you will be able to load it at any time later on. Remember to save it into your catkin_ws/src workspace so that it will also be saved when you close the course.



**End of Exercise 6.4**

