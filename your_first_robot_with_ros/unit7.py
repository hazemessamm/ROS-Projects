
Building and Rosifying a Robot from Scratch


Unit 5: Navigating with RIAbot (ORB-SLAM2)
SUMMARY

Estimated time to completion: 2 hours

In this unit, you are going to see how to use the ORB-SLAM2 approach, which will allow you to perform SLAM (Simultaneous Localization and Mapping) on your robot by just using an RGB camera.

END OF SUMMARY

In Part 1 of this unit, you saw how to create algorithms in order to perform lane following. This is quite useful in and of itself, but it's also limited. One of the main issues when talking about Navigation is solving the SLAM (Simultaneous Localization and Mapping) issue.

There are many packages and algorithms in the ROS ecosystem that already perform SLAM, which work very well (You can have a look at the ROS Navigation in 5 Days course of our Academy, for instance). But the problem is that almost all of these already existing packages require either a Laser or an RGB-D camera mounted on our robot. And our robot has none of them. In our case, we only have an RGB camera. So... what can we do?

ORB-SLAM2
ORB-SLAM2 is a real-time SLAM library for Monocular, Stereo, and RGB-D cameras. It works by computing the camera trajectory and then creating a sparse 3D reconstruction (in the case of Stereo and RGB-D cameras, with true scale). It is also able to detect loops and relocalize the camera in real time.

In our case, though, we cannot use the Stereo or RGB-D nodes, since we would need different cameras for that. In our robot, we have a regular Monocular camera, so we'll need to use the Monocular node.

Before we start working with it, though, let's remember that the project was developed by Raul Mur-Artal, Juan D. Tardos, J. M. M. Montiel, and Dorian Galvez-Lopez. You can check out their official repository here: https://github.com/raulmur/ORB_SLAM2

Or read some interesting articles related to ORB-SLAM (Monocular): http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf

OK! Now, let's see an example of how it works within the following exercise.

**Exercise 4.2**

a) Launch the ORB_SLAM2 node by issuing the commands below. First let's go to the directory where we have the Vocabulary and Examples folders.

Execute in WebShell #1

cd /home/simulations/public_sim_ws/src/all/orb_slam2_tc/ORB_SLAM2
Then, let's run the command to start the ORB_SLAM2 node:

Execute in WebShell #1

rosrun orb_slam2 Mono Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml
b) Hit the icon with a screen in the top-right corner of the IDE windowin order to open the Graphic Interface window.

You should now see the following displays in the new window:



In the beginning, you might see a message like this one instead:



Just wait a few seconds until the map correctly initializes. In some cases, it may even take up to some minutes to correctly initialize, so just be patient!

c) Move the robot around and see how the map gets built. Remember, in order to move the robot around, you can use the following command:

Execute in WebShell #1

rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/part2_cmr/cmd_vel
NOTE: While creating the map, try to avoid sharp movements, since the camera can easily get off track. If that is the case, you might see a message like the following:



NOTE: When you are done creating your map, DO NOT CLOSE the program. You will need to have it running for the next exercise.

**END Exercise 4.2**

Awesome, right? But let's now have a look at how this works. Below you can have a look at a graph that shows how the algorithm works:



So basically, the algorithm works on three threads: a TRACKING thread, a LOCAL MAPPING thread, and a LOOP CLOSING thread.

Tracking: The tracking part basically localizes and tracks the camera, and also decides when to insert a new keyframe.
Local Mapping: The mapping part is the one in charge of generating the map.
Loop Closing: The loop closing part checks for loops (the robot goes through the same area twice) in the map in order to clean the possible errors in the graph.
A quick look at the code
For this course, we have already compiled and built the code for you. So, in this case, you won't be able to modify it or re-compile it. So, let's have a look at the most important part of the code.

As you already know, for our case we are using the Monocular version of ORB-SLAM2, since our camera is Monocular. Because of this, the code that we are using is ros_mono.cc. You can have a look at this code below:

**ros_mono.cc**

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
​
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
​
#include<opencv2/core/core.hpp>
​
#include"../../../include/System.h"
​
using namespace std;
​
class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}
​
    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
​
    ORB_SLAM2::System* mpSLAM;
};
​
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();
​
    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    
​
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
​
    ImageGrabber igb(&SLAM);
​
    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/robot1/camera1/image_raw", 1, &ImageGrabber::GrabImage,&igb);
​
    ros::spin();
​
    // Stop all threads
    SLAM.Shutdown();
​
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
​
    ros::shutdown();
​
    return 0;
}
​
void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
​
    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
}
**ros_mono.cc**

Within this file, as you can see, you can set up two important things. First, the topic from which the node will get the Camera Image stream. In this case, we've set it to /robot1/camera1/image_raw.

ros::Subscriber sub = nodeHandler.subscribe("/robot1/camera1/image_raw", 1, &ImageGrabber::GrabImage,&igb);
Also, you can set the file name where the camera trajectory will be saved. Here, it is KeyFrameTrajectory.txt.

SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
Another file worth look at is the Settings File. In the previous exercise, when we executed the command:

rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml
We were specifying that we want to use the Settings File located at Examples/Monocular/TUM1.yaml. Let's have a look at this file:

**TUM1.yaml**

%YAML:1.0
​
#--------------------------------------------------------------------------------------------
# Camera Parameters. Adjust them!
#--------------------------------------------------------------------------------------------
​
# Camera calibration and distortion parameters (OpenCV) 
Camera.fx: 517.306408
Camera.fy: 516.469215
Camera.cx: 318.643040
Camera.cy: 255.313989
​
Camera.k1: 0.262383
Camera.k2: -0.953104
Camera.p1: -0.005358
Camera.p2: 0.002628
Camera.k3: 1.163314
​
# Camera frames per second 
Camera.fps: 30.0
​
# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
Camera.RGB: 1
​
#--------------------------------------------------------------------------------------------
# ORB Parameters
#--------------------------------------------------------------------------------------------
​
# ORB Extractor: Number of features per image
ORBextractor.nFeatures: 3000
​
# ORB Extractor: Scale factor between levels in the scale pyramid   
ORBextractor.scaleFactor: 1.2
​
# ORB Extractor: Number of levels in the scale pyramid  
ORBextractor.nLevels: 8
​
# ORB Extractor: Fast threshold
# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
# First, we impose iniThFAST. If no corners are detected we impose a lower value minThFAST
# You can lower these values if your images have low contrast           
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
​
#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1
Viewer.GraphLineWidth: 0.9
Viewer.PointSize:2
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3
Viewer.ViewpointX: 0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500
**TUM1.yaml**

As you can see, within this file, we can set up parameters related to the Camera calibration, but also to the ORB SLAM node.

Some Camera parameters:

fx, fy, cx, and cy as the K matrix
k1, k2, p1 ,p2 and k3 as the D matrix.

fps: frames per second. By default, it's 30

rgb: color order. 1 for RGB, and 0 for RBG.
Some ORB parameters:

nFeatures: Number of features per image
scaleFactor: Scale factor between levels in the scale pyramid
nLevels : Number of levels in the scale pyramid
SLAM and Localization Modes
The SLAM Mode is the default mode. In this mode, the system runs in three parallel threads: Tracking, Local Mapping, and Loop Closing, as you saw before. The system localizes the camera, builds a new map, and tries to close loops. This is the one you used in the previous exercise. This mode is used when you want to create a map of your environment.

On the other side, the Localization Mode can be used when you already have a good map of your working area. In this mode, the Local Mapping and Loop Closing are deactivated. In this case, the system simply localizes the camera in the map, which will no longer be updated.

Let's test this mode in the following exercise!

NOTE: Take into account that, in order to successfully run the following exercise, you will first need to have a map already created from your environment.

**Exercise 4.3**

c) In order to activate the Localization Mode, you just have to click on the specific button in the menu on the left. Check the image below:



**END Exercise 4.3**

