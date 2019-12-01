
Building and Rosifying a Robot from Scratch


Unit 5: Navigating with RIAbot (Line following)
SUMMARY

Estimated time to completion: 2 hours

In this unit, you are going to see how to create a simulation of your ROSbots robot so that you can test everything you want in the simulated environment, without worrying about damaging anything in the real robot.

END OF SUMMARY

In this unit, you will learn how to start using the most basic and also the most powerful tool for perception in ROS: OpenCV. OpneCV is the most extensive and complete library for image recognition. With it, you will be able to work with images like never before: applying filters, postprocessing, and working with images in any way you want.

Line following
As you might have guessed, OpenCV is not a ROS library, but it's been integrated nicely into it with OpenCV_bridge. This package allows the ROS imaging topics to use the OpenCV image variable format.

For example, OpenCV images come in BGR image format, while regular ROS images are in the more standard RGB encoding. OpenCV_bridge provides a nice feature to convert between them. Also, there are many other functions to transfer images to OpenCV variables transparently.

To learn how to use OpenCV, you will use the RGB camera from our RIAbot:



Note that our robot will operate in the city environment, which has yellow lines all across the road.



What you will have to do is make the robot move in this environment, following the yellow line. For that, we will divide the work into the following phases:

Get images from the ROS topic and convert them into OpenCV format
Process the images using OpenCV libraries to obtain the data we want for the task
Move the robot along the yellow line, based on the data obtained
1· Get Images from a ROS topic and show them with OpenCV
Before doing anything, create a new package called my_following_line_package, with dependency in rospy. Also, create two new folders inside this package: one named launch and the other one scripts.

The first step will be to get the images from a ROS topic, convert them into the OpenCV format, and visualize them in the Graphical Interface window.

Here you have an example of how to do it:

**line_follower_basics.py**

#!/usr/bin/env python
​
import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
​
​
class LineFollower(object):
​
    def __init__(self):
    
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/robot1/camera1/image_raw",Image,self.camera_callback)
​
    def camera_callback(self,data):
        
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
​
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)
​
​
def main():
    line_follower_object = LineFollower()
    rospy.init_node('line_following_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
​
if __name__ == '__main__':
    main()
**END line_follower_basics.py**

Here there are several elements to comment on:

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
These imports are the basics necessary to work with images in ROS.
You have OpenCV2 library (cv2). Why the 2? Because there is already a version 3.
You have the numpy library, which makes the matrix and other operations easy to work with, and CV_Bridge, which allows ROS to work with OpenCV easily.

self.image_sub = rospy.Subscriber("/robot1/camera1/image_raw",Image,self.camera_callback)
The subscriber to the image topic. This topic publishes information of the sensor_msgs/Image type. Execute the following command to see what are the different variables inside this message type:

Execute in WebShell #1

rosmsg show  sensor_msgs/Image
std_msgs/Header header                                           
  uint32 seq                                           
  time stamp                                           
  string frame_id                                          
uint32 height                                          
uint32 width                                           
string encoding                                          
uint8 is_bigendian                                           
uint32 step                                          
uint8[] data
You can extract data from certain variables by doing the following:

Execute in WebShell #1

rostopic echo -n1 /robot1/camera1/image_raw/height
Execute in WebShell #1

rostopic echo -n1 /robot1/camera1/image_raw/width 
Execute in WebShell #1

rostopic echo -n1 /robot1/camera1/image_raw/encoding
Execute in WebShell #1

rostopic echo -n1 /robot1/camera1/image_raw/data
It should give you something like this:

user ~ $ rostopic echo -n1 /robot1/camera1/image_raw/height                                                                                                      
240                                   
---                                                                                                                                                          
user ~ $ rostopic echo -n1 /robot1/camera1/image_raw/width                                                                                                       
320                                                                                                                                                          
---                                                                                                                                                          
user ~ $ rostopic echo -n1 /robot1/camera1/image_raw/encoding                                                                                                    
rgb8                                                                                                                                                         
---  
user ~ $ rostopic echo -n1 /robot1/camera1/image_raw/data 
[129, 104, 104, 129, 104,
...
129, 104, 104, 129, 104]
The most important information here is:

height and width: These are the dimensions in camera pixels. In this case, it's 240 by 320.
encoding: How these pixels are encoded. This means what each value in the data array will mean. In this case, it's rgb8. This means that the values in data will be a color value represented as red/green/blue in 8-bit integers.
data: The Image data.
If you want the full documentation of this class, please refer to: http://docs.ros.org/api/sensor_msgs/html/msg/Image.html

Thanks to the cv_bridge package, we can easily convert the image data contained in an ImageSensorData into a format that OpenCV understands. By converting it into OpenCV, we can use all the power of that library to process the images of the robot.

try:
    # We select bgr8 because its the OpenCV encoding by default
    cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
except CvBridgeError as e:
    print(e)
Retrieve the image from a ROS topic and store it in an OpenCV variable. The var data is the one that contains a ROS message with the image captured by the camera.

cv2.imshow("Image window", cv_image)
cv2.waitKey(1)
This will open a gui, where you can see the contents of the cv_image variable. This is essential to seeing the effects of the different filters and cropping of the image afterwards.

cv2.destroyAllWindows()
This command will close all the image windows when the program is terminated.

When executing this program, you will have to see the image by clicking on the Graphical Interface icon:



This program will give you an image similar to this one:



**Exercise 5.1**

a) Create a new package named my_following_line_package. Inside this package, create a scripts folder, and place the line_follower_basics.py file in it.

b) Launch the code using the command below, and test that it actually works.

Execute in WebShell #1

rosrun my_following_line_package line_follower_basics.py
NOTE: If you see the error message libdc1394 error: Failed to initialize libdc1394, DO NOT WORRY. It has NO EFFECT at all.

**END Exercise 5.1**

2· Apply Filters To the Image
The raw image is useless unless you filter it to only see the color you want to track, and you crop the parts of the image you are not interested in. This is to make your program faster.

We also need to extract some data from the images in a way that we can move the robot to follow the line.

First step: Get Image Info and Crop the image
Before you start using the images for detecting things, you must take into account two things:

One of the most basic pieces of data that you need to work with images are the dimensions. Is it 800x600? 1200x1024? 60x60?
This is crucial to positioning the elements detected in the image.
And second is cropping the image. It's very important to work as soon as possible with the minimum size of the image required for the task. This makes the detecting system much faster.
# We get image dimensions and crop the parts of the image we don't need
# Bear in mind that because its image matrix first value is start and second value is down limit.
# Select the limits so that they get the line not too close, not too far, and the minimum portion possible
# To make the process faster.
height, width, channels = cv_image.shape
descentre = 160
rows_to_watch = 20
crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]
So why these values and not other values? Well, it depends on the task. In this case, you are interested in lines that aren't too far away from the robot, nor too near. If you concentrate on lines too far away, it won't follow the lines, but will instead just go across the map. On the other hand, concentrating on lines that are too close won't give the robot time to adapt to changes in the line.

It's also vital to optimize the region of the image as a result of cropping. If it's too big, too much data will be processed, making your program too slow. On the other hand, it has to have enough image to work with. In the end, you will have to adapt it to each situation.

Do Exercise U2.3 to test the effects of different values.

Second step: Convert from BGR to HSV
Remember that OpenCV works with BGR instead of RGB, for historical reasons (some cameras, in the days when OpenCV was created, worked with BGR).
Well, it seems that it is not very easy to work with either RGB or BGR, when it comes to differentiating colors. That's why HSV is used. The idea behind HSV is to remove the component of color saturation. This way, it's easier to recognize the same color in different light conditions, which is a serious issue in image recognition. More information: https://en.wikipedia.org/wiki/HSL_and_HSV



By Hcl-hcv_models.svg: Jacob Rus HSV_color_solid_cone.png: SharkD derivative work: SharkD  Talk  - Hcl-hcv_models.svg HSV_color_solid_cone.png, CC BY-SA 3.0, Link**

# Convert from RGB to HSV
hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV).astype(np.float)
​
# Define the Yellow Colour in HSV
#RGB
#[[[222,255,0]]]
#BGR
#[[[0,255,222]]]
"""
To know which color to track in HSV, Put in BGR. Use ColorZilla to get the color registered by the camera
>>> yellow = np.uint8([[[B,G,R ]]])
>>> hsv_yellow = cv2.cvtColor(yellow,cv2.COLOR_BGR2HSV)
>>> print( hsv_yellow )
[[[ 34 255 255]]
"""
lower_yellow = np.array([20,100,100])
upper_yellow = np.array([50,255,255])
So, in this piece of code, you are converting the cropped_image (crop_img) into HSV.
Then, you select which color in HSV you want to track, selecting a single point from the base of the color cone. Because HSV values are quite difficult to generate, it's better that you use a color picker tool like ColorZilla to pick the RGB coding of the color to track. In this case, it's the yellow line in the simulation.

Once you have it, use the example code given in a Python terminal or create a tiny program that uses numpy as np and cv2 to convert it to HSV.
In the example given, the color of the line is HSV = [[[ 34 255 255]].

Finally, you have to select an upper and lower bound to define the region of the cone base that you will consider as Yellow. The bigger the region is, the more gradients of your picked color will be accepted. This will depend on how your robot detects the color variations in the image and how critical is mixing similar colors.



Third step: Apply the mask
Now, you need to generate a version of the cropped image in which you only see two colors: black and white. The white will be all the colors you consider yellow and the rest will be black. It's a binary image.

Why do you need to do this? It basically has two functions:

In doing this, you don't have continuous detection. It is the color or it's NOT, there is no in-between. This is vital for the centroid calculation that will be done after, because it only works on the principal of YES or NO.
Second, it will allow the generation of the result image afterwards, in which you extract everything on the image except the color line, seeing only what you are interested in seeing.
# Threshold the HSV image to get only yellow colors
mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
# Bitwise-AND mask and original image
res = cv2.bitwise_and(crop_img,crop_img, mask= mask)
You, then, merge the cropped, colored image in HSV with the binary mask image, to color only the detections, leaving the rest in black.



Fourth step: Get The Centroids, draw a circle where the centroid is, and show all the images
Centroids, in essence, represent points in space where mass concentrates; the center of mass. Centroid and centers of mass are the same thing as far as this course is concerned. And they are calculated using Integrals.

This is extrapolated into images. But, instead of having mass, we have color. The place where there is more of the color that you are looking for is where the centroid will be. It's the center of mass of the blobs seen in an image.

That's why you applied the mask to make the image binary. This way, you can easily calculate where the center of mass is located. This is because it's a discrete function, not a continuous one. This means that it allows us to integrate in a discrete fashion, and not need a function describing the fluctuations in quantity of color throughout the region.

These centroids are vital in blob tracking because they give you a precise point in space where the blob is. You will use this to follow the blob and, therefore, follow the line.

This is needed to calculate the centroids of the color blobs. You use the image moments for this:

# Calculate centroid of the blob of binary image using ImageMoments
m = cv2.moments(mask, False)
try:
    cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
except ZeroDivisionError:
    cy, cx = height/2, width/2
As you can see here, you will obtain the coordinates of the cropped image where detection of the centroid of the positive yellow color occur. If nothing is detected, it will be positioned in the center of the image.

Keep in mind that you have to assign the correct Cy, Cx values. Don't get the height and width mixed up, or you will have problems in the following exercises.

If you want a more detailed explanation of an OpenCV exercise and all that can be obtained with contour features, you can have a look at the following link: http://docs.opencv.org/trunk/dd/d49/tutorial_py_contour_features.html

If you are interested in all the mathematical justifications, check this other link: https://en.wikipedia.org/wiki/Image_moment

# Draw the centroid in the resultut image
# cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
cv2.circle(res,(int(cx), int(cy)), 10,(0,0,255),-1)
​
cv2.imshow("Original", cv_image)
cv2.imshow("HSV", hsv)
cv2.imshow("MASK", mask)
cv2.imshow("RES", res)
​
cv2.waitKey(1)
OpenCV allows you to draw a lot of things over the images, not just geometric shapes. But in this case, a circle will suffice.

cv2.circle(res,(centre_cicle_x, centre_cicle_y), LineWidth,(BGRColour of line),TypeOfLine)
We are using this feature to draw a circle in the location of the calculated centroid.



More Info: http://docs.opencv.org/2.4/modules/core/doc/drawing_functions.html

And then, finally, you show all of the image variables with their titles:



3· Move the ROSbots based on the position of the Centroid
error_x = cx - width / 2;
angular_z = -error_x / 100;
rospy.loginfo("ANGULAR VALUE SENT===>"+str(angular_z))
twist_object = Twist();
twist_object.linear.x = 0.2;
twist_object.angular.z = -error_x / 100;
# Make it start turning
self.moverosbots_object.move_robot(twist_object)
This control is passed on a Proportional control. This means that it will oscillate a lot and probably have an error. But, it is the simplest way of moving the robot and gets the job done.
It always gives a constant linear movement and the angular Z velocity depends on the difference between the centroid center in X and the center of the image.

To move the robot, you can use this module:

**move_robot.py**

#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
​
​
class MoveRosBots(object):
​
    def __init__(self):
    
        self.cmd_vel_pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)
        self.last_cmdvel_command = Twist()
        self._cmdvel_pub_rate = rospy.Rate(10)
        self.shutdown_detected = False
​
    def move_robot(self, twist_object):
        self.cmd_vel_pub.publish(twist_object)
                                    
    def clean_class(self):
        # Stop Robot
        twist_object = Twist()
        twist_object.angular.z = 0.0
        self.move_robot(twist_object)
        self.shutdown_detected = True
​
def main():
    rospy.init_node('move_robot_node', anonymous=True)
    
    
    moverosbots_object = MoveRosBots()
    twist_object = Twist()
    # Make it start turning
    twist_object.angular.z = 0.5
    
    
    rate = rospy.Rate(5)
    
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        moverosbots_object.clean_class()
        rospy.loginfo("shutdown time!")
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        moverosbots_object.move_robot(twist_object)
        rate.sleep()
​
    
if __name__ == '__main__':
    main()
**END move_robot.py**

Here you have an example of how all of this code would be put together:

**follow_line_step_hsv.py**

#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from move_robot import MoveRosBots
from geometry_msgs.msg import Twist
​
class LineFollower(object):
​
    def __init__(self):
    
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/robot1/camera1/image_raw",Image,self.camera_callback)
        self.moverosbots_object = MoveRosBots()
​
    def camera_callback(self,data):
        
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            
        # We get image dimensions and crop the parts of the image we don't need
        # Bear in mind that because the first value of the image matrix is start and second value is down limit.
        # Select the limits so that it gets the line not too close and not too far, and the minimum portion possible
        # To make process faster.
        height, width, channels = cv_image.shape
        rows_to_watch = 100
        top_trunc = 1*height / 2 #get 3/4 of the height from the top section of the image
        bot_trunc = top_trunc + rows_to_watch #next set of rows to be used
        crop_img = cv_image[top_trunc:bot_trunc, 0:width]
        
        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV).astype(np.float)
        
        # Define the Yellow Colour in HSV
        #RGB
        #[[[222,255,0]]]
        #BGR
        #[[[0,255,222]]]
        """
        To know which color to track in HSV, put in BGR. Use ColorZilla to get the color registered by the camera
        >>> yellow = np.uint8([[[B,G,R ]]])
        >>> hsv_yellow = cv2.cvtColor(yellow,cv2.COLOR_BGR2HSV)
        >>> print( hsv_yellow )
        [[[ 34 255 255]]
        """
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([50,255,255])
​
        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2
        
        
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(crop_img,crop_img, mask= mask)
        
        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        cv2.circle(res,(int(cx), int(cy)), 10,(0,0,255),-1)
​
        cv2.imshow("Original", cv_image)
        cv2.imshow("HSV", hsv)
        cv2.imshow("MASK", mask)
        cv2.imshow("RES", res)
        
        cv2.waitKey(1)
        
        
        error_x = cx - width / 2;
        angular_z = -error_x / 100;
        rospy.loginfo("ANGULAR VALUE SENT===>"+str(angular_z))
        twist_object = Twist();
        twist_object.linear.x = 0.2;
        twist_object.angular.z = -error_x / 100;
        # Make it start turning
        self.moverosbots_object.move_robot(twist_object)
        
    def clean_up(self):
        self.moverosbots_object.clean_class()
        cv2.destroyAllWindows()
        
        
​
def main():
    rospy.init_node('line_following_node', anonymous=True)
    
    
    line_follower_object = LineFollower()
   
    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        line_follower_object.clean_up()
        rospy.loginfo("shutdown time!")
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        rate.sleep()
​
    
    
if __name__ == '__main__':
    main()
**END follow_line_step_hsv.py**

**Exercise 5.2**

Add the two Python scripts provided above to the scripts directory of your package: move_robot.py and follow_line_step_hsv.py. Execute it and see how it performs.

Execute in WebShell #1

rosrun my_following_line_package follow_line_step_hsv.py
NOTE: If you see the error message libdc1394 error: Failed to initialize libdc1394, DO NOT WORRY. It has NO EFFECT at all.

Try some improvements:

Lower the speed of the robot to see if it works better. Change the linear and the angular speeds.
Change the behavior of the robot, maybe create a recovery behavior for when it loses the line.
**END Exercise 5.2**

