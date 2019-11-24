Part 2: How to give a Service
Until now, you have just been calling services that other nodes provided. But now, you are going to create your own service!

**Example 3.7**

Execute the following Python code simple_service_server.py by clicking on it and then clicking on the play button on the top right-hand corner of the IPython notebook.




You can also press **[CTRL]+[Enter]** to execute it.
**Python Program {3.7}: simple_service_server.py**

#! /usr/bin/env python
​
import rospy
from std_srvs.srv import Empty, EmptyResponse # you import the service message python classes generated from Empty.srv.
​
​
def my_callback(request):
    print "My_callback has been called"
    return EmptyResponse() # the service Response class, in this case EmptyResponse
    #return MyServiceResponse(len(request.words.split())) 
​
rospy.init_node('service_server') 
my_service = rospy.Service('/my_service', Empty , my_callback) # create the Service called my_service with the defined callback
rospy.spin() # maintain the service open.
**END Python Program {3.7}: simple_service_server.py**

Did something happen?

Of course not! At the moment, you have just created and started the Service Server. So basically, you have made this service available for anyone to call it.

This means that if you do a rosservice list, you will be able to visualize this service on the list of available services.

Execute in WebShell #1

rosservice list
On the list of all available services, you should see the /my_service service.

/base_controller/command_select
/bb8/camera1/image_raw/compressed/set_parameters
/bb8/camera1/image_raw/compressedDepth/set_parameters
/bb8/camera1/image_raw/theora/set_parameters
...
/my_service
...
Now, you have to actually CALL it. So, call the /my_service service manually. Remember the calling structure discussed in the previous chapter and don't forget to TAB-TAB to autocomplete the structure of the Service message.

Execute in WebShell #1


rosservice call /my_service [TAB]+[TAB]
Did it work? You should've seen the message 'My callback has been called' printed at the output of the cell with the Python code. Great!



INFO: Note that, in the example, there is a commented line in the my_callback function. This gives you an example of how you would access the request given by the caller of your service. It's always request.variables_in_the_request_part_of_srv_message.

So, for instance, let's do a flashback to the previous chapter. Do you remember Example 3.5, where you had to perform calls to a service in order to delete an object in the simulation? Well, for that case, you were passing the name of the object to delete to the Service Server in a variable called model_name. So, if you wanted to access the value of that model_name variable in the Service Server, you would have to do it like this:

request.model_name
Quite simple, right?

That commented line also shows you what type of object you should return. Normally, the Response Python class is used. It always has the structure name_of_the_messageResponse(). That's why for the example code shown above, since it uses the Empty service message, the type of object that returns is EmptyResponse(). But, if your service uses another type of message, let's say one that is called MyServiceMessage, then the type of object that you would return would be MyServiceMessageResponse().

END **Example 3.7**

**Exercise 3.2**

The objective of Exercise 3.2 is to create a service that, when called, will make BB8 robot move in a circle-like trajectory.
You can work on a new package or use the one you created for Exercise 3.1, called unit_3_services.
Create a Service Server that accepts an Empty service message and activates the circle movement. This service could be called /move_bb8_in_circle.

You will place the necessary code into a new Python file named bb8_move_in_circle_service_server.py. You can use the Python file simple_service_server.py as an example.
Create a launch file called start_bb8_move_in_circle_service_server.launch. Inside it, you have to start a node that launches the bb8_move_in_circle_service_server.py file.
Launch start_bb8_move_in_circle_service_server.launch and check that, when called through the WebShell, BB-8 moves in a circle.
Now, create a new Python file called bb8_move_in_circle_service_client.py that calls the service /move_bb8_in_circle. Remember how it was done in the previous chapter: Services Part 1.

Then, generate a new launch file called call_bb8_move_in_circle_service_server.launch that executes the code in the bb8_move_in_circle_service_client.py file.
Finally, when you launch this call_bb8_move_in_circle_service_server.launch file, BB-8 should move in a circle.
END **Exercise 3.2**

Solution Exercise 3.2

Please try to do it by yourself unless you get stuck or need some inspiration. You will learn much more if you fight for each exercise.



Follow this link to open the solutions notebook for Unit3 Services Part2: solutions_unit3_services_part2

END Solution Exercise 3.2

How to create your own service message
So, what if none of the service messages that are available in ROS fit your needs? Then, you create your own message, as you did with the Topic messages.

In order to create a service message, you will have to follow the next steps:

**Example 3.8**

1) Create a package like this:

Execute in WebShell #1


roscd;cd ..;cd src
catkin_create_pkg my_custom_srv_msg_pkg rospy
2) Create your own Service message with the following structure. You can put as many variables as you need, of any type supported by ROS: ROS Message Types. Create a srv folder inside your package , as you did with the topics msg folder. Then, inside this srv folder, create a file called MyCustomServiceMessage.srv. You can create with the IDE or the WebShell, as you wish.

Execute in WebShell #1


roscd my_custom_srv_msg_pkg/
mkdir srv
vim srv/MyCustomServiceMessage.srv
You can also create the MyCustomServiceMessage.srv through the IDE, if you don't feel confortable with vim.

The MyCustomServiceMessage.srv could be something like this:

int32 duration    # The time (in seconds) during which BB-8 will keep moving in circles
---
bool success      # Did it achieve it?
How to Prepare CMakeLists.txt and package.xml for Custom Service Compilation
You have to edit two files in the package similarly to how we explained for Topics:

CMakeLists.txt
package.xml
Modification of CMakeLists.txt
You will have to edit four functions inside CMakeLists.txt:

find_package()
add_service_files()
generate_messages()
catkin_package()
I. find_package()
All the packages needed to COMPILE the messages of topics, services, and actions go here. It's only getting its paths, and not really importing them to be used in the compilation.
The same packages you write here will go in package.xml, stating them as build_depend.

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)
II. add_service_files()
This function contains a list of all of the service messages defined in this package (defined in the srv folder).
For our example:

add_service_files(
  FILES
  MyCustomServiceMessage.srv
)
III. generate_messages()
Here is where the packages needed for the service messages compilation are imported.

generate_messages(
  DEPENDENCIES
  std_msgs
)
IV. catkin_package()
State here all of the packages that will be needed by someone that executes something from your package. All of the packages stated here must be in the package.xml file as <exec_depend>.

catkin_package(
      CATKIN_DEPENDS
      rospy
)
Once you're done, you should have something similar to this:

cmake_minimum_required(VERSION 2.8.3)
project(my_custom_srv_msg_pkg)
​
​
## Here is where all the packages needed to COMPILE the messages of topics, services and actions go.
## It's only getting its paths, and not really importing them to be used in the compilation.
## It's only for further functions in CMakeLists.txt to be able to find those packages.
## In package.xml you have to state them as build
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)
​
## Generate services in the 'srv' folder
## In this function will be all the action messages of this package ( in the action folder ) to be compiled.
## You can state that it gets all the actions inside the action directory: DIRECTORY action
## Or just the action messages stated explicitly: FILES my_custom_action.action
## In your case you only need to do one of two things, as you wish.
add_service_files(
  FILES
  MyCustomServiceMessage.srv
)
​
## Here is where the packages needed for the action messages compilation are imported.
generate_messages(
  DEPENDENCIES
  std_msgs
)
​
## State here all the packages that will be needed by someone that executes something from your package.
## All the packages stated here must be in the package.xml as exec_depend
catkin_package(
  CATKIN_DEPENDS rospy
)
​
​
include_directories(
  ${catkin_INCLUDE_DIRS}
)
Modification of package.xml:
Add all of the packages needed to compile the messages.
In this case, you only need to add the message_generation.
You will have to import those packages as <build_depend>.

On the other hand, if you need a package for the execution of the programs inside your package, you will have to import those packages as <exec_depend>.
In this case, you will only need to add these 3 lines to your package.xml file:

<build_depend>message_generation</build_depend>
​
<build_export_depend>message_runtime</build_export_depend>
<exec_depend>message_runtime</exec_depend>
So, at the end, you should have something similar to this:

<?xml version="1.0"?>
<package format="2">
  <name>my_custom_srv_msg_pkg</name>
  <version>0.0.0</version>
  <description>The my_custom_srv_msg_pkg package</description>
​
  <maintainer email="user@todo.todo">user</maintainer>
​
  <license>TODO</license>
​
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>message_generation</build_depend>
  <build_export_depend>rospy</build_export_depend>
  <exec_depend>rospy</exec_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>std_msgs</exec_depend>
  <build_export_depend>message_runtime</build_export_depend>
  <exec_depend>message_runtime</exec_depend>
​
  <export>
  </export>
</package>
Once you're done, compile your package and source the newly generated messages:

roscd;cd ..
catkin_make
source devel/setup.bash
**Important!!** When you compile new messages through catkin_make, there is an extra step that needs to be done. You have to type in the WebShell, in the **catkin_ws** directory, the following command: **source devel/setup.bash**.

This command executes the bash file that sets, among other things, the newly generated messages created with **catkin_make**.
If you don't do this, it might give you a Python import error, saying that it doesn't find the message generated.
You should see among all the messages something similar to:

Generating Python code from SRV my_custom_srv_msg_pkg/MyCustomServiceMessage

To check that you have the new service message in your system, and ready to be used, type the following:

Execute in WebShell #1

rossrv list | grep MyCustomServiceMessage
It should output something like:

WebShell #1 Output

user ~ $ rossrv list | grep MyCustomServiceMessage
my_custom_srv_msg_pkg/MyCustomServiceMessage
That's it! You have created your own Service Message. Now, create a Service Server that uses this type of message.

It could be something similar to this:

**Python Program {3.3}: custom_service_server.py**


#! /usr/bin/env python
​
import rospy
from my_custom_srv_msg_pkg.srv import MyCustomServiceMessage, MyCustomServiceMessageResponse # you import the service message python classes 
                                                                                         # generated from MyCustomServiceMessage.srv.
​
​
def my_callback(request):
    
    print "Request Data==> duration="+str(request.duration)
    my_response = MyCustomServiceMessageResponse()
    if request.duration > 5.0:
        my_response.success = True
    else:
        my_response.success = False
    return  my_response # the service Response class, in this case MyCustomServiceMessageResponse
​
rospy.init_node('service_client') 
my_service = rospy.Service('/my_service', MyCustomServiceMessage , my_callback) # create the Service called my_service with the defined callback
rospy.spin() # maintain the service open.
**END Python Program {3.3}: custom_service_server.py**


END **Example 3.8**

**Exercise 3.3**

Create a new Python file called bb8_move_custom_service_server.py. Inside this file, modify the code you used in Exercise 3.2, which contained a Service Server that accepted an Empty Service message to activate the circle movement. This new service will be called /move_bb8_in_circle_custom. This new service will have to be called through a custom service message. The structure of this custom message is presented below:
int32 duration    # The time (in seconds) during which BB-8 will keep moving in circles
---
bool success      # Did it achieve it?
Use the data passed to this new /move_bb8_in_circle_custom to change the BB-8 behavior.
During the specified duration time, BB-8 will keep moving in circles. Once this time has ended, BB-8 will then stop its movement and the Service Server will return a True value (in the success variable). Keep in mind that even after BB-8 stops moving, there might still be some rotation on the robot, due to inertia.
Create a new launch file called start_bb8_move_custom_service_server.launch that launches the new bb8_move_custom_service_server.py file.
Test that when calling this new /move_bb8_in_circle_custom service, BB-8 moves accordingly.
Create a new python code called call_bb8_move_custom_service_server.py that calls the service /move_bb8_in_circle_custom. Remember how it was done in Unit 3 Services Part 1.

Then, generate a new launch file called call_bb8_move_custom_service_server.launch that executes the call_bb8_move_custom_service_server.py through a node.
**END of Exercise 3.3**

Solution Exercise 3.3

Please try to do it by yourself unless you get stuck or need some inspiration. You will learn much more if you fight for each exercise.



Follow this link to open the solutions notebook for Unit3 Services Part2: solutions_unit3_services_part2

END Solution Exercise 3.3

Summary
Let's do a quick summary of the most important parts of ROS Services, just to try to put everything in place.

A ROS Service provides a certain functionality of your robot. A ROS Service is composed of 2 parts:

Service Server: This is what PROVIDES the functionality. Whatever you want your Service to do, you have to place it in the Service Server.

Service Client: This is what CALLS the functionality provided by the Service Server. That is, it CALLS the Service Server.

ROS Services use a special service message, which is composed of 2 parts:

Request: The request is the part of the message that is used to CALL the Service. Therefore, it is sent by the Service Client to the Service Server.
Response: The response is the part of the message that is returned by the Service Server to the Service Client, once the Service has finished.
ROS Services are synchronous. This means that whenever you CALL a Service Server, you have to wait until the Service has finished (and returns a response) before you can do other stuff with your robot.
