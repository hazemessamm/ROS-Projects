First Step: catkin_create_pkg my_custom_srv_msg_pkg rospy #this will create package to store the message in it
Second Step: Create a srv folder inside your package , as you did with the topics msg folder. Then, inside this srv folder, create a file called MyCustomServiceMessage.srv. You can create with the IDE or the WebShell, as you wish.
2nd A): roscd my_custom_srv_msg_pkg/
2nd B): mkdir srv #this will make directory to store the service message in it
2nd C): vim srv/MyCustomServiceMessage.srv #this will create and open the file to write inside it
This what will be written for example:
int32 duration    # The time (in seconds) during which BB-8 will keep moving in circles
---
bool success      # Did it achieve it?

the 3 dashes means that the above the 3 dashes is the request and what is below the 3 dashes is the respond

3rd Step: How to Prepare CMakeLists.txt and package.xml for Custom Service Compilation.

You have to edit two files in the package similarly to how we explained for Topics:
CMakeLists.txt
package.xml

Modification of CMakeLists.txt
You will have to edit four functions inside CMakeLists.txt:

find_package()
add_service_files()
generate_messages()
catkin_package()

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_service_files(
  FILES
  MyCustomServiceMessage.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
      CATKIN_DEPENDS
      rospy
)

------------------------Modification in package.xml------------------------

Add all of the packages needed to compile the messages.
In this case, you only need to add the message_generation.
You will have to import those packages as <build_depend>.

On the other hand, if you need a package for the execution of the programs inside your package, you will have to import those packages as <exec_depend>.

In this case, you will only need to add these 3 lines to your package.xml file:
<build_depend>message_generation</build_depend>

<build_export_depend>message_runtime</build_export_depend>
<exec_depend>message_runtime</exec_depend>

final Step is compiling everything together (Compiling must be done in the catkin_ws directory):
roscd;cd ..
catkin_make
source devel/setup.bash

source devel/setup.bash is important because without it the python will give you an ImportError

to check the service message you will write the following command:
rossrv list | grep MyCustomServiceMessage
