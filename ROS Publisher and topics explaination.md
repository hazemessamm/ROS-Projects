#! /usr/bin/env python

import rospy
from std_msgs.msg import Int32 

rospy.init_node('topic_publisher')
pub = rospy.Publisher('/counter', Int32, queue_size=1)
rate = rospy.Rate(2)
count = Int32()
count.data = 0

while not rospy.is_shutdown(): 
  pub.publish(count)
  count.data += 1
  rate.sleep()
  
  Nothing happens? Well... that's not actually true! You have just created a topic named /counter, 
  and published through it as an integer that increases indefinitely. Let's check some things.

A topic is like a pipe. Nodes use topics to publish information for other nodes so that they can communicate.
You can find out, at any time, the number of topics in the system by doing a rostopic list. You can also check for a specific topic.

On your webshell, type rostopic list and check for a topic named '/counter'.

user-$ rostopic list | grep  '/counter'

user ~ $ rostopic list | grep '/counter'
/counter

Here, you have just listed all of the topics running right now and filtered with the grep command the ones that contain the word /counter. If it appears, then the topic is running as it should.

You can request information about a topic by doing rostopic info <name_of_topic>.

Now, type rostopic info /counter.

user ~ $ rostopic info /counter
Type: std_msgs/Int32

Publishers:
 * /topic_publisher (http://ip-172-31-16-133:47971/)

Subscribers: None

The output indicates the type of information published (std_msgs/Int32), the node that is publishing this information (/topic_publisher), and if there is a node listening to that info (None in this case).

Now, type rostopic echo /counter and check the output of the topic in realtime.


rostopic echo /counter
data:
985
---
data:
986
---
data:
987
---
data:
988
---


Ok, so... what has just happened? Let's explain it in more detail. 
First, let's crumble the code we've executed. You can check the comments in the code below explaining what each line of the code does:


#! /usr/bin/env python

import rospy                               # Import the Python library for ROS
from std_msgs.msg import Int32             # Import the Int32 message from the std_msgs package

rospy.init_node('topic_publisher')         # Initiate a Node named 'topic_publisher'
pub = rospy.Publisher('/counter', Int32, queue_size=1)    
                                           # Create a Publisher object, that will publish on the /counter topic
                                           # messages of type Int32

rate = rospy.Rate(2)                       # Set a publish rate of 2 Hz
count = Int32()                            # Create a var of type Int32
count.data = 0                             # Initialize 'count' variable

while not rospy.is_shutdown():             # Create a loop that will go until someone stops the program execution
  pub.publish(count)                       # Publish the message within the 'count' variable
  count.data += 1                          # Increment 'count' variable
  rate.sleep()                             # Make sure the publish rate maintains at 2 Hz
  
  
  So basically, what this code does is to initiate a node and create a publisher that keeps publishing into the '/counter' topic a sequence of consecutive integers. Summarizing:

A publisher is a node that keeps publishing a message into a topic. So now... what's a topic?

A topic is a channel that acts as a pipe, where other ROS nodes can either publish or read information. 
Let's now see some commands related to topics (some of them you've already used).

To get a list of available topics in a ROS system, you have to use the next command:


rostopic list

To read the information that is being published in a topic, use the next command:

rostopic echo <topic_name>


This command will start printing all of the information that is being published into the topic, 
which sometimes (ie: when there's a massive amount of information, 
or when messages have a very large structure) can be annoying. In this case, 
you can read just the last message published into a topic with the next command:

rostopic echo <topic_name> -n1

To get information about a certain topic, use the next command:

rostopic info <topic_name>

Finally, you can check the different options that rostopic command has by using the next command

rostopic -h

Messages
As you may have noticed, topics handle information through messages. There are many different types of messages.

In the case of the code you executed before, the message type was an std_msgs/Int32, but ROS provides a lot of different messages. You can even create your own messages, but it is recommended to use ROS default messages when its possible.

Messages are defined in .msg files, which are located inside a msg directory of a package.

To get information about a message, you use the next command:

rosmsg show <message>

For example, let's try to get information about the std_msgs/Int32 message. Type the following command and check the output

rosmsg show std_msgs/Int32

user ~ $ rosmsg show std_msgs/Int32
[std_msgs/Int32]:
int32 data

n this case, the Int32 message has only one variable named data of type int32. 
This Int32 message comes from the package std_msgs, and you can find it in its msg directory. If you want, 
you can have a look at the Int32.msg file by executing the following command:

roscd std_msgs/msg/

