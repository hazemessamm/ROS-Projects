What will you learn with this unit?

What a service is
How to manage services of a robot
How to call a service
Part 1
Congratulations! You now know 75% of ROS-basics!

The reason is that, with topics, you can do more or less whatever you want and need for your astromech droid. 
Many ROS packages only use topics and have the work perfectly done.

Then why do you need to learn about services?

Well, that's because for some cases, topics are insufficient or just too cumbersome to use. 
Of course, you can destroy the Death Star with a stick, but you will just spend ages doing it. 
Better tell Luke SkyWalker to do it for you, right? Well, it's the same with services. They just make life easier.

Topics - Services - Actions
To understand what services are and when to use them, you have to compare them with topics and actions.

Imagine you have your own personal BB-8 robot. It has a laser sensor, a face-recognition system, 
and a navigation system. The laser will use a Topic to publish all of the laser readings at 20hz. 
We use a topic because we need to have that information available all the time for other ROS systems, such as the navigation system.

The Face-recognition system will provide a Service. Your ROS program will call that service 
and WAIT until it gives you the name of the person BB-8 has in front of it.

The navigation system will provide an Action. Your ROS program will call the action to move the robot somewhere, 
and WHILE it's performing that task, your program will perform other tasks, such as complain about how tiring C-3PO is. 
And that action will give you Feedback (for example: distance left to the desired coordinates) along the process of moving to the coordinates.

So... What's the difference between a Service and an Action?

Services are Synchronous. When your ROS program calls a service, your program can't continue until it receives a result from the service.
Actions are Asynchronous. It's like launching a new thread. When your ROS program calls an action, 
your program can perform other tasks while the action is being performed in another thread.

Conclusion: Use services when your program can't continue until it receives the result from the service.
