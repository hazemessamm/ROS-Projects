
ROS IN 5 DAYS 
Appendix 1: Installing ROS


How to install ROS in my local computer?
During the whole Course, you have already been using ROS with its full capabilities. In fact, you are almost a master in ROS programming! And that is because in our Robot Ignite Academy, you already have everything installed and set up, so that you can go straight and focus in learning the really important things of ROS: how to apply it to interact with robots. But what if you want now to apply everything you have learned during the Course, in your local computer?

Well, the first step would be, of course, to install ROS in your local computer. And that is what you are going to do during this appendix. For this case, we are going to explain the steps to install and setup the same environment that you've been using during the whole Course. This is, a ROS Kinetic distribution installed in an Ubuntu 16.04 Xenial machine. If you want to install a different version, or you are using a different machine, please refer to the official documentation, here: http://wiki.ros.org/kinetic/Installation

With the proper introductions made, let's go with the steps needed in order to install ROS.

Setup your sources.list
First of all, you'll need to setup your computer in order to be able to download packages from packages.ros.org. For that, execute the below command in your local shell:

Execute in Local Shell #1

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
Setup your keys
Next, you will download the key from the keyserver using the following command:

Execute in Local Shell #1

sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
If everything goes fine, you should see something like this:



Installation
Great! Now we are ready to actually install ROS. First of all, we'll make sure that our Debian packages index is up to date. For that, execute the following command:

Execute in Local Shell #1

sudo apt-get update
Great! Now you are ready to start installing ROS packages into your system. In order to have all the basic packages for start working with ROS, we recommend you to install the Desktop Full installation. For doing so, you can execute the following command:

Execute in Local Shell #1

sudo apt-get install ros-kinetic-desktop-full
NOTE: Since it will download and install several packages, this installation can take some minutes. So be patient.

At this point, you have installed some of the basic tools that ROS provides, like RViz, rqt, navigation libraries... With these tools, you will be ready to start working with ROS. Anyways, you will need to install some extra packages eventually. For installing an specific ROS package, you just need to use the following command structure:

sudo apt-get install ros-kinetic-<PACKAGE_NAME>
For instance:

sudo apt-get install ros-kinetic-slam-gmapping
The above command will install the slam gmapping package for the ROS Kinetic version.

Initialize rosdep
Before you can actually start using ROS, though, you will need to initialize rosdep. rosdep will allow you to easily install system dependencies, and it is also required to run some core components in ROS. To initialize rosdep, execute the following command:

Execute in Local Shell #1

sudo rosdep init
rosdep update
If everything goes fine, you should get something like this after executing the rosdep update command.



Environment setup
Finally, it is also recommended to automatically add the ROS Environment Variables (do you remember them from the 1st Chapter of the Course?) to your bash session every time a new shell is launched. For doing so, you can execute the following command:

Execute in Local Shell #1

source /opt/ros/kinetic/setup.bash
You will need to run this command on every new shell you open to have access to the ROS commands, unless you add this line to your .bashrc file. So, unless you wish to run that command every time you open a new shell, you should add it to your .bashrc. For that, you can run the following command:

Execute in Local Shell #1

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
With the above command, you will add the line source /opt/ros/kinetic/setup.bash to your .bashrc file. This way, each time you open a new shell in your computer, all the ROS Environment variables will be automatically set up.

Also, this process allows you to install several ROS distributions (for instance, indigo and kinetic) on the same computer and switch between them. So, for instance, if you also had ROS Indigo installed in your local computer, you could switch between both distributions by using the bellow commands:

source /opt/ros/indigo/setup.bash # To use Indigo
source /opt/ros/kinetic/setup.bash # To use Kinetic
Dependencies for building packages
Great! So at this point, you have already installed and set up everything you need to run the core ROS packages. Anyways, there are various tools that you will also need in order to manage your ROS workspaces (remember your catkin_ws?). To install all this tools, you can run the following command:

Execute in Local Shell #1

sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
Test your setup
And we are done! Now let's test that our setup actually works, and that we can run ROS on our local machine. For that, let's follow the next Example.

**Example A.1**

a) In the same shell where you have set up everything during this Notebook, run the following command:

Execute in Local Shell #1

roscore
If everything goes fine, you should see something like this:



By the way... do you remember roscore? We introduced it to you back in the first chapter of this Course. roscore is the main process that manages all the ROS systems. So, if we want to do anything with ROS, we will always need to first start roscore in one shell.

And you may ask yourself... and why I didn't use this command during the Course, if it so important? Well, it is because in the Ignite Academy, the roscore is automatically started for you whenever you enter a Course. But now, if you want to work in your local computer, you will need to manage it yourself!

b) With the roscore running in one shell, let's open a new shell. Within this new shell, type the following command:

Execute in Local Shell #2

rostopic list
If everything goes fine, you should get the following topics:



Also, you can execute the roscd command to make sure that your ROS system is properly set.

Execute in Local Shell #2

roscd
If everything goes fine, you should go to the following path:



Excellent! So you have successfully installed ROS in your local computer.

**End Example A.1**

Excellent! So you have successfully installed ROS in your local computer. In the next chapter, Appendix 2, you will see how to create new workspaces and how to manage them, so that you can start developing your own ROS packages.

