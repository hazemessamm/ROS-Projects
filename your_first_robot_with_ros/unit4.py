
Building and Rosifying a Robot from Scratch


Unit 3: Connecting to your RIAbot real robot
SUMMARY

Estimated time to completion: Unknown hours

In this unit, you are going to see how to connect to your RIAbot real robot, both using an Ethernet wire and setting up the WIFI.

END OF SUMMARY

Connection through Ethernet
1. Download the ROBsots ROS+OpenCV Raspbian Image in the following link: https://medium.com/@rosbots/ready-to-use-image-raspbian-stretch-ros-opencv-324d6f8dcd96.

NOTE: When downloaded, remember to uncompress the image file, so that you end up with a .img file.

2. On a Windows, Linux or Mac machine, install the image on to your micro-sd card which you plan to use for your Raspberry Pi. For that, you can follow the official Raspberry Pi instructions described in the following link: https://www.raspberrypi.org/documentation/installation/installing-images/README.md

At the end of the page, you will see detailed instructions for each OS.

IMPORTANT NOTE: When following the instructions to write the image to your micro-sd card, you will need to use the Image you downloaded on step 1, instead of their Raspbian image.

3. Connect your RPi to your network using an ethernet cable.

4. Insert the micro-sd card into your RPi, and plug in the power. The micro-sd card is inserted under the RPi, so maybe you'll need to take the RPi out for a moment in order to insert the card.



To plug in the power of the RPi, you will need to put the small switch of the PowerPack to ON.



4.b. If everything is fine, the light on the Ethernet connector should light up.



5. Find the IP address assigned to you Raspberry Pi's ethernet module:

Note: On a Mac or Linux system, that's easily discoverable using the following command:

nmap -sP x.x.x.0/24
Where you will replace x.x.x.0 with your network subnet. So, for instance, if your network IP is 192.168.1.129, the command to execute would be the following:

nmap -sP 192.168.1.0/24
This command will give an output similar to this one:

Starting Nmap 7.70 ( https://nmap.org ) at 2018-11-23 12:42 CET
Nmap scan report for 192.168.1.1
Host is up (0.0030s latency).
Nmap scan report for 192.168.1.129
Host is up (0.00064s latency).
Nmap scan report for 192.168.1.132
Host is up (0.019s latency).
Nmap done: 256 IP addresses (3 hosts up) scanned in 2.81 seconds
HINT: Probably, the last IP that appears on the list will be the one assigned to your Rpi.

6. Now, let's access through ssh to our RIAbot. Type the following command:

ssh pi@xxx.xxx.xxx.xxx
Where you will replace xxx.xxx.xxx.xxx with the IP address assigned to your RPi. The password is rosbots!.

7. Once logged in, run the following commands in the ssh terminal:

update_rosbots_git_repos
This will retrieve the latest setup routines. Also, type the following command:

initialize_rosbots_image
This command will do several things:

Set up new SSH keys
Ask you for a new password
Run raspi-config, where you will enable your Pi Camera and expand your filesystem to use entire SD card space.
When first running raspi-config, you should see a screen like this one:



First, you will select Interfacing Options -> Camera -> Enable.



Now, you will select Advanced Options -> Expand Filesystems





Finally, just Finish and Reboot



8. Wait a couple of minutes and ssh back into your Pi. To make sure ROS is running, type the following command:

rosnode list
You should see the following topic:



9. Finally, let's complete the software setup. From an ssh terminal into your RPi, type the following command:

setup_rosbots_code
After some seconds, it will ask you for your Rpi password, so be aware. After you've entered your password, it will take some time to finish all the setup, so take it easy and be patient!

When setup has finished, check again the ROS nodes running:

rosnode list
You should now see the following topics:



Connection with WIFI
1. cd into the rpi_setup/ folder directory in the repository.

cd rpi_setup/
2. Type the following command:

fab -H ipaddressforyourpi setup_wifi_on_pi
and enter your password for your Pi.

3. Answer the questions about your WiFi network (select a 2.4 GHz network for greater range).

4. With your ethernet still connected into your Pi, SSH into your Pi. While ssh'd into your Pi, turn on then off the wlan0 interface:

sudo ip link set wlan0 down
sudo ip link set wlan0 up
Wait about 1-2 minutes for wifi to connect to your access point.

5. If above doesn't work, then try reboot via:

sudo shutdown -r now
Count to 60 to make sure the Pi has rebooted.

6. Unplug your Pi's ethernet cable.

7. Discover your Pi's new IP address assigned to its Wifi module:

sudo nmap -sP x.x.x.0/24
Where you replace x.x.x.0 with your network subnet.

8. Now you can SSH into your Pi with the new IP Address assigned to your Pi's Wifi.

With the pre-build Raspbian ROS+OpenCV image, set up ROSbots modules
1. cd into the rpi_setup/ folder directory in the repository.

cd rpi_setup/
2. Type the following command:

fab -H ipaddressforyourpi main_setup_only_rosbots_components
And enter your password for your Pi.

3. Wait for completion (about 30 minutes).

4. Type the following command:

rosnode list
And you should get the following topic running:

/uno_serial_node
NOTE: If the arduino you are using is not original, you might need to do some things in order to be able to upload the firmware. You can have a look at the following link: http://docs.platformio.org/en/latest/faq.html#faq-udev-rules

Set up ROS, OpenCV, ROSbots modules
1. cd into the rpi_setup/ folder directory in the repository.

cd rpi_setup/
2. Type the following command:

fab -H ipaddressforyourpi main_setup_ros_opencv_for_rosbots
And enter your password for your Pi.

Note 1: This step will take a long while (about 4-6 hours) so leave this running and check in every now and then to make sure everything is ok.

Note 2: There are cases in which the command may not finish correctly due to small swap space in the RPi. You can have a look at the following tutorial in order to increase the swap memory: https://www.bitpi.co/2015/02/11/how-to-change-raspberry-pis-swapfile-size-on-rasbian/

3. When the step completes:

SSH into your Pi
Type "rosnode list" to see the current ROS nodes running.
