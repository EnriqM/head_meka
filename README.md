### Head Meka

Project to give 2DOF to cameras installed on Meka robot, in this case, Kinect
You will need meka_description and common_sensors (to load Kinect model properly) from [here] (https://github.com/EnriqM/meka_pmb2_ros/) 
Reference to pololu and CNC shield for schematics

Designs were entirely made (and more complete) by JDLR in [reference his project will be here in upcoming days]
Connection of end switches are connected like this:
![end_switch_image.png](/end_switch_image.png)

COM goes to X+ (or Z+ for the other DOF). NC is connected to GND (both of them). Input_pullup from Arduino is used to avoid errors.



![Pic_mekaheadKinect.png](/Pic_mekaheadKinect.png)

There are a few steps in order to upload your code in Arduino and setting it up:

### Upload and compile your code
1. Download the code (don't forget meka_description and common_sensors!)
  Add the git repository to your catkin workspace:
```
cd <your-catkin-ws>/src
git clone https://github.com/EnriqM/head_meka.git
```
Make sure you give permissions to the python files !!

2. Make sure you download the AccelStepper ([AccelStepper Library Download](http://www.airspayce.com/mikem/arduino/AccelStepper/AccelStepper-1.57.zip)) and ros_lib ([rosserial-arduino tutorial](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)) libraries into your Arduino environment.
	- If ros_lib already exists in your Arduino libraries (<Arduino sketchbook>/libraries), follow the last troubleshooting tip or you'll get an error.  ROS makes you remove ros_lib and regenerate it every time you introduce a new custom message.
 
3. Run the next code. At this point, your Arduino needs to be connected, you'll have write the next commands. Serial port may change, you can check it easily using Arduino IDE
EDIT: script from Arduino that is currently used is ![THIS ONE](/headMeka_ArduinoROS/headMeka_ArduinoROS.ino)
```
rosrun rosserial_python serial_node.py /dev/ttyACM0 
roslaunch head_meka_description head_urdf.launch
rosrun head_meka_control steps_goals.py 
```

4. Now you'll se a picture like this. Let Arduino auuto-calibrate properly
![Pic_RVIZ.png](/Pic_RVIZ.png)
You can use joint_states GUI to operate and change the position of your Arduino
