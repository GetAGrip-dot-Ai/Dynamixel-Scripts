# Dynamixel-Scripts

## two-motor-async

**Description:** Using the Dynamixel Shield SDK developed by Robotis [link](https://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/), actuates two motors that are chained together between two seperate goal points using position control. This mimics the behavior of opening and closing an end-effector. 

**Visualization:**<br>
![moving-motors](https://github.com/artrela/Dynamixel-Scripts/blob/main/Resources/ezgif.com-resize.gif)

**Required Resources:**
- Arduino (Mega or Uno)
- Power Supply Capable of Providing 12V Power to the Shield
- MX-28 & MX-64 Dynamixel Motors
- Compatible Dynamixel Wires 

**Required Downloads**: 
- Dynamixel2Arduino by Robotis v0.6.2 (via the Arduino IDE)
- DynamixelShield by Robotis v0.2.6 (via the Arduino IDE)

<hr>

## two-motor-async-ros

**Description:** Using the Dynamixel Shield SDK developed by Robotis [link](https://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/), actuates two motors that are chained together between two seperate goal points using position control. Additional functionalities are explained below.

**Functionality:**
| **Function**       | **Description**                                                                                         | **Purpose**                                                                                        |
|--------------------|---------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------|
| openCutter()       | Open the MX-64 motor using PID position Control                                                         |  ---                                                                                               |
| closeCutter()      | Open the MX-28 motor using PID position Control                                                         |  ---                                                                                               |
| checkOpenGripper() | By using a difference between expected position & current position, check if the motor moved            | openCutter() only ensures command was received by motor, not if it was opened                      |
| checkOpenCutter()  | By using a difference between expected position & current position, check if the motor moved            | openGripper() only ensures command was received by motor, not if it was opened                     |
| resetMotors()      | Factory Reset and Change the IDs of each motor                                                          | If the motor becomes overtorqued/overheats/etc. it shuts down to prevent damage and cannot be used |
| pingMotors()       |  ---                                                                                                    | Ensure Proper Communication is happening with motors                                               |
| update_state()     | Encoding the state of the end-effector (both open, both closed, etc.)                                   | To be tracked by the system or a GUI                                                               |
| harvestCb()        | Isolated states functionality to open, close, or some combination for autonomous or manual use over ROS |  ---            

**Required Resources:**
- Arduino Mega
- Power Supply Capable of Providing 12V Power to the Shield
- MX-28 & MX-64 Dynamixel Motors
- Compatible Dynamixel Wires 
- FTDI Chip & Compatible Cable

**Required Downloads**: 
- Rosserial by Michael Ferguson v0.9.1 (via the Arduino IDE)

**Special Notes:**

Rosserial Workarounds:
- When using a breakboard and rosserial, you cannot simply use the USB port provided by Arduino as both communicate over RX/TX pins
- One solution to this is using an Arduino Mega and using the extra Hardware Serial pin, along with a standard FTDI chip
- You also need to specify which hardware pin you want to communciate over (below)

```cpp
// set up the ros comms
  nh.initNode();
  Serial2.begin(57600);
  nh.getHardware()->setPort(&Serial2);
  nh.getHardware()->setBaud(57600);
```

Rosserial Setup:

**Step 1**: Install Libraries

Type the following code in the terminal : (Replace Indigo with your current version of ROS)


```powershell
sudo apt-get install ros-indigo-rosserial-arduino
sudo apt-get install ros-indigo-rosserial
```
**Step 2**: Create Ros-Lib

```powershell
cd ~/sketchbook/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```

(DO NOT FORGET THE PERIOD AT THE END)

**Step 3**: Start writing your program in the arduino IDE

**Step 4**: Run the code

**Step 4.1** Run roscore in a new terminal

**Step 4.2** Next, run the rosserial client application that forwards your Arduino messages to the rest of ROS. Make sure to use the correct serial port: (serial port to vary)

```powershell
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```

<hr>

## facotry_reset_id_change

**Description:** Using the Dynamixel Shield SDK developed by Robotis [link](https://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/), factory resets a motor using the Arduino, and changes its ID. Helpful if you don't have access to a [USB2Dynamixel](https://emanual.robotis.com/docs/en/parts/interface/usb2dynamixel/)

**Required Resources:**
- Arduino (Mega or Uno)
- Power Supply Capable of Providing 12V Power to the Shield
- Single Dynamixel Motor
- Compatible Dynamixel Wires 
- [USB Communciation Device](https://www.robotis.us/usb-downloader-ln-101_int/)

**Required Downloads**: 
- Dynamixel2Arduino by Robotis v0.6.2 (via the Arduino IDE)
- DynamixelShield by Robotis v0.2.6 (via the Arduino IDE)





