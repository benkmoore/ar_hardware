# ar_hardware

## Teensy Notes

### Setup

1. Test functioning Teensy: [https://www.pjrc.com/teensy/first_use.html]

2. Download Teensyduino, Version 1.52 to jetson from: [https://www.pjrc.com/teensy/td_download.html]
Follow instructions. Note: Download Arduino 1.8.12/latest tested support. (We need to determine which libraries are needed so we don't install them all for production)

3. Extract arduino dowloaded files. Install Arduino IDE by running `./installArduinoIDE.sh`

4. Check Teensey Board is visable in Arduino IDE -> Tools -> Board and select Teensy 4.0

5. Install Teensy loader for command line from source. Follow instructions: [https://www.pjrc.com/teensy/loader_cli.html]

6. Compile test_stepperMotor.ino in ArduinoIDE using Sketch -> Compile. Load onto teesny: Press load button on teensy (need to use command line for this later using teensy_loader_cli). When uploading via the command line make sure the programming symbol adn then the reboot symbol appear on the teensy gui. If you find that upload is stalling check the usb connection to the usb and replug/reset if needed. Pressing the manual upload button on the teensy is a workaround but try to fix the usb connection as this can cause serial communication problems when you launch the ROS. Make sure usb cable is securely connected to nano, replace/swap usb cableif you suspect it's faulty.

7. Setup port for teensy in ArudinoIDE Tools -> Port, select Teensy. Tools -> Serial Monitor, outputs console of teensy programs. Good way to check program is running correctly on teensy.


-----------------------------------------------------------------------------------------------------

### Setup for ROS msgs over Teensy

1. Install ROS serial communication package for arduino. For more details o the isntall commands below see: [http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup]

To install ROS serial pkgs run the following commands:
`sudo apt-get install ros-melodic-rosserial-arduino `

AND:
rosserial_client
rosserial_embeddedlinux
rosserial_mbed
rosserial_msgs
rosserial_python
rosserial_server
rosserial_test

`sudo apt-get install ros-melodic-rosserial`

To install necccesarry ros libarires in the arduino environment run the following commands:
`cd Arduino/libraries`
`rosrun rosserial_arduino make_libraries.py` .

To check install:
Restart ArduinoIDE -> File -> Examples -> ros_lib -> Hello_world

2. Add teensy 4.0 serial functionality to line 44 in Arduino/libraries/ros_lib/ArduinoHardware.h . See [pjrc.com/teensy/loader_cli.html] for more info on the names of each MCU/teensy X.X .

Code to append: `defined(__IMXRT1062__)`

Line 44 should now look like:
`#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__MKL26Z64__) || defined(__IMXRT1062__)`

3. Add custom AR messages to arduino ros_lib: http://wiki.ros.org/rosserial_arduino/Tutorials/Adding%20Custom%20Messages, using this command

`rosrun rosserial_arduino make_libraries.py /home/ben/Arduino/libraries /home/ben/catkin_ws/src/ar_commander/msg`


NOTE: Always delete the existing ros_lib folder in `~/Arduino/libraries` before rebuilding headers for msgs otherwise no updates will be made.


------------------------------------------------------------------------------------------------

### Creating connection over usb between teensy node and ros master on jetson

1. Check baud rate on teensy - default is 57600. If baud rate is too high can cause comms issues. Start with 57600 can go higher later and test. Load script onto teensy with teens_loader.

2. Check teensy port on AarduinoIDe -> Tools -> Port. Check that the port exists: `ls /dev/ttyACM0`. Check port has read and write permissions: `sudo chmod 777 /dev/ttyACM0`

3. (a) Make changes to SerialClient.py as per issue at: https://analyticalrobotics.atlassian.net/browse/AR1-40

3. (b) Compile arduino file using load script: `./load_arduino_file.sh motor_interface.ino`

3. (c) After launching ros master run: `rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600`

make sure the correct port and baud rate is used. If connection succesful will see this:
`INFO [1593459264.359302]: ROS Serial Python Node`
`INFO [1593459264.368955]: Connecting to /dev/ttyACM0 at 57600 baud`
`INFO [1593459266.475355]: Requesting topics...`
`INFO [1593459269.612953]: Note: subscribe buffer size is 2048 bytes`
`INFO [1593459269.616665]: Setup subscriber on controller_cmds [ar_commander/ControllerCmd]`


4. Test with controller msg: `rostopic pub -r 10 /controller_cmds ar_commander/ControllerCmd "{velocity_arr:ta: [0,0,0,0]}, phi_arr: {data:[1.5, 1.5, 1.5, 1.5]}}"`


5. Check that the teensy is receiving the messages by monitoring the `/chatter` (may need to add this manually to `hardware_interface.ino`) topic which is published within the controller_cmds callback from the teensy. See [Teensy Setup Notes](#setup-for-ros-msgs-over-teensy) for more debug info on this.

#### Debugging notes:

A. Make sure you have serial monitor closed and no serial print lines in your code. Cant use both the serial usb and serial monitor at the same time. If you do you'll get this error: `"Unable to sync with device; possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino"`

B. Make sure the buffer size is big enough for a backlog of your msgs:

`ros::NodeHandle_<HardwareType, MAX_PUBLISHERS=25, MAX_SUBSCRIBERS=25, IN_BUFFER_SIZE=512, OUT_BUFFER_SIZE=512> nh;`

More info: http://wiki.ros.org/rosserial/Overview/Initialization

C. Unplug and replug teensy to clear port if needed (last resort)

D. Compile code and reboot teensy

E. Increase time out if error is  `"lost sync..."` see: https://analyticalrobotics.atlassian.net/browse/AR1-39

F. Check that correct ports are being called for the teensy and other decawave boards see [localization notes](#localization-notes).

G. `check sum error` or `wrong msg id`. 1) Check that the msg is updated and built correctly for the msg version in ar_commander in `~/Arduio/libbraries/ros_lib` remake if not see [teensy msg setup](#setup-for-ros-msgs-over-teensy) for details. 2) Check that the hardware interface is accessed the msg in the correct format. 3) Check you are  njot overflowing the buffer by publishing in the loop/over publishing.


---------------------------------------------------------------------------------------------------

### Decawave dev board setup:

1. Download J-Link Software and Documentation pack [here](https://www.segger.com/downloads/jlink/#J-LinkSoftwareAndDocumentationPack)

2. Download the firmware image and all the documentation and install android app [here](https://www.decawave.com/product/dwm1001-development-board/)

3. Flash the boards with the firmware image from above using j-flash lite and following the instructions on page 14 of the DWM1001 Firmware User Guide -Version 1.0 from 2017. Note: this is different to the firmware API guide.

4. Connect to boards using android app & follow instructions to configure as anchor or tag. NB: This step isn't necessary, it can all be done through minicom or python but the app has a simple gui and is the fastest method especially if you're not sure what you're doing.

5. Download minicom using `sudo apt-get install minicom`.

6. To connect to a board over USB, make sure that the correct port is being opened. To check this, before plugging the board in, use `dmesg | grep tty`. Then plug in the board and use `dmesg | grep tty` again. There should be a new entry that looks something like `ttyACM0`, where the last number may be different.

7. Use `sudo minicom -D /dev/ttyACM0` using the correct port found above in place of `ttyACM0`. `Welcome to minicom` screen should show up.

8. Press enter/return on the keyboard twice within 1 second to enter shell mode. If the port is correct and the USB cable is good and the firmware is good, you should see `DWM1001 TWR Real Time Location System`, and `dwm>`. Type `apg` and hit enter to get current tag pose.



#### Debugging notes:

A. Check port numbers are correct for each usb connection - NB. Decawave/teensy will appear to hang but is trying to send signals acorrs the wrong port or a decawave port. Decawaves may also be trying to send signals to a "teensy" port. Check the ports using `dmesg | grep tty` while removing/plugging in usbs. It appears that port numbering is selected at startup and once first inserted and doesnt change if you unplug after that.

B. Anytime you connect to the board via USB make sure you use a good, reliable data cable or you will have a bad time.

C. The boards should all be perpendicular to the ground, ie. the antenna should be the furthest point from the ground. If the boards are horizontal they won't be able to communicate properly.






---------------------------------------------------------------------------------------------------

### Setup Time-Of-Flight (TOF) sensors - VL53L1X

1. Setup wiring of multiple TOFs as in wiring digram (here)[/home/ben/Arduino/test_TOF]. Note use 3.3V source, some articles suggest 5V but that didnt work.

2. Install VL53L1X arduino package: ArduinoIDE -> Sketch -> Include Libraries -> Manage Libraries -> VL53L1X.

3. Upload the `catkin_ws/src/ar_hardware/scripts/tof_tests/test_TOF3` script to the teensy: `./load_arduino_file.sh test_TOF3_VL53L1X.ino` and view serial monitor to check outputted distances.

NOTE: VL53L1X [docs](https://github.com/pololu/vl53l1x-arduino) and [library](https://www.pololu.com/product/3415).

---------------------------------------------------------------------------------------------------

### Stepper Motors & Drivers

1. NOTE: For stepper motors adjust voltage across motor driver, DRV8825: see video at this link: https://youtu.be/89BHS9hfSUk, more info on DRV8825: [https://www.pololu.com/product/2133]. Aim for 0.5V or less (rotation appears smoothest at 0.35V)
