#!/bin/bash

file="$1"

if [ -z "$1" ]; then echo Must specify a file to compile and upload e.g. ./load_arduino_file.sh [file_name.ino]; exit 1; fi

mkdir /tmp/arduino_build
mkdir /tmp/arduino_cache

file_path=$(find $PWD -path "*/catkin_ws/src/ar_hardware/*" -type f | grep "$file")
echo File being uploaded to teensy: "$file_path"


~/arduino-1.8.12/arduino-builder -dump-prefs -logger=machine -hardware ~/arduino-1.8.12/hardware/ -tools ~/arduino-1.8.12/tools-builder -tools ~/arduino-1.8.12/hardware/tools/avr -built-in-libraries ~/arduino-1.8.12/libraries/ -libraries ~/Arduino/libraries -fqbn=teensy:avr:teensy40:usb=serial,speed=600,opt=o2std,keys=en-us -ide-version=10812 -build-path /tmp/arduino_build -warnings=none -build-cache /tmp/arduino_cache -verbose "$file_path" 

~/arduino-1.8.12/arduino-builder -compile -logger=machine -hardware ~/arduino-1.8.12/hardware/ -tools ~/arduino-1.8.12/tools-builder -tools ~/arduino-1.8.12/hardware/tools/avr -built-in-libraries ~/arduino-1.8.12/libraries/ -libraries ~/Arduino/libraries -fqbn=teensy:avr:teensy40:usb=serial,speed=600,opt=o2std,keys=en-us -ide-version=10812 -build-path /tmp/arduino_build -warnings=none -build-cache /tmp/arduino_cache -verbose "$file_path"

~/arduino-1.8.12/hardware/teensy/../tools/teensy_post_compile -file=$1 -path=/tmp/arduino_build -tools=/home/$USER/arduino-1.8.12/hardware/teensy/../tools -board=TEENSY40 -reboot -port=/sys/devices/70090000.xusb/usb1/1-2/1-2.4 -portlabel=/dev/ttyACM0 Serial -portprotocol=Teensy





