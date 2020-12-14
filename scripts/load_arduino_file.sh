#!/bin/bash

board="$1"
file="$2"
port="ttyTeensy"

if [ -z "$1" ]; then echo Must specify a teensy board and file to compile and upload e.g. ./load_arduino_file.sh [teensy40, teensy41] [file_name.ino]; exit 1; fi

if [ -z "$2" ]; then echo Must specify a teensy board and file to compile and upload e.g. ./load_arduino_file.sh [teensy40, teensy41] [file_name.ino]; exit 1; fi

mkdir /tmp/arduino_build
mkdir /tmp/arduino_cache

file_path=$(find $PWD -path "*/catkin_ws/src/ar_hardware/*" -type f | grep "$file")
echo File being uploaded to teensy: "$file_path"
echo Board: "$board"
echo Port: "$port"

if [[ "$board" == "teensy40" ]]; then
~/arduino-1.8.12/arduino-builder -dump-prefs -logger=machine -hardware ~/arduino-1.8.12/hardware/ -tools ~/arduino-1.8.12/tools-builder -tools ~/arduino-1.8.12/hardware/tools/avr -built-in-libraries ~/arduino-1.8.12/libraries/ -libraries ~/Arduino/libraries -fqbn=teensy:avr:teensy40:usb=serial,speed=600,opt=o2std,keys=en-us -ide-version=10812 -build-path ~/tmp/arduino_build -warnings=none -build-cache /tmp/arduino_cache -verbose"$file_path" 

~/arduino-1.8.12/arduino-builder -compile -logger=machine -hardware ~/arduino-1.8.12/hardware/ -tools ~/arduino-1.8.12/tools-builder -tools ~/arduino-1.8.12/hardware/tools/avr -built-in-libraries ~/arduino-1.8.12/libraries/ -libraries ~/Arduino/libraries -fqbn=teensy:avr:teensy40:usb=serial,speed=600,opt=o2std,keys=en-us -ide-version=10812 -build-path /tmp/arduino_build -warnings=none -build-cache /tmp/arduino_cache -verbose "$file_path"

~/arduino-1.8.12/hardware/teensy/../tools/teensy_post_compile -file=$2 -path=/tmp/arduino_build -tools=/home/$USER/arduino-1.8.12/hardware/teensy/../tools -board=TEENSY40 -reboot -port=/sys/devices/70090000.xusb/usb1/1-2/1-2.4 -portlabel=/dev/"$port" Serial -portprotocol=Teensy

elif [[ "$board" == "teensy41" ]]; then
~/arduino-1.8.12/arduino-builder -dump-prefs -logger=machine -hardware ~/arduino-1.8.12/hardware/ -hardware ~/.arduino15/packages -tools ~/arduino-1.8.12/tools-builder -tools ~/arduino-1.8.12/hardware/tools/avr -tools ~/.arduino15/packages -built-in-libraries ~/arduino-1.8.12/libraries/ -libraries ~/Arduino/libraries -fqbn=teensy:avr:teensy41:usb=serial,speed=600,opt=o2std,keys=en-us -ide-version=10812 -build-path /tmp/arduino_build -warnings=none -build-cache /tmp/arduino_cache -verbose "$file_path" 

~/arduino-1.8.12/arduino-builder -compile -logger=machine -hardware ~/arduino-1.8.12/hardware/ -hardware ~/.arduino15/packages -tools ~/arduino-1.8.12/tools-builder -tools ~/arduino-1.8.12/hardware/tools/avr -tools ~/.arduino15/packages -built-in-libraries ~/arduino-1.8.12/libraries/ -libraries ~/Arduino/libraries -fqbn=teensy:avr:teensy41:usb=serial,speed=600,opt=o2std,keys=en-us -ide-version=10812 -build-path /tmp/arduino_build -warnings=none -build-cache /tmp/arduino_cache -verbose "$file_path"

~/arduino-1.8.12/hardware/teensy/../tools/teensy_post_compile -file=$2 -path=/tmp/arduino_build -tools=/home/$USER/arduino-1.8.12/hardware/teensy/../tools -board=TEENSY41 -reboot -port=/sys/devices/70090000.xusb/usb1/1-2/1-2.4 -portlabel=/dev/"$port" Serial -portprotocol=Teensy

else
	echo Invalid board;  
fi
