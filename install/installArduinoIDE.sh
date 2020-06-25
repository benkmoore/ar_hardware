#!/bin/bash

INSTALL_DIR=${HOME}
# Direct Jetson support starts at 1.8.10
ARDUINO_VERSION=1.8.12

# Only download if newer version exists
wget -N https://downloads.arduino.cc/arduino-$ARDUINO_VERSION-linuxaarch64.tar.xz
tar -C $INSTALL_DIR/ -xvf arduino-${ARDUINO_VERSION}-linuxaarch64.tar.xz
cd $INSTALL_DIR/arduino-${ARDUINO_VERSION}
sudo ./install.sh
./arduino-linux-setup.sh "$USER"
