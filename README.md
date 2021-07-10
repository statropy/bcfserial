# bcfserial
Kernel driver utilizing serdev to connect WPAN via HDLC over uart

# Installation Instructions for Pocket Beagle

## Installing the Device Tree Overlay

    > cd dts
    > dtc -O dtb -o bcfserial.dtbo -b 0 -@ bcfserial.dts
    > sudo cp ./bcfserial.dtbo /lib/firmware/

Add the following line to `/boot/uEnv.txt`

    dtb_overlay=/lib/firmware/bcfserial.dtbo

Reboot the board

## Connecting the BeagleConnect Freedom to the Pocket Beagle

With the orientation of the BeagleConnect Freedom board pigtail antenna 'up' and the USB/JST connectors 'down', and the board flipped to the buttons on the underside:

|Pocket Beagle |  | BeagleConnect Freedom|
|--------------|----------|----------------------|
|U4 RX | < ---- > | Right side connector, 3rd from top|
|U4 TX | < ---- > | Right side connector, 4th from top|
|GND | < ---- > | Right side connector, last from top|


Power the BeagleConect Freedom with a LiPo battery. Do not connect USB.

## Building the module

If not already present, insall the Kernel header files for your system

    > sudo apt install linux-headers-$(uname -r)

Then build the module

    > make

## Loading the module

    > sudo ./modprode.sh

## Starting the LOWPAN device

    > sudo ./lowpan.sh

This will create the lowpan0 device with IP address 2001:db8::2/64 using channel 26 with PAN ID 0xabcd.
