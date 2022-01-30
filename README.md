# omni-robot-rpi

This repository includes code for controlling a mobile robot utilizing a commercial video-game gamepad.

The robot control runs on a Jetson Nano embedded PC or a RaspberryPi (GPIO library would need to be changed accordingly), and the gamepad is attached to a general use Laptop or Desktop PC.

The gamepad commands are transferred through gRPC from the Laptop/Desktop PC to the embedded PC via Wi-Fi.


Disclaimer:

This repository contains work in progress and its current purpose is to demonstrate gRPC as a means for software integration for robotics use. The author of this repository and code will not take responsibility of damages of any kind incurred by the use of this software.
