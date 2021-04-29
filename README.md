# PIANO MAN

## ABOUT
Developed as a part of UIUC'S CS 498- AI For Robot Control class, this project takes in piano `.mid` files and simulates a robot playing them! Functionality includes single-hand, multi-phalanged performance.

## SETUP
In order to have a stable development environment, install a virtual machine from scratch. It is recommended to use a local VM with GUI attached instead of using any form of X-forwarding due to complications with library dependencies.

### Tested VM Installation
- Download and install VirtualBox from https://www.virtualbox.org/
- Download the VDI for Ubuntu 18.04.3 from https://www.osboxes.org/ubuntu/
- Select "New"
- Select your new VM to be of type Ubuntu-64bit
- Allocate a reasonable amount of memory. 4GB should be sufficient.
- Select the VDI you downloaded under "use an existing virtual hard disk"
- Once the VM's basic setup has completed, go into the VM's settings.
- Under Network, Select Adapter 1. Ensure the network adapter is NAT. Select "Advanced" and ensure "Cable Connected" is selected.
- Click on "Port Forwarding" and add a rule. Name the rule "SSH". Set the protocol to be "TCP". Host IP should be 127.0.0.1, Guest IP should be 10.0.2.15, and Guest Port should be 22.
- The login and root password is `osboxes.org`
- Note- if there's a pop-up saying "system error detected", you can safely ignore it.

### VM Updating
- Start up the VM.
- Because of some updating issues, we want to disable the VM's auto-update feature. So open up Software & Updates, go to the Updates tab, and set "Automatically check for updates" to "Never". Apply and close.
- Open up Terminal.
- Type in `sudo apt update && sudo apt upgrade -y && sudo apt autoremove -y`, then type in the password when prompted.

### Recommended VSCode Forwarding
- To make code editing easier, install VSCode on your computer. Install the Remote-SSH extension.
- Start up the VM.
- Open up Terminal and type `ip a`. Note down the IP address- it is most likely `192.168.56.101`.
- Open up VSCode's command menu. Select "Add New SSH Host". Type in `ssh osboxes@192.168.56.101`.
- Open the command menu again and select "Connect to Host", then select the VM you just added. Select Linux when prompted, then enter the password.

### Software/Library Installation
- `echo alias pip=pip3 >> ~/.bashrc`
- `echo alias python=python3 >> ~/.bashrc`
- `source ~/.bashrc`
- Clone the repo at https://github.com/krishauser/Klampt
- Follow the build instructions at https://github.com/krishauser/Klampt/blob/master/Cpp/docs/Tutorials/Install-Linux.md. Use the "Linux, from source" sections.
- `pip install numpy mido`
- Clone the repo at https://github.com/nikwalia/piano-man
- Clone the repo at https://github.com/krishauser/cs498ir_s2021
- Clone the repo at https://github.com/ros-industrial/universal_robot
- Ensure the above 3 repositories are all in the same directory

## FILES/DIRECTORIES
- `arm.py`- definitions for loading in the robot file
- `piano.py`- definitions for a piano model to load in
- `music_functions.py`- loads in `.mid` files and creates action sequences
- `sr_common-melodic-devel`- contains mesh and URDF definitions, sourced from Shadowhand

## CONTRIBUTORS:
- Nikash Walia
- Ben Schneider
