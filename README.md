# Webots URDF [![Build Status](https://travis-ci.org/omichel/urdf2webots.svg?branch=master)](https://travis-ci.org/omichel/urdf2webots)

This tool converts URDF files into Webots PROTO files, and vice-versa. The code is distributed under the Apache 2.0 License and has been forked from: https://github.com/omichel/urdf2webots


## Install

```
git clone https://github.com/robotlearn/webots_urdf
cd webots_urdf
pip install -r requirements.txt
```

## Usage

`python urdf2webots.py --input=someRobot.urdf [--output=outputFile] [--box-collision] [--normal]`

Outputs: someRobot_textures (folder), someRobot.proto.  
Use in Webots: put the outputs in the protos folder of your Webots project.

## Notes
This tool have been tested with ur5, pr2, motoman and kinova using Webots R2019a on Ubuntu16.04.  
You can find the sources of these URDF files here:  
  - universal robot: https://github.com/ros-industrial/universal_robot/tree/kinetic-devel/ur_description  
  - pr2 robot: https://github.com/PR2/pr2_common/tree/kinetic-devel/pr2_description  
  - motoman robot: https://github.com/ros-industrial/motoman/tree/kinetic-devel/motoman_sia20d_support
  - kinova robot: https://github.com/Kinovarobotics/kinova-ros/tree/kinetic/kinova_description


## TODO

- [ ] comment the code and make it more pythonic
- [ ] parse proto file and write urdf file
- [ ] test proto2urdf parser
