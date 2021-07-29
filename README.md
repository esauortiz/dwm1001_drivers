# DWM1001 communication through serial API

This repository provides python ROS nodes to both send and receive data through serial port based on [pyserial](https://pypi.org/project/pyserial/) mainly in order to retrieve data from DWM1001 tag. Modules in this repository are based on [DWM1001 UART API](https://www.decawave.com/dwm1001/api/). Python version used is python2.7.

## Installation
Copy this repository:
 
```bash
git clone https://github.com/esauortiz/dwm1001_drivers.git
```

This repository depends on [uwb_msgs](https://github.com/esauortiz/uwb_msgs). It can be downloaded with:
```bash
git clone https://github.com/esauortiz/uwb_msgs.git
```
## Usage

The main module in this package is ```dwm1001_driver_ros.py``` which funcionality retrieving anchor data (position and anchor-tag distance) and publishing them through ```AnchorInfo.msg``` messages (see [uwb_msgs](https://github.com/esauortiz/uwb_msgs)). This main module is configured and launch as a ROS node with:

```bash
roslaunch dwm1001_drivers dwm1001.launch
```

A positioning module ```dwm1001_localization.py``` based on Least Squares (see [UWB-Based Self-Localization Strategies](https://www.mdpi.com/1424-8220/20/19/5613)) is also provided and could be launched as a ROS node with:

```bash
roslaunch dwm1001_drivers dwm1001_localization.launch
```

Lastly, RVIZ configuration file is provided to visualize both Least Squares-based and DWM1001-based tag position. The latter is provided by DWM1001 and computed with the firmware module only if tag is configured with localization engine enabled (```localization_engine: 1```). TFs of the anchors whose data is received through serial port are also represented. RVIZ with this configuration could be launche with:

```bash
roslaunch dwm1001_drivers nodes_viz.launch
```