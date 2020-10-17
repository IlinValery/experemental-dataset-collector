# Dataset collector for Raspberry Pi 3 model B+

## Description

## Requirement

ROS Melodic has to be be installed.

## Launching scripts

### Installation

```bash
cd ~/catkin_ws/src
git clone https://gitlab.com/isr-lab-mobile-group/warevision_dataset_collector
catkin_make
source devel/setup.bash
```

### Launching of the package

After enviroment activation use next script (to launch dataset collector to bag file):

```bash
rolaunch warevision_dataset_collector collector.launch
```
