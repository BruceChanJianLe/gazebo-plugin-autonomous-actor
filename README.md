# GAZEBO Plugin Autonomous Actor

This repository is Gazebo plugin for actor to navigation in simulation environment autonomously.  

## Installation

This code may be treated as a ROS package. Therefore, it can be built with `catkin_make` or `catkin build`.

```bash
mkdir -p catkin_ws/src
cd catkin/src
git clone https://github.com/BruceChanJianLe/gazebo-plugin-autonomous-actor.git
cd ..
catkin_make
```

## Parameters

ACTOR  
- **pose**: Modify the <pose> element of each actor to change their starting location.

AUTONOMOUS ACTOR PLUGIN
- **targets**: The <targets> element contains multiple <target> elements.
- **target**: Add <target> element for actor to navigate to in sequence.
- **target_tolerance**: Modify <target_tolerance> for invoking next target. Please note that the Gazebo simulator is not very accurate.
