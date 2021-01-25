# GAZEBO Plugin Autonomous Actor

This repository is Gazebo plugin for actor to navigation in simulation environment autonomously.  
Please refer for more usage of actor in Gazebo at this [repository](https://github.com/BruceChanJianLe/gazebo-actor)  

CURRENTLY, THIS PACKAGE IS STILL UNDER DEVELOPMENT. IT IS NOT YET FUNCTIONAL.

## Installation

This code may be treated as a ROS package. Therefore, it can be built with `catkin_make` or `catkin build`.

```bash
mkdir -p catkin_ws/src
cd catkin/src
git clone https://github.com/BruceChanJianLe/gazebo-plugin-autonomous-actor.git
cd ..
catkin_make
```

## Usage

Add your plugin in your sdf world file.  

```xml
<actor name="actor0">
    <pose>0 0 0 0 0 0</pose>
    <skin>
    <filename>walk.dae</filename>
    <scale>1.0</scale>
    </skin>
    <animation name="walking">
    <filename>walk.dae</filename>
    <scale>1.000000</scale>
    <interpolate_x>true</interpolate_x>
    </animation>

    <plugin name="actor0_plugin" filename="libAutonomousActorPlugin.so">
    <target_weight>1.15</target_weight>
    <obstacle_weight>1.8</obstacle_weight>
    <animation_factor>5.1</animation_factor>
    <!-- Usage: Modify the set of models that the vector field should
            ignore when moving the actor -->
    <ignore_obstacles>
        <model>cafe</model>
        <model>ground_plane</model>
    </ignore_obstacles>
    <targets>
        <target>0 -5 0</target>
        <target>0 0 0</target>
        <target>-3 -4 0</target>
    </targets>
    </plugin>
</actor>
```

## Parameters

ACTOR  
- **pose**: Modify the <pose> element of each actor to change their starting location.

AUTONOMOUS ACTOR PLUGIN
- **targets**: The <targets> element contains multiple <target> elements.
- **target**: Add <target> element for actor to navigate to in sequence.
- **target_tolerance**: Modify <target_tolerance> for invoking next target. Please note that the Gazebo simulator is not very accurate.


## Models

The models folder is obtained from [servicesim_competition](https://github.com/osrf/servicesim/tree/master/servicesim_competition)

## More about Gazebo plugin

![image](https://raw.githubusercontent.com/osrf/gazebo_tutorials/master/install_dependencies_from_source/files/gazebo_dependency_tree.svg)

## Reference

- Reference plugin repo [link1](https://github.com/osrf/gazebo/tree/gazebo9/plugins)
- Gazebo debug msg [link1](https://answers.gazebosim.org//question/17428/how-print-the-output-of-a-plugin/)
- Gazebo dependencies [link1](http://gazebosim.org/tutorials?tut=install_dependencies_from_source)
