# GAZEBO Plugin Autonomous Actor

This repository is Gazebo plugin for actor to navigation in simulation environment autonomously.  

## Parameters

ACTOR  
- **pose**: Modify the <pose> element of each actor to change their starting location.

AUTONOMOUS ACTOR PLUGIN
- **targets**: The <targets> element contains multiple <target> elements.
- **target**: Add <target> element for actor to navigate to in sequence.
- **target_tolerance**: Modify <target_tolerance> for invoking next target. Please note that the Gazebo simulator is not very accurate.
