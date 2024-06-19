# moveit2_scripts

---

This package is a ROS2 / MoveIt2 package, 
which regroup a bunch of scripts that perform basic motion using MoveIt2 and ROS2.

---

### Pick and place

To try out the pick and place algorithm, use the following command:
**Simulation :**
 1- launch the move_group node
  `ros2 launch my_moveit_config move_group.launch.py`  

 2- (optional) launch Rviz
  `ros2 launch my_moveit_config moveit_rviz.launch.py`

  3- launch the pick and place algorithm
  `ros2 launch moveit2_scripts pick_and_place.launch.py`

**Real robot :**
 1- launch the move_group node
  `ros2 launch real_moveit_config move_group.launch.py`  

 2- (optional) launch Rviz
  `ros2 launch real_moveit_config moveit_rviz.launch.py`

  3- launch the pick and place algorithm
  `ros2 launch moveit2_scripts pick_and_place_real.launch.py