# moveit2_scripts

---

This package is a ROS2 / MoveIt2 package, 
which regroup a bunch of scripts that perform basic motion using MoveIt2 and ROS2.

---

### Pick and place

To try out the pick and place algorithms, use the following command:  

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
  `ros2 launch moveit2_scripts pick_and_place_real.launch.py` 

### Perception   
 
To test the pick and place with perception scripts, use the following command:  

**Simulation :**  
  1- launch move_group + rviz + simple_grasping node     
  `ros2 launch moveit2_scripts setup.launch.py`     

  2- launch the pick and place algorithm  
  `ros2 launch moveit2_scripts pick_and_place_perception.launch.py`  

  
**Real robot :**    
  1- launch move_group + rviz + simple_grasping node  
  `ros2 launch moveit2_scripts setup_real.launch.py`     
  
  2- launch the pick and place algorithm    
  `ros2 launch moveit2_scripts pick_and_place_perception_real.launch.py` 