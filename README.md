# robotics_big_proj
## launch
To launch the world:
1. place the `big_world_pkg/` directory in your catkin workspace `src/` directory.
2. run `source ~catkin_ws/devel/setup.bash`
3. run `catkin_make` in the `catkin_ws/` directory.
4. Now you should be able to run `roslaunch big_project_pkg big_project.launch` and it will launch the world in gazebo.  
  
I also added the code from project 1 so that should be runnable.

The two programs for the big project are in `src/`  
  
`summon.py` is the summon function and can be ran using `rosrun big_project_pkg summon.py`  
  
`landmarkID.py` is the landmark identification function and it can be ran using `rosrun big_project_pkg landmarkID.py`
