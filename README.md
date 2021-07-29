# My Practice catkin workspaces
This repo consists of four main catkin workspaces that I created and used to learn ROS and Moveit
## Content description
* maze-solver_ws - Catkin workspace created for maze solver robot
* moveit_ws - Catkin workspace created to learn moveit
* my_maze_models - maze models created to insert in Gazebo
* refr_learns - Catkin workspace created to implement my ROS learnings
* wanderbot_ws - Catkin workspace created for obstracle avoidance robot
* ros_with_cpp - Catkin workspace created to practice code on C++
* cpp_revisit_ws - Catkin workspace created to practice C++ from different material
* robot_modeling_ws - Catkin workspace created to learn about 3D modeling in ROS
## Seven DOF manipulator
A custom made seven degrees of freedom manipulator using [freeCAD](https://www.freecadweb.org/) and [MeshLab](https://www.meshlab.net/#description) whose moveit configuration package is available in [robot_modeling_ws](./robot_modeling_ws) workspace under the name of [penguin_moveit_config](./robot_modeling_ws/src/penguin_moveit_config). Mesh files required for the package is available in [robot_description_pkg](./robot_modeling_ws/src/robot_description_pkg) of the same workspace.

The final result of the arm is shown in the below video.

https://user-images.githubusercontent.com/85285960/125417504-7cc05450-85c0-4f32-a732-9b1896419d2a.mp4

### Pick and place operation
In this program two tables (table1, table2) and an object on table1 is spawned. Objective of the program is to pick the object from table1 and place it on table 2. [pick_and_place.cpp](./robot_modeling_ws/src/moveit_revisit_pkg/src/pick_and_place.cpp) under [moveit_revisit_pkg](./robot_modeling_ws/src/moveit_revisit_pkg) package of [robot_modeling_ws](./robot_modeling_ws) workspace is the code written for this operation. There are a total of six parameters taken in at the command line which are,
1) table_dist - Table spawning distance from origin (default: 0.4 m)
2) table_height - Height of the table (default: 0.2 m)
3) object_height - Height of the object (default: 0.06 m)
4) object_width - Width of the object (default: 0.03 m)
5) eef_offset - End effector offset value (default: 0.145 m)
6) extra_offset - Extra offset value (default: 0.01 m)

The final result of the pick and place operation is shown in the below video.

https://user-images.githubusercontent.com/85285960/127445042-61dcc24c-ad8f-44c9-adaf-8e5a41e5d1ac.mp4
