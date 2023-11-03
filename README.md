# RossenheimerMajor
_For I am become turtle [DTor]: of fire and flood._

To run this TurtleBot simulation the following steps must be taken.

For our project, we used the plywood maze library found [here](https://github.com/rfzeg/plywood_mazes), for which the majority of our turtlebot testing was done on.

Using this, a new terminal should be opened and the following executed 
```
roslaunch plywood_mazes maze_1_6x5.launch
```
This will open an empty maze world.
A new terminal should be opened and execute 
```
roslaunch plywood_mazes spawn_turtlebot3.launch
```
This will put a turtlebot3 into the current gazebo world.
Following this to start simulation
```
roslaunch Rossenheimer ken_navigation.launch
```
This will launch the simulation for the turtlebot and also open RVIZ to view frontiers and goals.
