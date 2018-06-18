# Theseus
Theseus is a path planner for the AUVSI competition. It plans a path for aircraft to obtain waypoints and other missions while avoiding stationary obstacles boundaries. Theseus also has a ground station for viewing the paths planned. All competition elements are plotted in 3d - the obstacles, boundaries, current position of the UAV, past positions of the UAV, paths sent to the plane, proposed paths, waypoints, loiters at the end of a mission, and a satellite image of Webster field.

## Getting Started
In order to use all of the repository, the following repositories are needed:
* ROSplane (needs rosflight, rosflight_plugins and WiringPi steps 1 and 2 [here] (https://wiki.odroid.com/accessory/development/c_tinkering/c_tinkering#linux)
* uav_msgs
* interop_pkg (which needs vision_pkg) and the judges server (explained in the interop_pkg readme)
* rviz_satellite
* ros_groundstation (This has tons of stuff in it, but the only thing used for 2018 was mission_controller.py)

Create a catkin workspace if you haven't already
```
mkdir -p ws/src
cd ws
catkin_make
```

Download the repositories.
```
cd src
git clone https://github.com/BYU-AUVSI/theseus.git
git clone https://github.com/BYU-AUVSI/rosplane.git
git clone --recurse-submodules https://github.com/BYU-AUVSI/rosflight.git
git clone https://github.com/BYU-AUVSI/rosflight_plugins.git
git clone https://github.com/BYU-AUVSI/uav_msgs.git
git clone https://github.com/BYU-AUVSI/interop_pkg.git
git clone https://github.com/BYU-AUVSI/rviz_satellite.git
git clone https://github.com/BYU-AUVSI/ros_groundstation.git
git clone https://github.com/BYU-AUVSI/vision_pkg.git

git clone https://github.com/hardkernel/wiringPi
cd wiringPi
./build
cd ../..
```
Then build the workspace
```
catkin_make
```
That build took forever! You mine as well download the firmware and inertial_sense_ros repos from the BYU-AUVSI site as well if you will be working with the NAZE or INS.

## Using the Repository
The repository is used by launching the following launch file:
```
roslaunch theseus rospathplanner.launch
```
The ground station can also be launched:
```
roslaunch theseus groundstation.launch
```

If you want to send waypoints to the plane you will need to either launch the code on the plane or the simulator:
```
roslaunch rosplane (insert your launch file here competition used Official.launch)
```
or
```
roslaunch rosplane_sim fixedwing.launch
```

If you want to talk to the judges server, and receive mission objectives you will need to have the docker running and launch the client
```
rosrun interop_pkg client.py
```

The repository can be tested without the other programs running.
Make sure the the launch file initializes the position of the UAV if there is nothing on the /state topic.
```
<param name="testing/init_references" value="true"/>
```

The mission_controller GUI is intuitive. There are multiple services (listed at the beginning of path_planner_base.cpp) that can be called from the command line that don't require the GUI to be running. They will look for text files in your .ros folder.
* bomb.txt
* landing.txt
* path.txt

These files can contain mission information. path_planner_base.cpp is the interface between the path planner (RRT.cpp) and the user.
### Example rosservice call
```
rosservice call /theseus/add_wps
```
That command has theseus look at path.txt and plan a path from the initial state of the UAV to each waypoint listed in that text file.
```
rosservice call /theseus/send_waypoints
```
Then the path (all of the waypoints) are sent to the path_manager in ROSplane.
Another mission can be planned after that:
```
rosservice call /theseus/add_bomb
```
At any time during the flight, the missions can be abandoned in favor of a new mission
```
rosservice call /theseus/land_now
```
In the mission controller 'Generate path' plans the mission starting from the last mission ending point. 'Execute path' sends the waypoints to the UAV. Pushing 'Generate and Execute' causes the path planner to plan a path from the current position and tells the UAV to forget the previous missions and execute this new one right away.

The RRT planner plans out fillet paths, as described in Small Unmanned Aircraft: THeory and Practice (Beard and McLain).
The waypoint mission causes the path to extend beyond the waypoint such that the path's line hits the waypoint directly.
This was designed so that the waypoint miss distance was low and that the curvature between path segments are accounted.
The other missions do not behave this way, the corners are filleted like normal.


## Known Shortcomings
Planning to drop a bomb mission from a take-off position (initial position near the ground) sometimes creates an unusual path. If it has a mission prior to dropping the bomb there seems to be no problem.
RRT is fast and it is udsually calculated multiple times and the shortest path is chosen. This could be done by increasing num_paths_ in RRT.cpp. Right now that causes a seg fault. Somehow the tree might not be setup right to keep track of all the memory.
If the path (or boundary) goes perfectly north, east, or up, then the collisions will not be detected. This is a shortcoming in how collisions with cylinders and boundaries are detected.
If there is a problem in solving the path it is probably the smoother. If it cannot solve the path, it shouldn't crash, but it will end the algorithm. Often trying again works or changing the random seed in path_planning.yaml will work.


## Future Considerations
The code could be easily adapted for multi-rotors. Perfectly vertical movements might cause some issues in the collision detector. The max_climb_angle and max_decent_angle could be increased dramatically. The turn_radius could decrease to a very small value (a radius of 0 might cause some issues), or the code could be reworked to plan line paths instead of fillets.
The AUVSI judges absolutely loved the 3D ground station. It was a huge hit. They would like the waypoints numbered. I think it would be also helpful to display one mission path at a time. The plot gets really crowded after 25 minutes of missions.


Good luck, feel free to contact me, If you are developing it, make sure to use the tools that visualize the path as it being created. Set animating_ in RRT.cpp to true. See what functions that variable allows to be called, they are pretty helpful in developing capabilities.
Austin Hurst
