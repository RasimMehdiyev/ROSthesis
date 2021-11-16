# gopigo3_nav_sim

Navigation and simulation extensions to the `gopigo3` ROS package (ROS 1).



## Prerequisites

We assume that you have a Desktop-Full installation of ROS, including the Gazebo simulator and `gazebo_ros_pkgs`, the set of packages for interfacing ROS with Gazebo. If you miss any of these, please read the corresponding tutorials:
- [ROS Installation](http://wiki.ros.org/ROS/Installation)
- [Installing gazebo_ros_pkgs (ROS 1)](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)

The following ROS packages are required to use `gopigo3_nav_sim`:

- `gopigo3` ROS package. Please also download it to your workspace.
It is recommended to use my fork from 
https://github.com/domikiss/gopigo3

  Note that `gopigo3_nav_sim` may also work with the original `gopigo3` package:
https://github.com/ros-gopigo3/gopigo3

- [ROS Navigation Stack](http://wiki.ros.org/navigation) and [Gmapping](http://wiki.ros.org/gmapping). You can install them by

  ```
  sudo apt install ros-<distro>-navigation ros-<distro>-gmapping
  ```

  Replace `<distro>` with the name of your ROS distribution (melodic, noetic, etc.)

- [turtlebot3_gazebo](http://wiki.ros.org/turtlebot3_gazebo) package is requred for some simulated worlds:

  ```
  sudo apt install ros-<distro>-turtlebot3-gazebo
  ```



## Simulation

To simulate the robot in Gazebo, use one of the following roslaunch commands. Note that Gazebo may need longer time (even more minutes) to load a world model first time. Be patient, subsequent loads will be much faster.

```
roslaunch gopigo3_gazebo gopigo3_willowgarage_world.launch
```
![Gazebo simulation of GoPiGo3 in the WillowGarage world](https://domikiss.github.io/gopigo3/gopigo3_gazebo_willowgarage_world.png)


```
roslaunch gopigo3_gazebo gopigo3_turtlebot3_world.launch
```
![Gazebo simulation of GoPiGo3 in the WillowGarage world](https://domikiss.github.io/gopigo3/gopigo3_gazebo_turtlebot3_world.png)

```
roslaunch gopigo3_gazebo gopigo3_turtlebot3_house.launch
```
![Gazebo simulation of GoPiGo3 in the WillowGarage world](https://domikiss.github.io/gopigo3/gopigo3_gazebo_turtlebot3_house.png)



## Mapping

To start a SLAM (Simultaneous Localization and Mapping) course run the following launch file in a separate terminal. You can use this feature alongside with Gazebo simulation (see previous section) or a real GoPiGo3 instance. 
```
roslaunch gopigo3_slam gopigo3_slam.launch
```
This will use Gmapping to build a map while the robot is moving around. RViz is also opened to show the map under construction:

![GoPiGo3 mapping](https://domikiss.github.io/gopigo3/gopigo3_slam_turtlebot3_world.png)


You have to publish velocity commands to the `/cmd_vel` topic in order to drive the robot. A possible tool to do this is `rqt`. In the `rqt` window choose Plugins > Robot Tools > Robot Steering. Drive around until you have a reasonable map:

![GoPiGo3 mapping ready](https://domikiss.github.io/gopigo3/gopigo3_slam_turtlebot3_world_ready.png)

In a new terminal you can run `map_saver` to save the map:
```
rosrun map_server map_saver -f /tmp/test_map
```
This will create a `test_map.pgm` and a `test_map.yaml` file in the `/tmp` directory. You can specify the required location of your map by changing the path `/tmp/test_map` to any conventient place. Note that a map of the `turtlebot3_world` is already available in this repository under `gopigo3_navigation/maps`.



## Navigation in a Known Environment

Close all previous terminals you opened for simulation and mapping. We will use the created map for localization and autonomous navigation. To do this, we have to load the map, specify the initial pose of the robot and select navigation goal(s). We will show an example with a simulated robot, but this should work similarly with a real robot.

First launch the simulation using the world in which you built the map:
```
roslaunch gopigo3_gazebo gopigo3_turtlebot3_world.launch
```

Note that you may encounter performance issues when running Gazebo simulation and navigation concurrently (this is usual on middle-class notebook computers). If you experience performance degradation, it is recommended to run the simulation without graphical user interface by setting the parameter `gui:=false`:
```
roslaunch gopigo3_gazebo gopigo3_turtlebot3_world.launch gui:=false
```

In another terminal, start navigation and specify the requested map as a parameter:
```
roslaunch gopigo3_navigation gopigo3_navigation.launch map_file:=/tmp/test_map.yaml
```
Note that if no map file is provided, the `turtlebot3_map` will be loaded from `gopigo3_navigation/maps` by default. This launch file will open RViz and show the map and the robot in a default pose:

![GoPiGo3 navigation: map loaded](https://domikiss.github.io/gopigo3/gopigo3_navigation_turtlebot3_world_1.png)

In RViz, an initial pose estimate should be given: click `2D Pose Estimate`, then click the approximate location of the robot on the map, and drag to indicate the orientation. If a good estimate is given, the laser points (green dots) are mostly aligned with the obstacles in the map:

![GoPiGo3 navigation: initial pose estimated](https://domikiss.github.io/gopigo3/gopigo3_navigation_turtlebot3_world_2.png)

In RViz, publish a navigation goal (click on `2D Nav Goal` and click+drag similarly to what you have done with `2D Pose Estimate`). The robot starts navigating autonomously to the goal. You can also examine the localization process during motion: the estimated positions (small green arrows) should converge on the true pose while the robot moves.

![GoPiGo3 navigation: moving](https://domikiss.github.io/gopigo3/gopigo3_navigation_turtlebot3_world_3.png)



## Navigation in an Unknown Environment

It is not necessary to divide mapping and navigation into two separate phases. These can be performed simultaneously if the map is provided by a SLAM process. This will make the navigation process use the map just under construction. The mapping is also more user friendly this way because no manual teleoperation is needed. Run the following launch file (we assume that a simulation or a real robot is already running):
```
roslaunch gopigo3_navigation gopigo3_slam_navigation.launch
```
In RViz you should simply place a `2D Nav Goal` anywhere on the map, and the robot starts moving there while revealing the unknown parts of the map at the same time.

![GoPiGo3 SLAM navigation](https://domikiss.github.io/gopigo3/gopigo3_slam_navigation_turtlebot3_world.png)

