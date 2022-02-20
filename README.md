# Control of Mobile Robots
Control of Mobile Robots course project 2021/2022 - Politecnico di Milano.

- Insert the ROS packages car_traj_ctrl for the Bicycle Kinematic Model and car_simulator for the Single-Track Dynamic Model by typing:
```
  $ mv car_traj_ctrl <path>/catkin_ws/src
  $ mv car_simulator <path>/catkin_ws/src
```
- Go to your own \texttt{catkin\_ws} folder in the system and recompile everything by doing:
```
  $ cd <path>/catkin_ws
  $ source ./devel/setup.bash
  $ catkin_make
```
- Then run `ROS core` by typing:
```
  $ roscore
```

## Bicycle Kinematic Model – `car_traj_ctrl` package
<img src="/imgs/bicycle_kinematic.png" alt="drawing" width="400"/>

- Open a new terminal and record the robot activity through a rosbag in this manner by reading the topic `car_state`:
```
  $ cd <path>/catkin_ws/src/car_traj_ctrl/script
  $ rosbag record -O test_eight /car_state
```
- Open a new terminal and launch the robot simulation as:
```
  $ cd <path>/catkin_ws
  $ source ./devel/setup.bash
  $ roslaunch car_traj_ctrl test_car_eight.launch
```
- Go back to the terminal you ran `rosbag` and press `ctrl + c`
- Do the same in the terminal which is running the robot simulation
- Go back to the terminal where you stopped `rosbag` and run the following python script by typing on the bash:
```
  $ python plot_result.py test_eight.bag 
```
- Now you can see the result produced by the execution you recorded.
    
## Single-Track Dynamic Model – `car_simulator` package
<img src="/imgs/bicycle_dynamic.png" alt="drawing" width="800"/>

- Open a new terminal and record the robot activity through a rosbag in this manner by reading the topic `car_state`:
```
  $ cd <path>/catkin_ws/src/car_simulator/script
  $ rosbag record -O test_simple /car_state
```
- Open a new terminal and launch the robot simulation as:
```
  $ cd <path>/catkin_ws
  $ source ./devel/setup.bash
  $ roslaunch car_simulator test_car_simple.launch
```
- Go back to the terminal you ran `rosbag` and press `ctrl + c`
- Do the same in the terminal which is running the robot simulation
- Go back to the terminal where you stopped `rosbag` and run the following python script by typing on the bash:
```
  $ python plot_result.py test_simple.bag 
```
- Now you can see the result produced by the execution you recorded.
