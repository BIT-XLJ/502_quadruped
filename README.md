## Installation

### OCS2

OCS2 is a huge monorepo; **DO NOT** try to compile the whole repo. You only need to compile `ocs2_legged_robot_ros` and
its dependencies following the step below.

1. You are supposed to clone the OCS2, pinocchio, and hpp-fcl as described in the documentation of OCS2.

   ```
   # Clone OCS2
   git clone git@github.com:leggedrobotics/ocs2.git
   # Clone pinocchio
   git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git
   # Clone hpp-fcl
   git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git
   # Clone ocs2_robotic_assets
   git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git
   # Install dependencies
   sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev
   ```
2. Compile the `ocs2_legged_robot_ros` package with [catkin tools](https://catkin-tools.readthedocs.io/en/latest/)
   instead of `catkin_make`. It will take you about ten minutes.

   ```
   catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
   catkin build ocs2_legged_robot_ros ocs2_self_collision_visualization
   ```

   Ensure you can command the ANYmal as shown in
   the [document](https://leggedrobotics.github.io/ocs2/robotic_examples.html#legged-robot) and below.
   ![](https://leggedrobotics.github.io/ocs2/_images/legged_robot.gif)

### Build

Build the source code of `legged_control` by:

```
catkin build legged_controllers legged_unitree_description
```

Build the simulation

```
catkin build legged_gazebo
```

## Quick Start

1. Set your robot type as an environment variable:

   ```
   export ROBOT_TYPE=quake_without_arm
   source devel/setup.bash
   ```
2. Run the simulation:

   ```
   roslaunch legged_controllers one_start_gazebo.launch
   ```
3. load the controller:

   ```
    roslaunch legged_controllers load_controller.launch 
   ```
4. start the controller:

   ```
   rostopic pub --once /load_controller std_msgs/Float32 "data: 1.23"
   ```
5. Set the gait in the terminal of **`load_controller.launch`**

   enter "list" in the terminal, then enter "trot"
6. Open another terminal, then enter **rqt**
7. Find the Robot Steering plugin and control the velocity of the robot in simulaiton. Note the topic name is **`/cmd_vel`**
