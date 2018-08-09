# youBot Nodes

Some custom nodes to manipulate the youBot. Running in Ubuntu 14.04, ROS Indigo.

### my_youbot_ik

A node that uses inverse kinematics to move the youBot to a desired pose (position + orientation).

Valid inputs include:

 * `x y z r p y`: Position (`x`, `y`, `z`) and orientation in roll, pitch, yaw format (`r`, `p`, `y`).
 * `x1 y1 z1 x2 y2 z2 w2`: Position (`x1`, `y1`, `z1`) and orientation in quaternion format (`x2`, `y2`, `z2`, `w2`).

### my_pick_place (WIP)

A node that introduces an object to the planning scene, picks up an object, or places an object at a desired location.

Currently, valid inputs include:

 * `add name`: Introduces a box with id `name` to the planning scene. Once called, subsequent inputs will be prompted for the dimension and pose of the box. Dimension should be inputted in the format `x y z`, and pose should be inputted in the format `x y z r p y` or `x1 y1 z1 x2 y2 z2 w2`, explained above.

### my_hello_world

A simple hello world node that moves the youBot in extended (candle) position, then moves it into folded position.

### RPY_Solver

An executable that takes in any number of x, y, z axis rotations and outputs equivalent orientations in roll, pitch, yaw format. The RPY format is not unique; in fact, it seems like there are always two equivalent RPY solutions representing the same orientation.

Valid inputs include:

 * `axis amount`: Rotates current orientation by `amount` in the `axis` axis. `amount` should be in degrees, and valid inputs for `axis` are `x`, `y`, and `z`.
 * `Solve`: Outputs equivalent orientations in RPY format. `solve` and `s` are also allowed.
 * `Reset`: Resets to base orientation. `reset` and `r` are also allowed.

### test

A node that has multiple miscellaneous methods. Make sure the correct planning group is uncommented when using different methods. The functionality of each method is documented at the top of each block comment.

# Dependencies

These nodes require several projects to work properly:

 * [MoveIt!](http://moveit.ros.org/install/source/)
 * [youbot-manipulation](https://github.com/svenschneider/youbot-manipulation) (branch: indigo)
 * [youbot_description](https://github.com/youbot/youbot_description) (branch: indigo-devel)

Additionally, when running on a real youBot along with simulation:

 * [youbot_driver_ros_interface](https://github.com/youbot/youbot_driver_ros_interface) (branch: indigo-devel)

# Installation

Go to your catkin workspace or create one if you haven't done so.

```
cd src
git clone https://github.com/kevinchoik/youbot_nodes.git
cd ..
catkin build
```

Always remember to source `setup.bash` of your workspace. It is convenient if this command is included in `~/.bashrc`:

`echo "source [path_to_your_catkin_workspace]/devel/setup.bash" >> ~/.bashrc`

# Running

If running on the actual youBot, launch the ROS wrapper for the desired configuration of the youBot (in this example, I use the single arm, no base version):

`roslaunch youbot_driver_ros_interface youbot_driver_arm_only.launch`

Regardless of running on the actual youBot or just in simulation, a simulated youBot is required. In one terminal window, load a world in RViz that contains at least the youBot. An empty world with just the youBot can be launched with:

`roslaunch youbot_moveit demo.launch`

Note that the `demo.launch` file from the `youbot-manipulation` package spawns both the youBot arm and the base. If working only with the arm, the following files should be edited:
 * `youbot-manipulation/youbot_moveit/config/youbot.srdf`: Delete the lines of code that deal with the links that are in the base.
 * `youbot-manipulation/youbot_moveit/launch/planning_context.launch`: Load `youbot_arm_only.urdf.xacro` instead of `youbot.urdf.xacro`.
 * `youbot-manipulation/youbot_moveit/launch/moveit.rviz`: Change fixed frame (line 651) and target frame (line 670) from `/base_footprint` to `/base_link`.

Alternatively, copy the five files in `arm_only_files` from this repo (`youbot_nodes`) accordingly. The folder structure should match that of `youbot-manipulation`, so simply merging the two folders should suffice. Then, launch:

`roslaunch youbot_moveit youbot_arm_only.launch`

When RViz successfully loads the world, in one terminal window per node, run one or several of:

```
rosrun my_youbot_ik my_youbot_ik_node
rosrun my_pick_place my_pick_place_node
rosrun my_hello_world my_hello_world_node
rosrun test test_node 
```