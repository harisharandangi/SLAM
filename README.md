Installation & Setup
Clone the package

cd ~/catkin_ws/src
git clone https://github.com/your-username/pathplanner.git
Build the workspace

cd ~/catkin_ws
catkin_make
Source the setup file

source devel/setup.bash
Run the Simulation
A launch file named full_stack.launch is provided, which integrates:

earth_world.launch
global_planner.launch
local_planner.launch
static_tf.launch

To start the full simulation:

roslaunch pathplanner full_stack.launch
This will open both the Gazebo simulator and RViz viewer.

Navigation Instructions
In RViz:

Use the 2D Nav Goal tool.
Click and drag to set the robot's destination.
The robot will autonomously plan and follow a path to the goal.
Done!
