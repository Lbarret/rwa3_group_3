# rwa3_group_3

To run:

Create a catkin workspace as described in https://github.com/usnistgov/ARIAC/blob/ariac2020/wiki/tutorials/installation.md

Go to the src directory and clone the repository:

cd ~/ariac_ws/src

git clone https://github.com/Lbarret/rwa3_group_3

Build the workspace:

catkin build

Source the workspace:

cd ~/ariac_ws
source devel/setup.bash

In one window, run the launch file:

roslaunch rwa3_group3 rwa3.launch

In another window, run the program:

rosrun rwa3_group3 rwa3_node
