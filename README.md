# Group 3 ARIAC Project
Finding, picking and placing different types of parts within an industrial environment, while taking different agility challenges into account.
Agility challenges include:
1. Faulty Parts

2. Flipped Parts

3. Faulty Gripper

4. High-Priority Order

5. Moving Obstacles
 
6. Sensor Blackout

## Team Members:
1. Loic Barret
2. Eitan Griboff
3. Santosh Kesani
4. Moumita Paul

## To Run the Package:

1. Create a catkin workspace as described in https://github.com/usnistgov/ARIAC/blob/ariac2020/wiki/tutorials/installation.md

2. Go to the src directory and clone the repository:

   - `cd ~/ariac_ws/src`

   - `git clone https://github.com/Lbarret/rwa3_group_3`

3. Build the workspace:

   - `catkin build`

4. Source the workspace:

   - `cd ~/ariac_ws`
   
   - `source devel/setup.bash`

5. In one terminal, run the launch file:

   - `roslaunch rwa3_group3 rwa3.launch`

6. In another terminal, run the program:

   - `rosrun rwa3_group3 rwa3_node`
