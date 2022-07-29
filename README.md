# Cart-Pushing-with-an-Autonomous-Robot
RBE 550 Project with an autonomous PR2 robot intended for cart pushing

The ROS directory used a PR2 model and some scripts and launch files from a project from The Construct that can be fully downloaded from this link: https://app.theconstructsim.com/#/Rosject/544128. This directory is not needed to run our ROS directory.

In order to run this ROS directory the requirements are being on Linux 20.04 and having ROS Noetic.
For instructions on how to install ROS Noetic and setup a workspace directory, consult this link:
http://wiki.ros.org/noetic/Installation/Ubuntu

Once a ROS workspace has been created, download this Github repository with whatever method is desired and follow the following steps to be able to run Gazebo simulations:

1. After the repository has been downloadeded and extracted, copy/move the pr2_tc_description folder to the src folder in the ROS workspace folder (usually "catkin_ws/src").

3. Navigate to the scripts folder within a terminal or open a terminal on this folder if using the gui ("pr2_tc_description/scripts").

4. Make the following files executable using the command "chmod +x file_name.py" like so:
```
chmod +x controller.py 
```
```
chmod +x pr2_teleop_bridge.py
```
```
chmod +x D_star-Comparison/dstar_main.py 
```
```
chmod +x RRT+Replanning+RRN/rrt_main.py
```
  
4. Navigate to the ROS workspace folder (usually "catkin_ws") in the terminal and run the following commands to build and source the directory:
```
catkin_make
```
```
source devel/setup.bash
```
  
5. Now the setup has been completed and to run the gazebo simulation at least four terminals need to be open.

6. On the first three terminals execute the following commands:
```
roscore
```
```
roslaunch pr2_tc_description main.launch
```
```
rosrun pr2_tc_description controller.py
```
   
7. All that's left is to navigate to "catkin_ws/src/pr2_tc_description/scripts" on the fourth terminal and choose whether to plan with Modified MRRT* or D*. For Modified MRRT* execute the following command:
```
python3 RRT+Replanning+RRN/rrt_main.py
```
For D* execute the following command:
```
python3 D_star-Comparison/dstar_main.py
```

8. To run a new simulation, terminate all terminal commands, except the one runnning roscore, by using the 'Ctrl+C' keys on each terminal. Then repeat steps 6 and 7.

To run the motion planners in their 2D form, simply navigate to the other downloaded folders on a terminal: to run the Modified MRRT* execute "python3 main.py" on the RRT+Replanning+RRN folder and to run the D* execute "python3 main.py" on the D_star-Comparison folder.
