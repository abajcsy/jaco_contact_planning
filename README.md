# jaco_contact_planning
Contact planning functionality for the Jaco2 7DOF robot

## Repository Organization
This code is written in the Robot Operating System (ROS) framework. The ros/ directory is the root workspace, and packages live inside the ros/src/ directory.

# Dependencies 
* [SNOPT](https://ccom.ucsd.edu/~optimizers/solvers/snopt/)
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page#Download)
* [kdl_parser](http://wiki.ros.org/kdl_parser/Tutorials/Start%20using%20the%20KDL%20parser)

# Setup
## SNOPT
Request a [SNOPT license](https://ccom.ucsd.edu/~optimizers/downloads/) and follow the website instructions for [setting up the license](https://ccom.ucsd.edu/~optimizers/faq/). 

Set the SNOPT environment variable to point to the SNOPT C++ interface:
```
export SNOPT=$HOME/jaco_contact_planning/ros/src/snopt-interface-2.1.1
````
Add this line to your ```.bashrc``` for this to be set in every terminal. 

## ROS
Install the kdl_parser that allows us to read the robot's urdf by running:
```
rosdep install kdl_parser
rosmake kdl_parser
```
