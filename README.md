# Trajectory-Planning-for-a-Quadcopter
This project is a graded part of the module 'Robotic Control Theory and Systems' COMP0128 at [UCL London](https://www.ucl.ac.uk/). The goal of this project is to implement a feedback controller that makes the drone perform the following trajectory:
* Starts at (0,0,0)
* Moves up to (5,5,5)
* Stays at (5,5,5) for 5 seconds.
* Moves along a circular trajectory with radius 2.5, with constant altitude z=5, passing through the
points (2.5,2.5,5), (0,5,5), (2.5,7.5,5) and back to (5,5,5).
* Lands at (5,5,0) safely (less than 0.1 m/s linear velocity when hitting the ground).

The task description is detailed [here in Question 3a).](Task_Description.pdf)

## How to run the package
Open the file [quadcopter_script.m](Implementation/quadcopter_script.m) in MATLAB and run it with additional installation of necessary toolboxes.

## Result
The trajectory of the drone is shown below:
[](Results/q3_fig1_quadcopter%20trajectory.png)

## License 
[MIT](LICENSE)