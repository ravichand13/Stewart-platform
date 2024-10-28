# Stewart-platform
code for 6 DOF hexapod Stewart platform using linear actuators

The all-code file combines the calibration, forward kinematics, and inverse kinematics code.

The calibration is used to find home points and the length of actuators.
The forward kinematics is to find the angles for the given lengths of the actuators, by comparing them with the home points.
The inverse kinematics is to verify the code, by providing the angles will will get the lengths of actuators and we can verify them.
