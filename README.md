# IK-Model-Development
Matlab scripts for modeling Inverse Kinematics

Inverse Kinematics for a 6 Degree of Freedom All revolute arm  Robotic Arm with a Spherical Wrist

The IK problem is easier to solve by splitting the six joint system into two independent systems of three joints each

This decouples the 'Inverse Position' and 'Inverse Orientation' problems as joints 1 and 2 are orthogonal

A Spherical wrist is when the axes of rotation of the final 3 joints (J4, J5, and J6) all intersect in a single point.  

The three Spherical wrist joints (J4, J5, and J6) all intersect in a single point aligned with the Rover-Coordinate-System (RCS) coordinate frame 

The remaining three Arm joints (J1, J2, and J3) are only responsible for the location of the end-effector using the Denavit-Hartenberg (DH) coordinate frame

Joints 1 and 2 must be orthogonal

There are 2 strict rules for assigning coordinate frames with the DH convention:

1) Axis Xi is perpendicular to axis Zi-1
2) Axis Xi intersects axis Zi-1

3) If Joint i is revolute:

    Axis Zi-1 aligns with the axis of rotation for Joint i (θi)
	
  Else if Joint i is prismatic:

    Axis  Zi-1 aligns with the direction of translation of Joint i (θi)

It is preferred but not strictly necessary and sometimes not even possible, that axis Xi is in the direction of Link i.  
 
Input the link lengths and offsets using DH convention and input your desired position and orientation of the end effector 

The system is solved for the elbow up, forward reach, arm configuration 
