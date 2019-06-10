
TODO
===

- Add a conversion from quaternion to rotation matrix.

LOGIC
===

1. Subscribe to pose estimation and robot pose stamped
2. Grip the part as desired, and move into camera FOV.
3. Do a pose estimation
4. If the pose estimation is correct, (press key), record a robot pose.
5. Convert the robot pose and part pose to transformation matrix
6. Represent the robot pose relative to the part frame.
7. Convert back to quaternion (PoseStamped)
8. Save in yaml, with a name. 

- Load these into the state machine as gripping poses.
- Depending on the estimated pose, select the apropriate 
gripping pose.