This package calls ar_track_alvar to track two markers with a logitech webcam. It provides the transformation between camera, robot and manipulators and calculates the target mount position in manipulator frame.

It is suggested to send the command to start sampling the target position when robot is stationary.

### Launch AR Tag tracking
```
roslaunch ar_tag_tracking ar_tag_tracking.launch
```

### Send a Command to fetch target position for the manipulator
```
rostopic pub /command std_msgs/String calculate_target_pose
```
