### Launch AR Tag tracking
```
roslaunch ar_tag_tracking ar_tag_tracking.launch
```

### Send a Command to fetch target position for the manipulator
```
rostopic pub /command std_msgs/String calculate_target_pose
```