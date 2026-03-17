TODO move into separate repo

```
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
points: [ {"positions":[0.2, 0, 0, 0, 0, 0]} ]"
```
