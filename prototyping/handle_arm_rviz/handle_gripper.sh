rostopic pub vp6242/gripper_position_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'gripper_finger1_joint'
points:
- positions: [0.50]
  velocities: [0]
  accelerations: [0]
  effort: [0]
  time_from_start: {secs: 1, nsecs: 0}" --once

