[apf_node]: clearance 0.193 m, speed 0.080 m/s
[apf_node]: clearance 0.193 m, speed 0.080 m/s
[apf_node]: clearance 0.193 m, speed 0.080 m/s
位置不变，但仍发送期望末端速度

机械臂停止条件
- 到达
- 末端到障碍物的安全距离 < d_min
-- 此时打印tool0 clearance ... is inside the stop distance 且期望速度为0
- 缺少以下任何数据：
-- /robot_description
-- /joint_states
-- /obstacle/pose
- 触发关节限位
- joint_trajectory_controller没activate


ros2 control list_controllers
控制器activate

ros2 topic hz /joint_trajectory_controller/joint_trajectory
有消息往控制器发


ros2 topic echo /joint_states
header:
  stamp:
    sec: 39
    nanosec: 246000000
  frame_id: base_link
name:
- shoulder_pan_joint
- shoulder_lift_joint
- elbow_joint
- wrist_1_joint
- wrist_2_joint
- wrist_3_joint
position:
- 0.6829986083824421
- -1.5355238177455333
- 1.4043353963406433
- -1.5737976343289888
- -1.5657420456744653
- -0.7334174841377237
velocity:
- -3.2448169419452017e-07
- 2.2723725915567775e-07
- -8.04452680736728e-07
- -9.013276236480294e-14
- -1.0839940056683872e-13
- -5.00588611361541e-14
effort:
- -330.0
- 330.0
- 150.0
- -3.4178286509286537
- 0.004427541788830281
- 7.513306096419399e-15

ros2 topic echo /joint_trajectory_controller/joint_trajectory

发现apf有给目标，但力矩顶满了，电机给不出速度，末端姿态不变，apf持续发送目标，但是不动

实测发现，current和target只差0.008，但是产生了330的力矩，说明可能没补偿重力
