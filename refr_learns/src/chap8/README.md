# Teleop tools
Simple teleop program which considers accleration and deceleration. Scale of linear, angular velocity and scale of linear, angular accleration can be changed by passing parameters to key_to_twist.py

## Steps to run teleop
1) Run key_publisher.py to run key press moniter where the key press input is taken and published as 'String' messages in 'keys' topic 
2) Run key_to_twist.py with required parameters which publish 'Twist' messages to 'cmd_vel' topic according to the key press

## Parameters of key_to_twist.py:
~linear_scale - denotes maximum linear speed of robot
~angular_scale - denotes maximum angular velocity robot can achieive
~linear_accel - denotes linear accleration of robot
~angular_accel - denotes angular accleration of robot
