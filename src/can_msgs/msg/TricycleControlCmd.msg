std_msgs/Header header
float32 throttle
float32 steer
string brake
bool reverse

## @file TricycleControlCmd.msg
# @ingroup ros2can_bridge
# @brief The Tricycle control data message type to publish can messages to ROS topic and can bus
# @param A scalar value to control the tricycle throttle [0. <= throttle <= 1.0]
# @param A scalar value to control the vehicle steering [-1.0 <= steer <= 1.0]
# @param A scalar value to control the vehicle brake [ON/OFF]
# @param reverse Determines whether the tricycle will move backwards. Default is False [0 or 1]