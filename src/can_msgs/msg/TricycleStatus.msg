std_msgs/Header header
uint32 id
#uint8 dlc
string tricycle_mode 
float64 battery_voltage
uint32 battery_pct 
float64 wheel_speed 
string brake_status 
string brake_sys_voltage 
string steering_sys_voltage 
string mcu_power_supply 

## @file TricycleStatus.msg
# @ingroup ros2can_bridge
# @brief message type to publish can messages to ROS topic
# @param id CAN message id (11/29 bit)
# @param dlc CAN data size in bytes
# @param tricycle_mode driving mode (Local or Remote)
# @param battery voltage for power supply
# @param battery percentage for power supply
# @param wheel speed (tr/s)
# @param brake status (ON/OFF)
# @param brake system voltage (Ok/Default)
# @param steering system voltage (Ok/Default)
# @param microcontrollers power suplly manager (Ok/Default)