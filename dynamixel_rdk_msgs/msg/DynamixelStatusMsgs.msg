########################################
# Messages
########################################
## Dynamixel Status Message
########################################

std_msgs/Header header
int32 id
bool torque_enabled             # 1 if torque is enabled, 0 if disabled
uint8 error_status            # https://emanual.robotis.com/docs/kr/dxl/mx/mx-106-2/#shutdown63  
float64 present_position        # radians
float64 present_velocity        # radians/s
float64 present_load            # Nm   
float64 present_current         # A 
float64 present_voltage         # V
float64 present_temperature     # C
