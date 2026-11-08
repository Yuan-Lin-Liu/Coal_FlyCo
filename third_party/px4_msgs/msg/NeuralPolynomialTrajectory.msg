# Compact polynomial trajectory for mc_nn_control.
# Position coefficients are expressed in the local NED frame and ordered from
# highest polynomial power to constant term. PX4 interprets t=0 as the local
# receive time of this message, so ROS 2 and PX4 clocks do not need to be synced.

uint8 MAX_POLY_PIECES = 4
uint8 POLY_COEFFS = 8

uint64 timestamp # [us] ROS 2 publish time, metadata only
uint32 sequence

float32[4] duration

float32[32] position_x
float32[32] position_y
float32[32] position_z

float32 yaw # [rad] local NED heading at t=0

# Keep uint8 metadata at the end so ROS 2 wire layout matches packed uORB.
uint8 piece_num
