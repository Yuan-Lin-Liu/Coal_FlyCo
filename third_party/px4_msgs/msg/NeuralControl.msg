# Neural control
#
# Debugging topic for the Neural controller, logs the latest target tracking error,
# the average tracking error during the active control period, the current desired
# position and yaw, the neural network output, and the time it takes to run
# Publisher: mc_nn_control
# Subscriber: logger

uint64 timestamp # [us] Time since system start

float32 latest_position_error # [m] Total 3-axis tracking error of the latest target point
float32 average_position_error # [m] Mean total tracking error during the active control period
float32[3] desired_position_ned # [m] Current desired position in local NED frame
float32[3] desired_velocity_ned # [m] Current desired velocity in local NED frame
float32 desired_yaw_ned # [rad] Current desired yaw in PX4 heading convention in local NED frame
float32[4] network_output # Output from neural network
bool local_trajectory_tracking_started # True once the selected local trajectory has entered tracking

int32 controller_time # [us] Time spent from input to output
int32 inference_time # [us] Time spent for NN inference
