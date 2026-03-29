# Command for a single leg of Mini Pupper
# Foot position target in the body frame
geometry_msgs/Point foot_position

# Joint torques override (optional, zero = position control)
float64[3] joint_torques

# True when this command is actively in use
bool active
