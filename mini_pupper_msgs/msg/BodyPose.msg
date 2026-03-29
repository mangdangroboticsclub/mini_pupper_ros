# Desired body pose for Mini Pupper
std_msgs/Header header

# Body position offset from nominal stance (meters)
geometry_msgs/Vector3 position

# Body orientation as roll/pitch/yaw (radians)
geometry_msgs/Vector3 orientation

# Stance height override (meters, 0 = use default)
float64 height
