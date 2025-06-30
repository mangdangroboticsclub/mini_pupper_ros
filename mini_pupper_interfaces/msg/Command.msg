# Horizontal velocity (2D vector)
float64[2] horizontal_velocity

# Yaw rate
float64 yaw_rate

# Height
float64 height

# Orientation (pitch, roll, yaw)
float64 pitch
float64 roll
float64 yaw

# Activation
int32 activation

# For dancing
Matrix3x4 legs_location 

# Events
bool hop_event
bool trot_event
bool activate_event
bool dance_activate_event
bool pseudo_dance_event