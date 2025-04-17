std_msgs/Header header

# One paddle drive, range -1 to 1
# 1 = max throttle, -1 = max brake
float64 throttle

# Steering control, range -1 to +1
# +1 = maximum left turn
float64 steer

# Gear mode: True = Reverse, False = Forward
bool reverse
