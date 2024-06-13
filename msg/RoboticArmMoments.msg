uint64 timestamp        # time since system start (microseconds)
uint64 timestamp_sample # timestamp of the data sample on which this message is based (microseconds)

float32[3] xyz          # torque setpoint about X, Y, Z body axis (normalized)

# TOPICS robotic_arm_moments
# TOPICS robotic_arm_moments_virtual_fw robotic_arm_moments_virtual_mc
