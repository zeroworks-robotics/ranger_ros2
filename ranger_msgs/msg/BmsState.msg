# Battery management system feedback, CAN frame 0x361.
#
# sensor_msgs/BatteryState carries most of this already and stays the topic to
# use for the common fields; it has no field for the state of health, which the
# chassis reports and which has to be logged, so the whole frame is mirrored
# here rather than bending BatteryState's fields out of their documented units.
#
# Scaling and ranges follow the RANGER MINI 3.0 manual, frame 0x361.

std_msgs/Header header

uint8 battery_soc         # state of charge, percent, 0~100 (byte[0])
uint8 battery_soh         # state of health, percent, 0~100 (byte[1])
float32 voltage           # V (byte[2..3], 0.01 V units)
float32 current           # A, positive charging / negative discharging (byte[4..5], 0.1 A units)
float32 temperature       # degrees Celsius (byte[6..7], 0.1 degree units)

# See SystemState.feedback_count. This one counts the BMS group, which the
# chassis sends far more slowly than the rest (500 ms per the manual, about
# 250 ms measured), and whose values can legitimately hold still for minutes -
# on a captured run all four readings held one combination for 481.6 s while
# parked at full charge. A published count is always at least 1.
uint32 feedback_count
