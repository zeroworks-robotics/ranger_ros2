# Values match the RANGER MINI 3.0 manual, frame 0x211 byte[0].
# The chassis reports only 0x00 and 0x02; there is no 0x01 for this model, so
# the ESTOP constant that the AgileX family enum carries is not declared here.
# An e-stop shows up as VEHICLE_STATE_EXCEPTION, and the bit that tells it
# apart (0x211 byte[7] bit7) is not reachable through error_code - ugv_sdk
# reads only byte[4..5] of the 4-byte error message.
uint8 VEHICLE_STATE_NORMAL = 0
uint8 VEHICLE_STATE_EXCEPTION = 2

# Values match the manual, frame 0x211 byte[1], and ugv_sdk AgxControlMode.
# 0x02 (UART) exists in the family enum but not in this model's manual and has
# never been observed, so it is left undeclared: treat any other value as
# unknown rather than mapping it.
uint8 CONTROL_MODE_STANDBY = 0
uint8 CONTROL_MODE_CAN = 1
uint8 CONTROL_MODE_RC = 3

# Values match ugv_sdk RangerInterface::MotionMode.
uint8 MOTION_MODE_DUAL_ACKERMAN = 0
uint8 MOTION_MODE_PARALLEL = 1
uint8 MOTION_MODE_SPINNING = 2
uint8 MOTION_MODE_PARKING = 3
uint8 MOTION_MODE_SIDE_SLIP = 4

########################################

std_msgs/Header header

# system state
uint8 vehicle_state
uint8 control_mode

# Frame 0x211 bytes [4..5]. On this model those two bytes only carry the
# steering zero-point calibration flags (bit 0~3, one per wheel) and the
# calibration timeout (bit 4); everything else in them is reserved. It is kept
# for consumers that already read it - the faults are in error_code_full.
uint16 error_code

# Frame 0x211 bytes [4..7] packed big-endian, byte [4] most significant. The
# manual defines the whole range as one unsigned int32, and the faults that
# matter live in the two bytes error_code drops:
#   byte [6] bit 0      driver status
#   byte [6] bit 2~5    motor 5~8 driver communication
#   byte [6] bit 6      over-temperature protection
#   byte [6] bit 7      over-current protection
#   byte [7] bit 0      battery undervoltage
#   byte [7] bit 1      overvoltage protection
#   byte [7] bit 2      remote control connection lost
#   byte [7] bit 3~6    motor 1~4 driver communication
#   byte [7] bit 7      emergency stop triggered
# So a driver fault reads 0x00000100, a lost remote 0x00000004 and an emergency
# stop 0x00000080. A captured run of this robot carried 0x100 for most of one
# file, and none of it was visible through error_code.
uint32 error_code_full

float64 battery_voltage

# motion state
uint8 motion_mode

# Number of distinct chassis feedback updates seen on the CAN bus for this
# group, incremented once per update and wrapping at 2^32 (about 2.7 years at
# the default 50 Hz). The chassis has no rolling counter of its own and this
# node republishes its last known state at a fixed rate, so a consumer cannot
# tell a live bus from a dead one by looking at the values: they can legitimately
# stay identical for minutes. Watch this field instead - it stops advancing
# within one or two publish periods of the feedback stopping.
# A published message always carries a count of at least 1: the node does not
# publish until the group has been updated at least once, and the counter is
# incremented before publishing. A 0 therefore means the field was not written
# at all - an older publisher whose message a newer ranger_msgs deserialised -
# and never means "the bus is dead". Keep that ordering if this is changed.
uint32 feedback_count
