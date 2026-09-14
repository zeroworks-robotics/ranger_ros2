std_msgs/Header header

ActuatorState[] states
# Number of distinct actuator feedback updates seen on the CAN bus, incremented
# once per update and wrapping at 2^32. See SystemState.feedback_count: this one
# counts the actuator group (0x251-0x258, 0x261-0x268) separately, so a consumer
# can tell that the drivers went quiet while the rest of the chassis keeps
# reporting.
# A published message always carries a count of at least 1. The counter skips 0
# when it wraps, so this holds for every message rather than only until the
# first wrap. A 0 therefore means the field was not written at all - an older
# publisher whose message a newer ranger_msgs deserialised - and never means
# "the bus is dead".
#
# Consumers use that to tell an old publisher from a live one, so the guarantee
# is part of the contract rather than an implementation detail: keep the skip if
# the counter changes.
uint32 feedback_count
