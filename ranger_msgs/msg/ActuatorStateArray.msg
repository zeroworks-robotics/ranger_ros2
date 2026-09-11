std_msgs/Header header

ActuatorState[] states

# Number of distinct actuator feedback updates seen on the CAN bus, incremented
# once per update and wrapping at 2^32. See SystemState.feedback_count: this one
# counts the actuator group (0x251-0x258, 0x261-0x268) separately, so a consumer
# can tell that the drivers went quiet while the rest of the chassis keeps
# reporting. The node republishes its last known state at a fixed rate, so
# message arrival alone does not carry that information.
# A published message always carries a count of at least 1: the node does not
# publish until the group has been updated at least once, and the counter is
# incremented before publishing. A 0 therefore means the field was not written
# at all - an older publisher whose message a newer ranger_msgs deserialised -
# and never means "the bus is dead". Keep that ordering if this is changed.
uint32 feedback_count