# Internal, volatile tailsitter controller transfer. All vectors are in the MC body frame.
uint64 timestamp
uint64 timestamp_sample
uint64 handoff_id # Request identity; unchanged by activation and acknowledgement. Zero cancels.
bool active # Router has accepted the matched FW output; release the transient correction.
float32[3] torque # Outgoing motor torque command (after MC output/battery scaling)
float32[3] torque_bias # Effective MC integral contribution (after battery scaling)
float32[3] rates # Outgoing rate target, rad/s
float32[3] differential_thrust_scale # MC-axis FW motor scales; zero for disabled axes
float32 thrust # Positive collective command, i.e. minus MC body-z thrust

# TOPICS tailsitter_handoff_mc tailsitter_handoff tailsitter_handoff_ack
