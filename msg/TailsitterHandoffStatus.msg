# Internal feedback from the destination controller's one-shot collective slew limiter.
uint64 timestamp
uint64 handoff_id
bool throttle_slew_active
bool in_transition # MC controller has not yet acquired the post-transition flight task
float32 thrust # Applied positive collective after destination battery scaling, [0,1]
float32 demand # Current collective demand after battery scaling, [0,1]
float32 battery_scale # Converts commands back into the destination controller's coordinates
