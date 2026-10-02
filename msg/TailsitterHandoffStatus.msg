# Internal feedback from the one-shot MC -> FW collective slew limiter.
uint64 timestamp
uint64 handoff_id
bool throttle_slew_active
float32 thrust # Applied positive collective after FW battery scaling, [0,1]
float32 demand # Current FW collective demand after battery scaling, [0,1]
float32 battery_scale # Converts these commands back into TECS throttle coordinates
