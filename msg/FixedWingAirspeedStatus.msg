# Airspeed override state reported by the fixed-wing mode manager.
uint64 timestamp # [us] Time since system start

# True while Mission throttle nudging is applied outside the centre deadband.
# Independent of speed-command updates and subsequent controller limits/slew limiting.
bool airspeed_override_active
