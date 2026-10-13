# Diagnostics for the calibrated SOC estimator; match instances by battery_id and source.
uint64 timestamp # [us] Publication time
uint64 timestamp_sample # [us] Battery measurement time
uint8 battery_id # 1-indexed BAT parameter instance
uint8 source # battery_status SOURCE
uint8 mode
uint8 MODE_DISABLED = 0
uint8 MODE_SHADOW = 1
uint8 MODE_ACTIVE = 2
float32 legacy_soc # [0,1] Existing PX4 estimate (or external SOC)
float32 coulomb_soc # [0,1] Pure coulomb estimate from the latest startup/rest seed; NaN if unavailable
float32 voltage_soc # [0,1] Calibrated compensated-voltage estimate; NaN if unavailable
float32 soc # [0,1] New combined estimate; NaN if unavailable
float32 voltage_v # [V] Terminal pack voltage
float32 current_a # [A] Battery current
float32 discharged_mah # [mAh] Independent measured discharge, never modified by SOC corrections
float32 compensated_cell_voltage # [V] Resistance-compensated filtered cell voltage
float32 filtered_terminal_cell_voltage # [V] Uncompensated cell voltage used for voltage warnings
bool profile_valid # Curve, capacity, cells and correction parameters are valid
bool protection_valid # Independent voltage protection is configured correctly
bool initialized # A defined SOC was seeded from measured voltage in this connection session
bool charge_history_valid # Measured charge history is complete since the latest voltage seed
bool full_charge_confirmed # Qualified disarmed relaxation reached the profile's full endpoint in this connection session
bool correction_eligible # Qualified disarmed relaxation or semi-stable electrical power while armed
bool active # New SOC is driving battery_status.remaining
bool external_soc # Source supplies its own SOC; new estimator does not override it
uint8 soc_warning # Warning derived from the reported SOC alone
uint8 voltage_warning # Independent prospective terminal-voltage warning (battery_status WARNING)
uint8 warning # Actual battery_status warning after mode selection
