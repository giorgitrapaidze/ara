std_msgs/Header header

# ========== v5 NEW: Battery Monitoring ==========
uint8 b12_soc              # 12V Battery State of Charge [%], Range: 0..100, Scale: 1, Offset: 0
uint8 b48_soc              # 48V Battery State of Charge [%], Range: 0..100, Scale: 1, Offset: 0
uint8 b12_temp             # 12V Battery Temperature [C], Range: -, Scale: 1, Offset: 0
uint8 b48_temp             # 48V Battery Temperature [C], Range: -, Scale: 1, Offset: 0

# ========== Existing Signals ==========
float64 throttle_position  # Throttle valve position [%], Range: -, Scale: 0.1, Offset: 0 (removed in DBC v5)
float64 engine_speed_limit  # Rev limiter [rpm], Range: -, Scale: 1, Offset: 0
float64 fuel_used_kg  # Estimated fuel usage [kg], Range: -, Scale: 0.01, Offset: 0
