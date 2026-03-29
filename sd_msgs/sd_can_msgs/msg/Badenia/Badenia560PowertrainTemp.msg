std_msgs/Header header

# ========== v5 NEW: Rear Pushrod Strain ==========
float64 m_pushrod_strain_rl  # Rear Left Pushrod Strain [-], Range: -, Scale: 1, Offset: 32768
float64 m_pushrod_strain_rr  # Rear Right Pushrod Strain [-], Range: -, Scale: 1, Offset: 32768

# ========== v5 NEW: BSU Diagnostics ==========
uint8 bsu_prefill_diag       # BSU Prefill Diagnostics [-], Range: 0..15, Scale: 1, Offset: 0

# ========== Existing Signals ==========
float64 engine_oil_temperature  # Engine oil temperature [C], Range: -, Scale: 1, Offset: -40
float64 coolant_temperature  # Engine water temperature [C], Range: -, Scale: 1, Offset: -40
float64 gear_box_oil_temp  # Gearbox oil temperature [C], Range: -, Scale: 0.1, Offset: 0
