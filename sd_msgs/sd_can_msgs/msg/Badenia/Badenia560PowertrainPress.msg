std_msgs/Header header

# ========== v5 NEW: Front Pushrod Strain ==========
float64 m_pushrod_strain_fl  # Front Left Pushrod Strain [-], Range: -, Scale: 1, Offset: 32768
float64 m_pushrod_strain_fr  # Front Right Pushrod Strain [-], Range: -, Scale: 1, Offset: 32768

# ========== Existing Signals ==========
float64 engine_oil_pressure  # Engine oil pressure [kPa], Range: -, Scale: 0.1, Offset: 0 (removed in DBC v5)
float64 coolant_pressure  # Engine water pressure [kPa], Range: -, Scale: 0.1, Offset: 0 (removed in DBC v5)
float64 fuel_press_direct  # GDI pressure [kPa], Range: -, Scale: 1, Offset: 0
float64 boost_pressure  # Boost pressure [kPa], Range: -, Scale: 0.1, Offset: 0
