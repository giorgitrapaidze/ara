std_msgs/Header header

float64 psa_actual_pos_rad  # Range: 0..10200 (-25.5..25.5 rad), Scale: 0.005, Offset: -25.5
float64 psa_actual_speed_rad_s  # Range: 0..5100 (-255..255 rad_s), Scale: 0.1, Offset: -255
float64 psa_actual_torque_m_nm  # Range: 0..6400 (-32000..32000 mNm), Scale: 10, Offset: -32000
uint8 psa_actual_mode_of_operation  # Range: 0..7 (0..7 enum), Scale: 1, Offset: 0
float64 psa_actual_current_a  # Range: 0..1000 (-50..50 A), Scale: 0.1, Offset: -50
float64 psa_actual_voltage_v  # Range: 0..160 (0..64 V), Scale: 0.4, Offset: 0
