std_msgs/Header header

uint8 hl_alive_02  # Range: 0..15 (0..15 uint), Scale: 1, Offset: 0
uint8 hl_psa_mode_of_operation  # Range: 0..7 (0..7 enum), Scale: 1, Offset: 0
float64 hl_target_psa_control  # Range: 0..4000 (-100..100 %), Scale: 0.05, Offset: -100
uint16 hl_psa_profile_acc_rad_s2  # If This Signal is set to 0, Default value will be send by the BSU, Range: 0..20000 (0..2000 rad/s2), Scale: 0.1, Offset: 0
uint16 hl_psa_profile_dec_rad_s2  # If This Signal is set to 0, Default value will be send by the BSU, Range: 0..20000 (0..2000 rad/s2), Scale: 0.1, Offset: 0
uint16 hl_psa_profile_vel_rad_s  # If This Signal is set to 0, Default value will be send by the BSU, Range: 0..20000 (0..2000 rad_s), Scale: 0.1, Offset: 0
