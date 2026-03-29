std_msgs/Header header

uint8 hl_alive_01  # Range: 0..15 (0..15 uint), Scale: 1, Offset: 0
float64 hl_target_pressure_rr  # Range: 0..2000 (0..100 %), Scale: 0.05, Offset: 0
float64 hl_target_pressure_rl  # Range: 0..2000 (0..100 %), Scale: 0.05, Offset: 0
float64 hl_target_pressure_fr  # Range: 0..2000 (0..100 %), Scale: 0.05, Offset: 0
float64 hl_target_pressure_fl  # Range: 0..2000 (0..100 %), Scale: 0.05, Offset: 0
uint8 hl_target_gear  # Range: 0..9 (-1..8 enum), Scale: 1, Offset: -1
float64 hl_target_throttle  # Range: 0..1000 (0..100 %), Scale: 0.1, Offset: 0
