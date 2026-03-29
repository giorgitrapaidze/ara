# Badenia560 Misc5 (NEW in v5)
# Frame ID: 0x454
# TPMS Lifetime Counters - Total transmissions from each sensor
std_msgs/Header header

# ========== TPMS LIFETIME TRANSMISSION COUNTERS ==========
# Total number of transmissions from each TPMS sensor over its lifetime
uint16 bad_tpms_tx_life_fl  # Front Left TPMS Lifetime Count [-], Range: 0..4096, Scale: 1, Offset: 0
uint16 bad_tpms_tx_life_fr  # Front Right TPMS Lifetime Count [-], Range: 0..4096, Scale: 1, Offset: 0
uint16 bad_tpms_tx_life_rl  # Rear Left TPMS Lifetime Count [-], Range: 0..4096, Scale: 1, Offset: 0
uint16 bad_tpms_tx_life_rr  # Rear Right TPMS Lifetime Count [-], Range: 0..4096, Scale: 1, Offset: 0

