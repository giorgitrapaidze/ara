# Badenia560 Misc4 (NEW in v5)
# Frame ID: 0x453
# TPMS Diagnostics - Transmission check and alive counters
std_msgs/Header header

# ========== TPMS TRANSMISSION CHECK FLAGS ==========
# 0 = TPMS sensor is transmitting, 1 = Not transmitting
uint8 bad_tpms_check_fl  # Front Left TPMS Transmission Check [-], Range: 0..1, Scale: 1, Offset: 0
uint8 bad_tpms_check_fr  # Front Right TPMS Transmission Check [-], Range: 0..1, Scale: 1, Offset: 0
uint8 bad_tpms_check_rl  # Rear Left TPMS Transmission Check [-], Range: 0..1, Scale: 1, Offset: 0
uint8 bad_tpms_check_rr  # Rear Right TPMS Transmission Check [-], Range: 0..1, Scale: 1, Offset: 0

# ========== TPMS ALIVE COUNTERS ==========
# Alive signal from each TPMS sensor (increments with each transmission)
uint8 bad_tpms_tx_count_fl  # Front Left TPMS Alive Count [-], Range: 0..255, Scale: 1, Offset: 0
uint8 bad_tpms_tx_count_fr  # Front Right TPMS Alive Count [-], Range: 0..255, Scale: 1, Offset: 0
uint8 bad_tpms_tx_count_rl  # Rear Left TPMS Alive Count [-], Range: 0..255, Scale: 1, Offset: 0
uint8 bad_tpms_tx_count_rr  # Rear Right TPMS Alive Count [-], Range: 0..255, Scale: 1, Offset: 0

