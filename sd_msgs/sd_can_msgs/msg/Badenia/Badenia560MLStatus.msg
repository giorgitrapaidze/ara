# Badenia560 ML Status (NEW in v5)
# Frame ID: 0x100
# ML status relay from BSU
# BSU Prefill Request signals
std_msgs/Header header

# ========== BSU PREFILL REQUEST SIGNALS ==========
# Relay of BSU prefill request values for all four corners
uint8 bsu_prefill_req_fl  # BSU Prefill Request Front Left [-], Range: 0..1, Scale: 1, Offset: 0
uint8 bsu_prefill_req_fr  # BSU Prefill Request Front Right [-], Range: 0..1, Scale: 1, Offset: 0
uint8 bsu_prefill_req_rl  # BSU Prefill Request Rear Left [-], Range: 0..1, Scale: 1, Offset: 0
uint8 bsu_prefill_req_rr  # BSU Prefill Request Rear Right [-], Range: 0..1, Scale: 1, Offset: 0

