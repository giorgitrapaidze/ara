std_msgs/Header header

# ========== v7 BREAKING CHANGES ==========
# NOTE: In v7, hl_pc_temp is renamed to hl_cpu_temp in DBC
# For backward compatibility, we keep hl_pc_temp here and map it in the handler
int8 hl_pc_temp           # Range: [-40..215] (degC), Scale: 1, Offset: -40 (maps to hl_cpu_temp in v7)
uint8 hl_cpu_usage        # Range: [0..100] (%), Scale: 1, Offset: 0
uint8 hl_gpu_usage        # Range: [0..100] (%), Scale: 1, Offset: 0

# NEW in v7: GPU Temperature Monitoring
int8 hl_gpu_temp          # Range: [-40..215] (degC), Scale: 1, Offset: -40 - New in v7

# REMOVED in v7 (kept for backward compatibility, will be 0 in v7):
uint8 hl_perception_acc   # Range: [0..100] (%), Scale: 1, Offset: 0 - REMOVED in v7
uint8 hl_localization_acc # Range: [0..100] (%), Scale: 1, Offset: 0 - REMOVED in v7
uint16 hl_watts_consumed  # Range: [0..1023] (W), Scale: 1, Offset: 0 - REMOVED in v7
uint8 hl_aggressiveness   # Range: [0..15] (enum), Scale: 1, Offset: 0 - REMOVED in v7
# ========================================== 