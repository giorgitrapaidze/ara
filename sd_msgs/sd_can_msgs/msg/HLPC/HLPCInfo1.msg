# HLPC Info1 (NEW in v5)
# Frame ID: 0x700
# High-Level Perception Computer sensor status information
# Reports status of perception sensors (lidars, radars, GPS)
# Frequency: 1 Hz
std_msgs/Header header

# ========== NETWORK STATUS ==========
uint16 hlpc_network_ping  # Network ping time [ms], Range: -, Scale: 1, Offset: 0, Freq: 1 Hz

# ========== LIDAR STATUS ==========
# Status values: 0x00 = Inactive, 0x01 = Active, 0x02 = Error, 0x03 = Reserved
uint8 hlpc_lidar_status_left   # Left Lidar Status [-], Range: 0..3, Scale: 1, Offset: 0
uint8 hlpc_lidar_status_front  # Front Lidar Status [-], Range: 0..3, Scale: 1, Offset: 0
uint8 hlpc_lidar_status_right  # Right Lidar Status [-], Range: 0..3, Scale: 1, Offset: 0

# ========== RADAR STATUS ==========
# Status values: 0x00 = Inactive, 0x01 = Active, 0x02 = Error, 0x03 = Reserved
uint8 hlpc_radar_status_front  # Front Radar Status [-], Range: 0..3, Scale: 1, Offset: 0
uint8 hlpc_radar_status_left   # Left Radar Status [-], Range: 0..3, Scale: 1, Offset: 0
uint8 hlpc_radar_status_right  # Right Radar Status [-], Range: 0..3, Scale: 1, Offset: 0
uint8 hlpc_radar_status_rear   # Rear Radar Status [-], Range: 0..3, Scale: 1, Offset: 0

# ========== GPS STATUS ==========
uint8 hlpc_vector_nav_fix      # VectorNav GPS Fix Status [-], Range: 0..8, Scale: 1, Offset: 0

# ========== ALIVE SIGNAL ==========
uint8 hlpc_alive_c4            # HLPC CAN4 Alive Counter [-], Range: 0..15, Scale: 1, Offset: 0

