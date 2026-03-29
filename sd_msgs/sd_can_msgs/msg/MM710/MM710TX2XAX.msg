std_msgs/Header header

# Fields from DBC file in order (by bit position)
uint8 crc  # MM710_CRC_TX2 (56|8)
uint8 ax_stat  # MM710_AX_STAT (52|4)
uint8 msg_cnt  # MM710_MSG_CNT_TX2 (48|4)
float64 ax  # MM710_AccX (32|16) in G
uint8 clu_diag  # MM710_CLU_DIAG (24|8)
uint8 roll_rate_stat  # MM710_RollRate_STAT (20|4)
uint8 clu_stat  # MM710_CLU_STAT5 (16|4)
float64 roll_rate  # MM710_RollRate (0|16) in °/s 