std_msgs/Header header

# Fields from DBC file in order (by bit position)
uint8 crc  # MM710_CRC_TX1 (56|8)
uint8 ay_stat  # MM710_AY_STAT (52|4)
uint8 msg_cnt  # MM710_MSG_CNT_TX1 (48|4)
float64 ay  # MM710_AccY (32|16) in G
float64 temp_rate_z  # MM710_TEMP_RATE_Z (24|8) in °C
uint8 yaw_rate_stat  # MM710_YawRate_STAT (20|4)
uint8 clu_stat  # MM710_CLU_STAT (16|4)
float64 yaw_rate  # MM710_YawRate (0|16) in °/s 