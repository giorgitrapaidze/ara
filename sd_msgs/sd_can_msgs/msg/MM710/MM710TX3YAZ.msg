std_msgs/Header header

# Fields from DBC file in order (by bit position)
uint8 crc  # MM710_CRC_TX3 (56|8)
uint8 az_stat  # MM710_AZ_STAT (52|4)
uint8 msg_cnt  # MM710_MSG_CNT_TX3 (48|4)
float64 az  # MM710_AccZ (32|16) in G
uint8 pitch_rate_stat  # MM710_PitchRate_STAT (20|4)
int8 hw_index  # MM710_HW_Index (16|4)
float64 pitch_rate  # MM710_PitchRate (0|16) in °/s 