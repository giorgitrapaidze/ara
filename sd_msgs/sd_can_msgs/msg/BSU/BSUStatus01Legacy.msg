std_msgs/Header header

#Range: 0..1 (0..1 bool)
uint8 bsu_hl_stop_request

#Range: 0.1 (0..1 bool)
uint8 bsu_hl_warning

#Range: 0..1 (0..1 enum)
uint8 bsu_em_stop_activated

#Range: 0..1 (0..1 enum)
uint8 bsu_ml_stop_activated

#Range: 0..15 (0..15 uint)
uint8 bsu_alive_counter

#Range: 0..1023 (0..1023 uint)
uint16 bsu_status

#Range: 0..1 (0..1 -)
uint8 abs_external_enable_ack

uint8 BSU_HL_STOP_REQUEST_FALSE = 0
uint8 BSU_HL_STOP_REQUEST_TRUE = 1