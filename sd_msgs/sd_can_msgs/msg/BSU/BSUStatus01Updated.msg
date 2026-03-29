std_msgs/Header header

# BSU Status Information
uint8 bsu_switch_off_req                      # BSU Switch Off Request (0..1)
uint8 bsu_limp_mode_activated                 # BSU Limp Mode Activated (0..1)
uint8 bsu_hl_stop_request                     # BSU HL Stop Request (0..1)
uint8 bsu_hl_warning                          # BSU HL Warning (0..1)
uint8 bsu_em_stop_activated                   # BSU EM Stop Activated (0..1)
uint8 bsu_ml_stop_activated                   # BSU ML Stop Activated (0..1)
uint8 bsu_alive_counter                       # BSU Alive Counter (0..15)
uint16 bsu_status                             # BSU Status (0..1023)
uint8 abs_external_enable_ack                 # ABS External Enable Acknowledgment (0..1)

uint8 BSU_HL_STOP_REQUEST_FALSE = 0
uint8 BSU_HL_STOP_REQUEST_TRUE = 1