std_msgs/Header header

uint16 sensor_id  # Range: 0..9999 (0..9999 -), Scale: 1, Offset: 0
float64 temperature  # Range: -128..127 (-128..127 °C), Scale: 1, Offset: 0
float64 lamp_current  # Range: 0..255 (0..2.55 A), Scale: 0.01, Offset: 0
uint8 filter_setting  # Range: 0..255 (0..255 -), Scale: 1, Offset: 0
uint8 stst  # Range: 0..1 (0..1 -), Scale: 1, Offset: 0
uint8 filter_off_on  # Range: 0..1 (0..1 -), Scale: 1, Offset: 0
uint8 lamp_current_control  # Range: 0..1 (0..1 -), Scale: 1, Offset: 0
uint8 temperature_ok  # Range: 0..1 (0..1 -), Scale: 1, Offset: 0
uint8 head_status  # Range: 0..3 (0..3 -), Scale: 1, Offset: 0
uint8 angle_switched_off  # Range: 0..1 (0..1 -), Scale: 1, Offset: 0
uint8 direction  # Range: 0..1 (0..1 -), Scale: 1, Offset: 0
uint8 ang_vel_correction  # Range: 0..1 (0..1 -), Scale: 1, Offset: 0
uint8 direction_motion  # Range: 0..1 (0..1 -), Scale: 1, Offset: 0
uint8 direction_mounting  # Range: 0..1 (0..1 -), Scale: 1, Offset: 0
uint8 direction_head_is_valid  # Range: 0..1 (0..1 -), Scale: 1, Offset: 0
uint8 direction_head  # Range: 0..1 (0..1 -), Scale: 1, Offset: 0
