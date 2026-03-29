# EAC24BsuHlMsg03.msg
std_msgs/Header header

# HL message for the system
uint8 hl_alive_03                      # Alive status (0..15)
bool hl_dbw_enable                      # DBW (Drive by Wire) enabled status (0..1)
uint8 hl_push_to_pass_on               # Push-to-pass status (0..1)
bool hl_pdu12_activate_gnss            # GNSS activation status (0..1)
bool hl_pdu12_activate_oss              # OSS activation status (0..1)
bool hl_ice_enable                      # ICE (Internal Combustion Engine) enabled status (0..1)
bool hl_pdu12_activate_lidar            # Lidar activation status (0..1)
bool hl_pdu12_activate_radar            # Radar activation status (0..1)
uint16 ice_start_fuel_level_l          # Start fuel level in liters (0..655.35)
bool hl_crancking_by_pass               # Cranking bypass status (0..1)
bool hl_switch_off_ok                   # Switch-off confirmation status (0..1)
