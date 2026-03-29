std_msgs/Header header
            
# EM status for the system
uint8 dcdc4812_voltage_v       # Voltage in volts (0..25.4 bar)
uint16 em_status                # EM status code
uint16 hl_stop_deceleration_1   # Deceleration 1 in m/s
uint16 hl_stop_deceleration_2   # Deceleration 2 in m/s
uint16 hl_stop_time_to_dec_1    # Time to deceleration 1 in seconds
uint16 hl_stop_time_to_dec_2    # Time to deceleration 2 in seconds

