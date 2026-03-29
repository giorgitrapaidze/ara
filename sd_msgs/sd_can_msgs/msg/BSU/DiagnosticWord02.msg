std_msgs/Header header

# Diagnostic word information for BSU
uint8 rc_force_race_mode                    # RC Force Race Mode (0..1)
uint8 rc_beacon_time_out                     # RC Beacon Timeout (0..1)
uint8 display_timeout_error                   # Display Timeout Error (0..1)
uint8 rm_red_flag                             # RM Red Flag (0..1)
uint8 rm_safe_stop                            # RM Safe Stop (0..1)
uint8 display_counter_error                   # Display Counter Error (0..1)
uint8 cba_em_brake_sat_not_received          # CBA EM Brake Sat Not Received (0..1)
uint8 cba_sat_values_not_received             # CBA Sat Values Not Received (0..1)
uint8 ice_oil_temp_start_limit_not_rec       # ICE Oil Temp Start Limit Not Received (0..1)
uint8 ml_stop_brake_sat_not_received          # ML Stop Brake Sat Not Received (0..1)
uint8 psa_sat_values_not_received              # PSA Sat Values Not Received (0..1)
uint8 limp_sat_values_not_received             # Limp Sat Values Not Received (0..1)
uint8 fan_car_speed_max_value_not_rec         # Fan Car Speed Max Value Not Received (0..1)
uint8 fan_car_speed_min_value_not_rec         # Fan Car Speed Min Value Not Received (0..1)
uint8 fan_water_temp_max_value_not_rec        # Fan Water Temp Max Value Not Received (0..1)
uint8 fan_water_temp_min_value_not_rec        # Fan Water Temp Min Value Not Received (0..1)
uint8 heater_oil_temp_max_value_not_rec       # Heater Oil Temp Max Value Not Received (0..1)
uint8 ice_fuel_pres_start_limit_not_rec       # ICE Fuel Pres Start Limit Not Received (0..1)
uint8 ice_oil_temp_by_pass_not_received       # ICE Oil Temp Bypass Not Received (0..1)
uint8 psa_sat_rates_def_val                   # PSA Sat Rates Default Value (0..1) - New in 2025

# ========== NEW v3 BMS DIAGNOSTIC SIGNALS (CRITICAL) ==========
# BMS 12V Battery Monitoring
uint8 bms12_soc_under_level1                  # BMS 12V SOC Under Level 1 (0..1) - New in v3
uint8 bms12_soc_under_level2                  # BMS 12V SOC Under Level 2 (0..1) - New in v3
uint8 bms12_temp_over_level1                  # BMS 12V Temperature Over Level 1 (0..1) - New in v3
uint8 bms12_temp_over_level2                  # BMS 12V Temperature Over Level 2 (0..1) - New in v3

# BMS 48V Battery Monitoring  
uint8 bms48_soc_under_level1                  # BMS 48V SOC Under Level 1 (0..1) - New in v3
uint8 bms48_soc_under_level2                  # BMS 48V SOC Under Level 2 (0..1) - New in v3
uint8 bms48_temp_over_level1                  # BMS 48V Temperature Over Level 1 (0..1) - New in v3
uint8 bms48_temp_over_level2                  # BMS 48V Temperature Over Level 2 (0..1) - New in v3

# Badenia System Diagnostics
uint8 badenia_counter_error                   # Badenia System Counter Error (0..1) - New in v3
uint8 badenia_timeout_error                   # Badenia System Timeout Error (0..1) - New in v3

# Race Management Emergency Flag
uint8 rm_em_flag                              # Race Management Emergency Flag (0..1) - New in v3

# Enhanced Default Value Signals
uint8 ml_stop_brake_bias_def_val              # ML Stop Brake Bias Default Value (0..1) - New in v3
uint8 limp_ice_sat_values_def_val             # Limp ICE Saturation Values Default Value (0..1) - New in v3
uint8 cba_sat_rates_def_val                   # CBA Saturation Rates Default Value (0..1) - New in v3
uint8 limp_cba_sat_values_def_val             # Limp CBA Saturation Values Default Value (0..1) - New in v3
# ===============================================================

# ========== NEW v7 PUSH-TO-PASS & ML SUPERVISOR SIGNALS ==========
# Push-to-Pass Configuration Default Values
uint8 p2p_max_num_activation_def_val          # P2P Maximum Number of Activations Default Value (0..1) - New in v7
uint8 p2p_cooldown_duration_def_val           # P2P Cooldown Duration Default Value (0..1) - New in v7
uint8 p2p_active_duration_def_val             # P2P Active Duration Default Value (0..1) - New in v7
uint8 rc_delta_timeout_lim_def_val            # Race Control Delta Timeout Limit Default Value (0..1) - New in v7

# ML Supervisor
uint8 hl_mlsupervisor_request                 # High-Level ML Supervisor Request Flag (0..1) - New in v7
# =================================================================