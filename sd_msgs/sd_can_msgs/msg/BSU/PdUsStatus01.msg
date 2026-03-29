std_msgs/Header header

# Power Distribution Unit Status
float64 pdu12_power_supply_voltage_v    # PDU12 power supply voltage (0..65.535 V)
float64 pdu12_total_current_a            # PDU12 total current (0..409 A)
float64 pdu48_power_supply_voltage_v     # PDU48 power supply voltage (0..65.535 V)
float64 pdu48_total_current_a            # PDU48 total current (0..409 A)

float64 pdu1_power_supply_voltage_v_legacy   # Range: 0..250 (0..25 V), Scale: 0.1, Offset: 0
float64 pdu1_total_current_a_legacy          # Range: 0..4090 (0..409 A), Scale: 0.1, Offset: 0
float64 pdu2_power_supply_voltage_v_legacy   # Range: 0..250 (0..25 V), Scale: 0.1, Offset: 0
float64 pdu2_total_current_a_legacy          # Range: 0..4090 (0..409 A), Scale: 0.1, Offset: 0
