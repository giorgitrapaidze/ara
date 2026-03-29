# Flag Info Output Message
# CAN ID: 0x7C (124)
# This message contains race management flag information and LED status
# New in DBC v3 - provides race control and marshalling information

std_msgs/Header header

# LED status output from EMM Car Marshalling Equipment
# Range: 0-255
uint8 led_status

# EMM Car Marshalling Equipment outputs free running counter
# Range: 0-255
uint8 free_running_counter

# Spare field 1 for CAN ID 0x7C
uint8 spare1_0x7_c

# Spare field 2 for CAN ID 0x7C  
uint8 spare2_0x7_c

# Session Management - Session Type
# Values: 0=No Session, 64=Practice, 128=Qualifying, 192=Race
uint8 sm_session_type

# Session Management - Track Flag
# Values: 0=No Flag, 5=Red Flag, 7=Safety Car Out, 11=Full Course Yellow, 13=Code 60
uint8 sm_track_flag

# Session Management - Car Flag  
# Values include: 0=Off, 3=Blue, 16=Chequered, 18=Black&White, 26=Blue Static,
# 29=Yellow Speed Offender, 30=Go To Pit, 31=Long Lap, 32=Change Position,
# 33=Long Lap x2, 34=Drive Through, 35=Ride Through, 38=Time Penalty,
# 39=Loudness, 40=In Pit Penalty, 129=Orange Flag, 130=Purple Flag,
# 131=Safe Stop Flag, 132=Emergency Stop Flag
uint8 sm_car_flag

# CRC checksum for Session Management data
# Range: 0-255
uint8 crc_sm
