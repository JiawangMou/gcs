Header header
uint8 mode_id
uint8[] data    # must include id as header

# modes
uint8 FAULT_MODE = 0 # 0x00
uint8 START_MODE = 8 # 0x08
uint8 MANUAL_MODE = 16 # 0x10
uint8 FLIGHT_MODE = 24 # 0x18
uint8 TUNING_MODE = 56 # 0x38