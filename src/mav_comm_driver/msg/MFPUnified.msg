Header header
uint8 frame_id
uint8 length
uint8[] data    # must include id as header

# frame id presets
uint8 UP_STATUS = 1 # 0x01
uint8 UP_SENSER = 2 # 0x02
