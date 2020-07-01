Header header
uint8 msg_id
uint8 length
uint8[] data    # include id , length , data (not include the frame head AAAF/AAAA)

# frame id presets
uint8 UP_STATUS = 1 # 0x01
uint8 UP_SENSER = 2 # 0x02
