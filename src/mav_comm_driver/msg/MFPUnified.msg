Header header
uint8 msg_id
uint8 length
uint8[] data    # include id , length , data (not include the frame head AAAF/AAAA)

# frame id presets
uint8 UP_STATUS = 1     # 0x01
uint8 UP_SENSER = 2     # 0x02
uint8 UP_CHECK = 239    # 0xEF

uint8 UP_PID1 = 16      # 0x10
uint8 UP_PID2 = 17      # 0x11
uint8 UP_PID3 = 18      # 0x12
uint8 UP_PID4 = 19      # 0x13
uint8 UP_PID5 = 20      # 0x14
uint8 UP_PID6 = 21      # 0x15

uint8 DOWN_ACK = 2      # 0x02