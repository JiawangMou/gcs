Header header
uint8 msg_id
uint8 length
uint8[] data    # include id , length , data (not include the frame head AAAF/AAAA)

# frame id presets
uint8 UP_STATUS = 1         # 0x01
uint8 UP_SENSER = 2         # 0x02
uint8 UP_RCDATA = 3         # 0x03
uint8 UP_CHECK = 239        # 0xEF
uint8 UP_POWER = 5          # 0x05
uint8 UP_MOTOR = 6          # 0x06

uint8 UP_PID1 = 16          # 0x10
uint8 UP_PID2 = 17          # 0x11
uint8 UP_PID3 = 18          # 0x12
uint8 UP_PID4 = 19          # 0x13
uint8 UP_PID5 = 20          # 0x14
uint8 UP_PID6 = 21          # 0x15

uint8 UP_PID_DEBUG = 8      # 0x08
uint8 UP_DATA_SAVE_DEBUG = 136   # 0x88

uint8 UP_USER_DATA1 = 241   # 0xF1
uint8 UP_USER_DATA2 = 242   # 0xF2
uint8 UP_USER_DATA3 = 243   # 0xF3

uint8 DOWN_ACK = 2          # 0x02
uint8 DOWN_MOTOR = 81       # 0x51

uint8 DOWN_PID1 = 16        # 0x10
uint8 DOWN_PID2 = 17        # 0x11
uint8 DOWN_PID3 = 18        # 0x12
uint8 DOWN_PID4 = 19        # 0x13
uint8 DOWN_PID5 = 20        # 0x14
uint8 DOWN_PID6 = 21        # 0x15