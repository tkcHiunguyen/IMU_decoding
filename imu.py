import serial
import time
import struct

# Cấu hình cổng serial
port = 'COM13'
baudrate = 115200
timeout = 1  # Thời gian chờ đọc dữ liệu (giây)

# Khởi tạo kết nối serial
ser = serial.Serial(port, baudrate, timeout=timeout)
MSG_START_BYTE = b"\x75\x65"
# Gửi lệnh kích hoạt stream dữ liệu
buffer = bytearray()
def classify_packet(descriptor, payload):
    offset=0
    # print(f"Descriptor Set byte: 0x{descriptor:02x}")
    while offset< len(payload):
        packet_length=payload[offset]
        if packet_length==0 or offset+packet_length> len(payload):
            print("stop parsing")
            break
        small_packet= payload[offset:offset+packet_length]
        # print(f"Small packet (length {packet_length}):", small_packet.hex())
        # IMU
        if descriptor==0x82:    
            if small_packet[1]==0x11:
                #GPS time stamp
                message=small_packet[1:]         # Message DATA  
                time_of_week= message[0:8]       # data type: double, unit: Secconds
                week_number= message[8:10]       # data type: U16, unit: N/A
                valid_flag= message[10:12]       # data type: Valid Flags, unit: 0x0000 – Time Invalid    0x0001 – Time Valid
            elif small_packet[1]== 0x05:
                #Orientation, Euler Angles 
                message=small_packet[2:]        # Message DATA  
                # roll= message[0:4]              # data type: float, unit: rad
                # pitch= message[4:8]             # data type: float, unit: rad
                # yaw= message[8:12]              # data type: float, unit: rad
                # valid_flag= message[12:14]      # data type: Valid Flags, unit: 0x0000 – Time Invalid    0x0001 – Time Valid
                euler_unpacked = struct.unpack('>fffH', message)
                roll = euler_unpacked[0]
                pitch = euler_unpacked[1]
                yaw = euler_unpacked[2]
                valid_flag = euler_unpacked[3]
                if valid_flag == 1:
                    print(f"roll = {roll}, pitch = {pitch}, yaw = {yaw}, valid flag: {valid_flag}")
                    # print(f"pitch = {pitch}")
                    # print(f"yaw = {yaw}")
                    # print(f"valid flag: {valid_flag}")
        # Estimate filter
        elif descriptor== 0x80:
            if small_packet[1]==0x12:
                #GPS Correlation Timestamp 
                message=small_packet[2:]            # Message DATA 
                GPS_time_of_week= message[0:8]      # data type: double, unit: Secconds
                GPS_week= message[8:10]             # data type: U16, unit: N/A
                valid_flag= message[10:12]          # data type: Valid Flags, unit: 0x0000 – Time Invalid    0x0001 – Time Valid
            elif small_packet[1]== 0x06:
                #Scaled Magnetometer Vector
                message=small_packet[2:]            # Message DATA  
                X_mag= message[0:4]                 # data type: float, unit: Gause
                Y_mag= message[4:8]                 # data type: float, unit: Gause
                Z_mag= message[8:12]                # data type: float, unit: Gause
            elif small_packet[1]== 0x05:
                # Scaled Gyro Vector
                message=small_packet[2:]            # Message DATA  
                X_gyro= message[0:4]                # data type: float, unit: Rad/s
                Y_gyro= message[4:8]                # data type: float, unit: Rad/s
                Z_gyro= message[8:12]               # data type: float, unit: Rad/s
            elif small_packet[1]== 0x04:
                # Scaled Accelerometer Vector
                X_accel= message[0:4]                # data type: float, unit: g
                Y_accel= message[4:8]                # data type: float, unit: g
                Z_accel= message[8:12]               # data type: float, unit: g
        offset += packet_length
try:
    while True:
        # Đọc dữ liệu từ cổng serial
        byte = ser.read_until(MSG_START_BYTE)
        # print(byte.hex())
        # print(hex((byte[0])))
        # print(byte.hex())
        if byte:
            if (byte[0] == 0x82 or byte[0] == 0x80):
                payload_length = byte[1]
                Packet_payload = byte[2:payload_length+2]
                Descriptor_Set_byte=byte[0]
                classify_packet(Descriptor_Set_byte, Packet_payload)
        time.sleep(0.05)
                # print(f"payload_length: {payload_length}")
                # print(f"Packet_payload: {Packet_payload.hex()}")
                # print("=================================")
                

        # if byte:
           
        #     if len(buffer) >= 6:  
        #         while len(buffer) >= 6:
        #             if buffer[0] == 0x75 and buffer[1] == 0x65 and (buffer[2] == 0x82 or buffer[2] == 0x80):
        #                 payload_length = buffer[3]
        #                 total_length = 4 + payload_length + 2  

        #                 if len(buffer) >= total_length:
        #                     packet = buffer[:total_length]
        #                     # for b in packet:
        #                     #     print(f'0x{b:02x}', end=' ')
        #                     Descriptor_Set_byte=packet[2]
        #                     Packet_payload=packet[4:-2]         
        #                     # print(Descriptor_Set_byte)
        #                     # print(Packet_payload.hex())
        #                     classify_packet(Descriptor_Set_byte, Packet_payload)
        #                     print()

        #                     # Xóa gói dữ liệu đã xử lý khỏi buffer
        #                     buffer = buffer[total_length:]
        #                 else:
        #                     break
        #             else:
        #                 buffer.pop(0)

except KeyboardInterrupt:
    print("Program interrupted")
finally:
    # Đóng kết nối serial
    ser.close()
    print("Serial port closed")
