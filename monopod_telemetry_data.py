import serial
import struct
import time
import uuid
import os
import spidev

# Open serial port
ser = serial.Serial('/dev/ttyS4', 115200, timeout=0.1)  # Shorter timeout for faster data reading

# Setup SPI
spi = spidev.SpiDev()
spi.open(3, 0)  # Open SPI bus 3, device 0
spi.max_speed_hz = 1000000  # 1 MHz
spi.mode = 0b01  # SPI mode 1, as required by AS5048A

# Create a unique filename for the output file
filename = f"telemetry_data_{uuid.uuid4().hex}.txt"

# Define the directory to save the file
save_directory = '/home/radxa/control'  # Adjust path as needed
os.makedirs(save_directory, exist_ok=True)

# Open the file for writing
file_path = os.path.join(save_directory, filename)
print(f"Saving data to: {file_path}")  # Debugging output

def send_msp_command(cmd):
    packet = b'$M<' + struct.pack('<BB', 0, cmd)
    checksum = 0
    for b in packet[3:]:
        checksum ^= b
    packet += bytes([checksum])
    ser.write(packet)
    response = ser.read(100)
    return response

def parse_battery_data(data):
    if len(data) < 11:
        print("Battery data packet too short")  # Debug output
        return None
    payload = data[5:]
    try:
        voltage = struct.unpack('<B', payload[0:1])[0]
        mah_drawn = struct.unpack('<H', payload[1:3])[0]
        rssi = struct.unpack('<H', payload[3:5])[0]
        amperage = struct.unpack('<H', payload[5:7])[0]
        return {
            'voltage': voltage / 10.0,
            'amperage': amperage / 100.0,
            'mah_drawn': mah_drawn,
            'rssi': rssi
        }
    except struct.error:
        print(f"Error unpacking battery data: {payload.hex()}")  # Debug output
        return None

def get_imu_data():
    response = send_msp_command(102)
    if len(response) >= 18:
        imu = struct.unpack('<9h', response[5:23])
        return f"{imu[0]} {imu[1]} {imu[2]} {imu[3]} {imu[4]} {imu[5]}"
    return "N/A N/A N/A N/A N/A N/A"

def get_attitude():
    response = send_msp_command(108)
    if len(response) >= 8:
        attitude = struct.unpack('<3h', response[5:11])
        return f"{attitude[0]/10.0} {attitude[1]/10.0} {attitude[2]}"
    return "N/A N/A N/A"

def get_motor_pwm():
    response = send_msp_command(104)
    if len(response) >= 13:
        motors = struct.unpack('<HHHH', response[5:13])
        return motors[0], motors[1], motors[2], motors[3]
    return None, None, None, None

def read_angle():
    response = spi.xfer2([0xFF, 0xFF])
    raw_angle = ((response[0] << 8) | response[1]) & 0x3FFF
    return raw_angle

def raw_to_degrees(raw):
    return (raw / 16383) * 360

def msp_encode(cmd, data=None):
    if data is None:
        data = []
    size = len(data)
    packet = ['$'.encode('ascii'), 'M'.encode('ascii'), '<'.encode('ascii'), size, cmd]
    packet.extend(data)
    checksum = 0
    for i in range(3, len(packet)):
        checksum ^= packet[i]
    packet.append(checksum)
    return struct.pack('<3c2B%dB' % (len(data) + 1), *packet)

def msp_decode(packet):
    if len(packet) < 6:
        return None, None
    if packet[0:3] != b'$M>':
        return None, None
    data_length = packet[3]
    cmd = packet[4]
    data = packet[5:-1]
    return cmd, data

def get_motor_telemetry(ser):
    ser.write(msp_encode(139))  # 139 is MSP_MOTOR_TELEMETRY
    response = ser.read(1024)
    cmd, data = msp_decode(response)
    
    if cmd == 139:
        motor_count = len(data) // 14
        rpms = [0] * 4
        for i in range(motor_count):
            start = i * 14
            end = start + 14
            motor_data = data[start:end]
            actual_motor = i + 2 if i < 3 else 1  # Map 0->2, 1->3, 2->4, 3->1
            
            if len(motor_data) >= 2:
                rpm = struct.unpack('<H', motor_data[-2:])[0]
                rpms[actual_motor - 1] = rpm
        
        return rpms
    return [0, 0, 0, 0]

try:
    with open(file_path, 'w') as file:
        while True:
            imu_data = get_imu_data()
            attitude = get_attitude()
            
            response = send_msp_command(110)
            battery_data = parse_battery_data(response)
            
            pwm_motor1, pwm_motor2, pwm_motor3, pwm_motor4 = get_motor_pwm()
            
            # Get RPM data
            rpms = get_motor_telemetry(ser)
            
            # Read angle in degrees
            raw_angle = read_angle()
            angle_in_degrees = raw_to_degrees(raw_angle)
            
            # Prepare data line
            data_line = f"{imu_data}\n{attitude}\n"
            if battery_data:
                data_line += f"{battery_data['voltage']:.2f} {battery_data['amperage']:.2f} {battery_data['mah_drawn']}\n"
            else:
                data_line += "N/A N/A N/A\n"
            
            if all(pwm is not None for pwm in [pwm_motor1, pwm_motor2, pwm_motor3, pwm_motor4]):
                data_line += f"{pwm_motor1} {pwm_motor2} {pwm_motor3} {pwm_motor4}\n"
            else:
                data_line += "N/A N/A N/A N/A\n"
            
            data_line += f"{rpms[0]} {rpms[1]} {rpms[2]} {rpms[3]}\n"  # RPM data
            data_line += f"{angle_in_degrees:.2f}\n"  # Angle data
            
            # Print and save data line to the file
            print(data_line)
            file.write(data_line)
            file.write("\n")
            
            # Flush file to ensure data is written
            file.flush()
            os.fsync(file.fileno())
            
            # Reduced sleep time for faster loop iterations
            time.sleep(0.01)

except KeyboardInterrupt:
    print("Script terminated by user")
except serial.SerialException as e:
    print(f"Serial port error: {e}")
except Exception as e:
    print(f"An unexpected error occurred: {e}")
finally:
    ser.close()
    spi.close()  # Ensure SPI is closed properly
    print("Serial port closed")
    print(f"Data saved to: {file_path}")