import serial
import struct
import time
import uuid
import os
import spidev
import datetime

# Setup the serial connection for communication with the flight controller
ser = serial.Serial('/dev/ttyS4', 115200, timeout=0.1)

# Setup SPI for communication with the angle sensor
spi = spidev.SpiDev()
spi.open(3, 0)
spi.max_speed_hz = 1000000  # Set max SPI speed
spi.mode = 0b01  # Set SPI mode

# Create a unique filename and ensure the directory exists
filename = f"telemetry_data_{uuid.uuid4().hex}.txt"
save_directory = '/home/radxa/monopod'
os.makedirs(save_directory, exist_ok=True)
file_path = os.path.join(save_directory, filename)
print(f"Saving data to: {file_path}")

# Record the start time
start_time = datetime.datetime.now()

def send_msp_command(cmd, data=None):
    if data is None:
        data = []
    packet = b'$M<' + struct.pack('<BB', len(data) * 2, cmd)
    packet += struct.pack('<' + 'H' * len(data), *data)
    checksum = 0
    for b in packet[3:]:
        checksum ^= b
    packet += bytes([checksum])
    ser.write(packet)
    response = ser.read(100)
    return response

def set_motor_pwm(motor_pwms):
    # Set the PWM values for all motors (assuming a quadcopter with 4 motors)
    cmd = 214  # MSP_SET_MOTOR
    data = motor_pwms  # Provide PWM values for all motors, even if you're only updating one or two
    send_msp_command(cmd, data)

def parse_battery_data(data):
    """ Parse battery-related telemetry from raw MSP response """
    if len(data) < 11:
        print("Battery data packet too short")
        return None
    payload = data[5:]  # Extract payload
    try:
        voltage = struct.unpack('<B', payload[0:1])[0] / 10.0
        mah_drawn = struct.unpack('<H', payload[1:3])[0]
        rssi = struct.unpack('<H', payload[3:5])[0]
        amperage = struct.unpack('<H', payload[5:7])[0] / 100.0
        return {
            'voltage': voltage,
            'amperage': amperage,
            'mah_drawn': mah_drawn,
            'rssi': rssi
        }
    except struct.error:
        print("Error unpacking battery data")
        return None

def get_imu_data():
    """ Retrieve and format IMU data from flight controller """
    response = send_msp_command(102)
    if len(response) >= 18:
        imu = struct.unpack('<9h', response[5:23])
        return ' '.join(map(str, imu))
    return "N/A N/A N/A N/A N/A N/A"

def get_attitude():
    """ Retrieve and format attitude data (pitch, roll, yaw) """
    response = send_msp_command(108)
    if len(response) >= 8:
        attitude = struct.unpack('<3h', response[5:11])
        return ' '.join(str(x / 10.0) for x in attitude)
    return "N/A N/A N/A"

def get_motor_pwm():
    """ Retrieve PWM values for each motor """
    response = send_msp_command(104)
    if len(response) >= 13:
        motors = struct.unpack('<HHHH', response[5:13])
        return motors[0], motors[1], motors[2], motors[3]
    return None, None, None, None

def get_motor_rpm():
    """ Retrieve RPM values for each motor """
    response = send_msp_command(139)  # Ensure this is the correct MSP command for your setup
    
    expected_response_length = 57  # Adjust this if your setup expects a different length
    if len(response) < expected_response_length:
        print(f"Insufficient data for RPM telemetry. Expected at least {expected_response_length} bytes, got {len(response)} bytes.")
        return [0, 0, 0, 0]

    rpms = []
    for i in range(4):
        start = 5 + i * 14  # Adjust based on the data packet structure
        end = start + 2
        if len(response[start:end]) == 2:  # Ensure there are 2 bytes to unpack
            rpm = struct.unpack('<H', response[start:end])[0]
            rpms.append(rpm)
        else:
            print(f"Warning: Could not unpack RPM for motor {i + 1}.")
            rpms.append(0)
    return rpms

def read_angle():
    """ Read raw angle from SPI connected sensor """
    response = spi.xfer2([0xFF, 0xFF])
    raw_angle = ((response[0] << 8) | response[1]) & 0x3FFF
    return raw_angle

def raw_to_degrees(raw):
    """ Convert raw sensor data to degrees """
    return (raw / 16383) * 360

try:
    with open(file_path, 'w') as file:
        print("Arming the drone...")
        send_msp_command(216, [1])  # MSP_SET_ARMED
        time.sleep(2)  # Wait to ensure the drone is armed

        # Set PWM to 1600 for motor 1 and 1400 for motor 3 (opposite directions)
        motor_pwms = [1600, 1500, 1400, 1500]  # PWM values for motors 1, 2, 3, 4 respectively
        set_motor_pwm(motor_pwms)
        print("Motors running...")

        # Run motors for 5 seconds
        run_duration = 5  # seconds
        data_logging_extension = 4  # seconds to continue logging after motors stop
        total_duration = run_duration + data_logging_extension
        start_time = datetime.datetime.now()

        while (datetime.datetime.now() - start_time).total_seconds() < total_duration:
            current_time = datetime.datetime.now()
            elapsed_time = int((current_time - start_time).total_seconds() * 1000)  # Time in milliseconds

            # Retrieve data from various sensors
            imu_data = get_imu_data()
            attitude = get_attitude()
            response = send_msp_command(110)
            battery_data = parse_battery_data(response)
            pwm_motor1, pwm_motor2, pwm_motor3, pwm_motor4 = get_motor_pwm()
            rpms = get_motor_rpm()  # Get RPM data

            # Convert angle to degrees
            raw_angle = read_angle()
            angle_in_degrees = raw_to_degrees(raw_angle)

            # Format all collected data into a single string
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
            data_line += f"{elapsed_time}\n"  # Time in milliseconds

            # Print and save data line to the file
            print(data_line)
            file.write(data_line)
            file.flush()
            os.fsync(file.fileno())

            time.sleep(0.001)  # Reduced sleep time for faster data collection

        # Stop motors after the run duration
        motor_pwms = [1500, 1500, 1500, 1500]  # Reset all motors to neutral
        set_motor_pwm(motor_pwms)
        print("Motors stopped.")

        time.sleep(1)  # Small delay before disarming

        print("Disarming the drone...")
        send_msp_command(216, [0])  # MSP_SET_DISARMED

except KeyboardInterrupt:
    print("Script terminated by user")
except serial.SerialException as e:
    print(f"Serial port error: {e}")
except Exception as e:
    print(f"An unexpected error occurred: {e}")
finally:
    # Ensure all motors are stopped and close connections
    motor_pwms = [1500, 1500, 1500, 1500]  # Reset all motors to neutral
    set_motor_pwm(motor_pwms)
    time.sleep(0.5)
    ser.close()
    spi.close()
    print("Connections closed and motors stopped")
    print(f"Data saved to: {file_path}")
