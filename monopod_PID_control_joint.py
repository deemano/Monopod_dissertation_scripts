import serial
import struct
import time
import spidev
import csv

# Setup the serial connection
ser = serial.Serial('/dev/ttyS4', 115200, timeout=0.1)

# Setup SPI for communication with AS5048A angle sensor
spi = spidev.SpiDev()
spi.open(3, 0)
spi.max_speed_hz = 1000000
spi.mode = 0b01

# PID Controller Class
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0
        self.integral = 0

    def calculate(self, setpoint, measurement, dt):
        error = (setpoint - measurement + 180) % 360 - 180  # Handle angle wrap-around
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative, error, self.integral, derivative

# Initialize the PID controller with tuned parameters
pid = PIDController(kp=2, ki=0.03, kd=0.05)

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

def set_motor_pwm(motor, pwm):
    cmd = 214  # MSP_SET_MOTOR
    data = [1000] * 4  # Initialize all motors to 1000 (minimum throttle)
    data[motor - 1] = pwm  # Set the specified motor's PWM
    send_msp_command(cmd, data)

def read_angle():
    num_samples = 5  # Number of samples for averaging
    total_angle = 0
    for _ in range(num_samples):
        response = spi.xfer2([0xFF, 0xFF])
        raw_angle = ((response[0] << 8) | response[1]) & 0x3FFF
        total_angle += (raw_angle / 16383) * 360
        time.sleep(0.005)  # Small delay between samples
    return total_angle / num_samples

def run_motor_by_angle(motor_number, initial_pwm, relative_angle, max_duration=10):
    # Read initial angle
    initial_angle = read_angle()
    target_angle = (initial_angle + relative_angle) % 360  # Calculate target angle with wrapping

    print(f"Starting motor {motor_number} at PWM {initial_pwm}, rotating by {relative_angle} degrees...")
    set_motor_pwm(motor_number, initial_pwm)

    start_time = time.time()
    last_time = start_time

    # Open CSV file for logging
    with open('motor_data_log.csv', 'w', newline='') as csvfile:
        fieldnames = ['time', 'current_angle', 'target_angle', 'pwm', 'error', 'integral', 'derivative']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        while True:
            current_time = time.time()
            elapsed = current_time - start_time

            if elapsed >= max_duration:
                print("Maximum duration exceeded, stopping motor.")
                break

            current_angle = read_angle()
            time_diff = current_time - last_time

            if time_diff >= 0.1:  # Control loop interval
                # Calculate the PWM adjustment using the PID controller
                pwm_adjustment, error, integral, derivative = pid.calculate(target_angle, current_angle, time_diff)
                new_pwm = initial_pwm + int(pwm_adjustment)
                new_pwm = max(1000, min(2000, new_pwm))  # Constrain PWM values
                set_motor_pwm(motor_number, new_pwm)
                last_time = current_time

                # Log data for analysis
                writer.writerow({
                    'time': elapsed,
                    'current_angle': current_angle,
                    'target_angle': target_angle,
                    'pwm': new_pwm,
                    'error': error,
                    'integral': integral,
                    'derivative': derivative
                })

                print(f"Time: {elapsed:.2f}s, Angle: {current_angle:.2f}°, PWM: {new_pwm}")

                # Check if the current angle is within 1 degree of the target angle
                if abs((current_angle - target_angle + 180) % 360 - 180) < 1:
                    print("Target angle reached.")
                    break

            time.sleep(0.05)  # Shorter sleep for more responsive control

    print("Stopping motor...")
    set_motor_pwm(motor_number, 1000)  # Stop the motor

    # Disarm the drone
    print("Disarming the drone...")
    send_msp_command(216, [0])  # MSP_SET_ARMED

try:
    # Read and display the initial angle
    initial_angle = read_angle()
    print(f"Initial angle before movement: {initial_angle:.2f}°")

    # Arm the drone (you may need to adjust this based on your setup)
    print("Arming the drone...")
    send_msp_command(216, [1])  # MSP_SET_ARMED
    time.sleep(2)  # Wait for arming to complete

    # Run motor 4 at PWM 1100 to rotate by 90 degrees from the current position
    run_motor_by_angle(motor_number=4, initial_pwm=1100, relative_angle=90, max_duration=10)

except KeyboardInterrupt:
    print("Script terminated by user")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    # Ensure all motors are stopped
    for i in range(1, 5):
        set_motor_pwm(i, 1000)
    time.sleep(0.5)  # Allow time for the command to be processed

    ser.close()
    spi.close()
    print("Connections closed and motors stopped")
