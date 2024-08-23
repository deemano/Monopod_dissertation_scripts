import spidev
import time

# Setup SPI
spi = spidev.SpiDev()
spi.open(3, 0)  # Open SPI bus 3, device 0
spi.max_speed_hz = 1000000  # 1 MHz
spi.mode = 0b01  # SPI mode 1, as required by AS5048A


def read_angle():
    response = spi.xfer2([0xFF, 0xFF])
    raw_angle = ((response[0] << 8) | response[1]) & 0x3FFF
    return raw_angle


def raw_to_degrees(raw):
    return (raw / 16383) * 360


try:
    while True:
        raw_angle = read_angle()
        angle_in_degrees = raw_to_degrees(raw_angle)
        print(f"Angle: {raw_angle} raw, {angle_in_degrees:.2f} degrees")
        time.sleep(1)  # Read every second
except KeyboardInterrupt:
    print("Exiting program.")
finally:
    spi.close()  # Close the SPI connection
