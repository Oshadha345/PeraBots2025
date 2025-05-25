# sensors/imu.py

import smbus
import time

class MPU6050:
    
    def __init__(self, address=0x68, bus_id=1):
        self.address = address
        self.bus = smbus.SMBus(bus_id)
        self.bus.write_byte_data(self.address, 0x6B, 0)  # Wake up

    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr+1)
        value = ((high << 8) | low)
        if value > 32768:
            value = value - 65536
        return value

    def get_accel_data(self):
        return {
            "x": self.read_raw_data(0x3B) / 16384.0,
            "y": self.read_raw_data(0x3D) / 16384.0,
            "z": self.read_raw_data(0x3F) / 16384.0
        }

    def get_gyro_data(self):
        return {
            "x": self.read_raw_data(0x43) / 131.0,
            "y": self.read_raw_data(0x45) / 131.0,
            "z": self.read_raw_data(0x47) / 131.0
        }
