# sensors/lidar.py
# Real Lidar driver must be implemented from a reverse-engineered LDS protocol

class XiaomiLidar:
    def __init__(self, port="/dev/ttyUSB0"):
        # Replace with serial init + decoder
        pass

    def get_scan(self):
        # TODO: parse real serial output
        return [1000 for _ in range(360)]
