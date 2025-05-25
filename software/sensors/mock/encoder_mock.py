# sensors/mock/encoder_mock.py

class MockEncoder:
    def __init__(self):
        self.ticks = 0

    def get_ticks(self):
        self.ticks += 1
        return (self.ticks, self.ticks)

    def get_distance(self):
        return self.ticks * 0.01  # 1 tick = 1cm
