class WebotsEncoderAdapter:
    def __init__(self, left_encoder, right_encoder, wheel_radius, wheel_distance):
        self.left_encoder = left_encoder
        self.right_encoder = right_encoder
        self.wheel_radius = wheel_radius
        self.wheel_distance = wheel_distance

    def enable(self, timestep):
        self.left_encoder.enable(timestep)
        self.right_encoder.enable(timestep)

    def get_data(self):
        left_distance = self.left_encoder.getValue() * self.wheel_radius
        right_distance = self.right_encoder.getValue() * self.wheel_radius
        velocity = (left_distance + right_distance) / 2.0
        dtheta = (right_distance - left_distance) / self.wheel_distance

        return {
            'left_distance': left_distance,
            'right_distance': right_distance,
            'velocity': velocity,
            'dtheta': dtheta
        }