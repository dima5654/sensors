from math import cos, sin, atan2

class Odometry:
    def __init__(self, wheel_radius=0.020, wheel_base=0.057, max_speed=6.28):
        # Robot parameters
        self.R = wheel_radius  # Wheel radius [m]
        self.D = wheel_base    # Distance between wheels [m]
        self.MAX_SPEED = max_speed

        # Robot state
        self.x = 0.0      # Position [m]
        self.y = 0.436    # Position [m]
        self.phi = 0.0531 # Orientation [rad]
        self.encoder_prev = [0.0, 0.0]  # [left, right]

    def update_encoders(self, encoder_left, encoder_right, delta_t):
        """Compute wheel speeds from encoder readings and update pose"""
        delta_left = encoder_left - self.encoder_prev[0]
        delta_right = encoder_right - self.encoder_prev[1]

        wl = delta_left / delta_t
        wr = delta_right / delta_t

        self.encoder_prev = [encoder_left, encoder_right]

        u = self.R * (wr + wl) / 2
        w = self.R * (wr - wl) / self.D

        delta_x = u * delta_t
        delta_phi = w * delta_t

        self.x += delta_x * cos(self.phi + delta_phi / 2)
        self.y += delta_x * sin(self.phi + delta_phi / 2)
        self.phi = atan2(sin(self.phi + delta_phi), cos(self.phi + delta_phi))

        return wl, wr, u, w, self.x, self.y, self.phi

    def get_wheel_speeds(self, u_desired, w_desired):
        """Convert linear and angular velocity to wheel speeds"""
        wr_d = (2 * u_desired + self.D * w_desired) / (2 * self.R)
        wl_d = (2 * u_desired - self.D * w_desired) / (2 * self.R)

        # Clamp speeds to MAX_SPEED
        max_wheel = max(abs(wl_d), abs(wr_d))
        if max_wheel > self.MAX_SPEED:
            scale = self.MAX_SPEED / max_wheel
            wl_d *= scale
            wr_d *= scale

        return wl_d, wr_d
