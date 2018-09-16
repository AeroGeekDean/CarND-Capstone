from math import atan

class YawController(object):
    def __init__(self, wheel_base, steer_ratio, min_speed,
                 max_lat_accel, max_steer_angle):
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel

        self.min_angle = -max_steer_angle
        self.max_angle =  max_steer_angle

    # def get_angle(self, radius):
    #     angle = atan(self.wheel_base / radius) * self.steer_ratio
    #     return max(min(angle, self.max_angle), self.min_angle)

    def get_steering(self, linear_vel_cmd, angular_vel_cmd, linear_vel_current):
        if abs(linear_vel_cmd) > 0.:
            psi_dot_cmd = angular_vel_cmd * (linear_vel_current/linear_vel_cmd)
        else:
            psi_dot_cmd = 0.0

        if abs(linear_vel_current) > 0.1:
            max_yaw_rate = abs(self.max_lat_accel / linear_vel_current);
            psi_dot_cmd = max(min(psi_dot_cmd, max_yaw_rate), -max_yaw_rate)

        vel_limited = max(linear_vel_current, self.min_speed)
        angle = atan(psi_dot_cmd * self.wheel_base / vel_limited) * self.steer_ratio
        angle = max(min(angle, self.max_angle), self.min_angle)
        return angle
