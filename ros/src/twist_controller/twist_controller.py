import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                 accel_limit, wheel_radius, wheel_base, steer_ratio,
                 max_lat_accel, max_steer_angle):

        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1,
                                            max_lat_accel, max_steer_angle)

        kp = 0.3
        ki = 0.1
        kd = 0.0
        mn = 0.0 # min throttle value
        mx = 0.2 # max throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.5 # 1/(2pi*tau) = cutoff freq, = 0.318 rad/s, or 2Hz
        ts = 0.02 # dt for 50 Hz
        self.vel_lpf = LowPassFilter(tau, ts) # for velocity noise

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle

        self.last_time = rospy.get_time()

    # def control(self, *args, **kwargs):
    def control(self, dbw_enabled, current_vel, linear_vel_cmd, angular_vel_cmd):

        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        current_vel_filtered = self.vel_lpf.filt(current_vel)

        # rospy.logwarn("Angular vel: {0}".format(angular_vel_cmd))
        # rospy.logwarn("Target velocity: {0}".format(linear_vel_cmd))
        # rospy.logwarn("Target angular velocity: {0}\n".format(angular_vel_cmd))
        # rospy.logwarn("Current velocity: {0}".format(current_vel))
        # rospy.logwarn("Filtered velocity: {0}".format(self.vel_lpf.get()))

        steering = self.yaw_controller.get_steering(linear_vel_cmd, angular_vel_cmd,
                                                    current_vel_filtered)

        vel_error = linear_vel_cmd - current_vel_filtered
        self.last_vel = current_vel_filtered

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        # to hold car in place if we're stopped
        if linear_vel_cmd == 0. and current_vel_filtered < 0.1:
            throttle = 0
            brake = 700 # N*m

        # PID controller is trying to slow the car down, convert to braking
        elif throttle < 0.1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius # Torque N*m

        return throttle, brake, steering
        # return 0.1, 0., 0.
