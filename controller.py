class CruiseController:
    def __init__(self, Kp=100, Ki=5, Kd=20,target_speed=30, max_integral=100):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target_speed = target_speed  # in m/s
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.max_integral = max_integral  # to prevent integral windup
    def compute(self,current_speed, dt):
        error = self.target_speed - current_speed
        self.integral_error += error * dt
        self.integral_error = max(min(self.integral_error, self.max_integral), -self.max_integral)
        derivative= (error - self.previous_error) / dt if dt > 0 else 0.0
        control_output = (self.Kp * error) + (self.Ki * self.integral_error) + (self.Kd * derivative)
        if control_output >= 0.0:
            throttle = min(1.0, control_output/self.Kp)
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(1.0, -control_output/self.Kp)
        self.previous_error = error
        return throttle, brake