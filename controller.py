class CruiseController:
    def __init__(self, Kp=0.1, Ki=0.02, Kd=0.05,target_speed=30, max_integral=50):
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
        self.previous_error = error
        throttle = max(0.0, min(1.0, control_output))
        brake = max(0.0, min(1.0, -control_output))

        if current_speed >= self.target_speed:
           throttle = 0.0
        return throttle, brake
class AdaptiveCruiseController:
    def __init__(self, target_speed=30, time_gap=2.0, gap_Kp=0.03, gap_Ki=0.005, gap_Kd=0.05, gap_max_integral=50):
        self.cruise_controller = CruiseController(target_speed=target_speed)
        self.time_gap = time_gap  # desired time gap in seconds
        self.gap_Kp = gap_Kp
        self.gap_Ki = gap_Ki
        self.gap_Kd = gap_Kd
        self.gap_integral_error = 0.0
        self.previous_gap_error = 0.0
        self.gap_max_integral = gap_max_integral  # to prevent integral windup
    def compute(self, current_speed, lead_position, car_position, dt):
        actual_gap = lead_position - car_position
        desired_gap = current_speed * self.time_gap
        gap_error = actual_gap - desired_gap

        self.gap_integral_error += gap_error * dt
        self.gap_integral_error = max(min(self.gap_integral_error, self.gap_max_integral), -self.gap_max_integral)
        gap_derivative = (gap_error - self.previous_gap_error) / dt if dt > 0 else 0.0
        gap_control_output = (self.gap_Kp * gap_error) + (self.gap_Ki * self.gap_integral_error) + (self.gap_Kd * gap_derivative)
        self.previous_gap_error = gap_error

        if gap_control_output < 0:
           throttle = 0.0
           brake = max(0.0, min(1.0, -gap_control_output))
        else:
           throttle,brake = self.cruise_controller.compute(current_speed, dt)
           self.gap_integral_error = 0.0  # Reset integral error when not braking

        return throttle, brake
    def get_desired_gap(self, current_speed):
        return current_speed * self.time_gap