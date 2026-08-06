import numpy as np
class Vehicle:
    def __init__(self, mass=1595, frontal_area=2.07, drag_coefficient=0.36, rolling_resistance_coefficient=0.013, wheel_radius=0.335, max_brake_force=22000, engine_power=221000, drivetrain_efficiency=0.88, max_traction_force=9200):
        self.mass = mass  # in kg
        self.frontal_area = frontal_area  # in m^2
        self.drag_coefficient = drag_coefficient  # dimensionless
        self.rolling_resistance_coefficient = rolling_resistance_coefficient  # dimensionless
        self.wheel_radius = wheel_radius  # in m
        self.max_brake_force = max_brake_force  # in N
        self.air_density = 1.225  # in kg/m^3, at sea level
        self.gravity = 9.81  # in m/s^2
        self.velocity = 0.0  # in m/s
        self.position = 0.0  # in m
        self.engine_power = engine_power  # in Watts
        self.drivetrain_efficiency = drivetrain_efficiency  # dimensionless
        self.max_traction_force = max_traction_force  # in N
        
    def update(self, throttle, brake, road_grade, dt):
        #throttle: 0 to 1, brake: 0 to 1, road_grade: in radians, dt: time step in seconds
        drag_force = 0.5 * self.air_density * self.frontal_area * self.drag_coefficient * self.velocity ** 2
        rolling_resistance_force = self.rolling_resistance_coefficient * self.mass * self.gravity * np.cos(road_grade)
        grade_force = self.mass * self.gravity * np.sin(road_grade)
        if self.velocity < 1.0:
         available_force = self.max_traction_force
        else:
         power_limited_force = (self.engine_power * self.drivetrain_efficiency) / self.velocity
         available_force = min(self.max_traction_force, power_limited_force)

        drive_force = throttle * available_force
        brake_force = brake * self.max_brake_force
        net_force = drive_force - drag_force - rolling_resistance_force - grade_force - brake_force
        acceleration = net_force / self.mass
        self.velocity += acceleration * dt
        if self.velocity < 0:
            self.velocity = 0
        self.position+= self.velocity * dt
    def get_state(self):
        return {
            'speed_m/s': self.velocity,
            'speed_mph': self.velocity * 2.237,
            'position_m': self.position,
        }