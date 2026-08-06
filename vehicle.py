import numpy as np
class Vehicle:
    def __init__(self, mass=1595, frontal_area=2.07, drag_coefficient=0.36, rolling_resistance_coefficient=0.013, wheel_radius=0.335, max_drive_force=5000,max_brake_force=22000, v_max=85):
        self.mass = mass  # in kg
        self.frontal_area = frontal_area  # in m^2
        self.drag_coefficient = drag_coefficient  # dimensionless
        self.rolling_resistance_coefficient = rolling_resistance_coefficient  # dimensionless
        self.wheel_radius = wheel_radius  # in m
        self.max_drive_force = max_drive_force  # in N
        self.max_brake_force = max_brake_force  # in N
        self.air_density = 1.225  # in kg/m^3, at sea level
        self.gravity = 9.81  # in m/s^2
        self.velocity = 0.0  # in m/s
        self.position = 0.0  # in m
        self.v_max = v_max  # in m/s
    def update(self, throttle, brake, road_grade, dt):
        #throttle: 0 to 1, brake: 0 to 1, road_grade: in radians, dt: time step in seconds
        drag_force = 0.5 * self.air_density * self.frontal_area * self.drag_coefficient * self.velocity ** 2
        rolling_resistance_force = self.rolling_resistance_coefficient * self.mass * self.gravity * np.cos(road_grade)
        grade_force = self.mass * self.gravity * np.sin(road_grade)
        drive_force = throttle * self.max_drive_force * max(0, 1 - self.velocity / self.v_max)
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
            'v_max_m/s': self.v_max
        }