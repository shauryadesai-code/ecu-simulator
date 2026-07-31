import numpy as np
class Vehicle:
    def __init__(self, mass=1595, frontal_area=2.07, drag_coefficient=0.36, rolling_resistance_coefficient=0.013, wheel_radius=0.335, max_drive_force=16300,max_brake_force=22000):
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
    