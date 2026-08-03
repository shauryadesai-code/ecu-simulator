from vehicle import Vehicle

# Create a Vehicle instance using the Jaguar F-Type defaults
car = Vehicle()

# Simulation parameters
dt = 0.1              # time step in seconds (10 Hz)
duration = 60         # total simulation duration in seconds
print_interval = 10   # print every 10 steps = every 1 second

print(f"{'Time (s)':>8} {'Speed (mph)':>12} {'Speed (m/s)':>12} {'Position (m)':>14}")
print("-" * 50)

# Run the simulation loop
for step in range(int(duration / dt)):
    t = step * dt

    # Full throttle for first 30 seconds, then coast
    if t < 30:
        throttle = 1.0
    else:
        throttle = 0.0

    brake = 0.0
    road_grade = 0.0  # flat road for this test

    # Update the vehicle state by one time step
    car.update(throttle, brake, road_grade, dt)

    # Print state periodically
    if step % print_interval == 0:
        state = car.get_state()
        print(f"{t:>8.1f} {state['speed_mph']:>12.2f} {state['speed_m/s']:>12.2f} {state['position_m']:>14.2f}")