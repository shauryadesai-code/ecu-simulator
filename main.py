from vehicle import Vehicle
from controller import AdaptiveCruiseController, CruiseController

# Your car
car = Vehicle(max_traction_force=9200)
car.velocity = 25.0
my_controller = AdaptiveCruiseController(target_speed=30.0, time_gap=2.0)

# Lead car - starts well ahead
lead_car = Vehicle(max_traction_force=9200)
lead_car.velocity = 25.0
lead_car.position = 60.0  # starts 30m ahead
lead_controller = CruiseController(target_speed=25.0, Kp=0.15, Ki=0.03, Kd=0.1)
time_history = []
car_speed_history = []
lead_speed_history = []
actual_gap_history = []
desired_gap_history = []
control_effort_history = []
dt = 0.1
duration = 60
print_interval = 10

print(f"{'Time':>6} {'Car(mph)':>10} {'Lead(mph)':>10} {'Gap(m)':>8} {'Throttle':>9} {'Brake':>7}")
print("-" * 60)

for step in range(int(duration / dt)):
    t = step * dt

   # Lead car phase logic - designed to test every behaviour
    if t < 10:
        lead_controller.target_speed = 32.0      # fast - lets our car sit at its 67mph ceiling
    elif t < 20:
        lead_controller.target_speed = 22.0      # slows down - forces gap-following
    elif t < 28:
        lead_controller.target_speed = 22.0      # holds slow - steady gap-following
    elif t < 30:
        lead_controller.target_speed = 10.0      # speeds back up past our ceiling - ceiling test
    elif t < 38:
        lead_controller.target_speed = 10.0      # holds fast - our car should hold at 67mph
    elif t < 40:
        lead_controller.target_speed = 40.0      # slows down - forces gap-following
    elif t < 52:
        lead_controller.target_speed = 24.0      # holds slow - steady gap-following
    elif t < 54:
        lead_controller.target_speed = 12.0      # speeds back up past our ceiling - ceiling test    
    else:
        lead_controller.target_speed = 12.0      # final decel - one more braking event

    # Update lead car
    lead_throttle, lead_brake = lead_controller.compute(lead_car.velocity, dt)
    lead_car.update(lead_throttle, lead_brake, 0.0, dt)

    # Update your car (adaptive)
    throttle, brake = my_controller.compute(car.velocity, lead_car.position, car.position, dt)
    car.update(throttle, brake, 0.0, dt)
    actual_gap_history.append(lead_car.position - car.position)
    time_history.append(t)
    desired_gap_history.append(my_controller.get_desired_gap(car.velocity))
    control_effort_history.append(throttle-brake)
    lead_speed_history.append(lead_car.get_state()['speed_mph'])
    car_speed_history.append(car.get_state()['speed_mph'])

    # Print periodically
    if step % print_interval == 0:
        gap = lead_car.position - car.position
        print(f"{t:>6.1f} {car.get_state()['speed_mph']:>10.2f} {lead_car.get_state()['speed_mph']:>10.2f} {gap:>8.1f} {throttle:>9.2f} {brake:>7.2f} {lead_throttle:>9.2f}")

import matplotlib.pyplot as plt

fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

# Graph 1 - Speed comparison
axes[0].plot(time_history, car_speed_history, label='Your Car', color='blue')
axes[0].plot(time_history, lead_speed_history, label='Lead Car', color='red')
axes[0].set_ylabel('Speed (mph)')
axes[0].set_title('Speed Comparison')
axes[0].legend()
axes[0].grid(True)

# Graph 2 - Gap
axes[1].plot(time_history, actual_gap_history, label='Actual Gap', color='green')
axes[1].plot(time_history, desired_gap_history, label='Desired Gap', color='orange', linestyle='--')
axes[1].set_ylabel('Gap (m)')
axes[1].set_title('Following Distance')
axes[1].legend()
axes[1].grid(True)

# Graph 3 - Control effort
axes[2].plot(time_history, control_effort_history, label='Throttle(+) / Brake(-)', color='purple')
axes[2].axhline(0, color='black', linewidth=0.5)
axes[2].set_ylabel('Control Effort')
axes[2].set_xlabel('Time (s)')
axes[2].set_title('Throttle/Brake Output')
axes[2].legend()
axes[2].grid(True)

plt.tight_layout()
plt.show()