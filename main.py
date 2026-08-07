from vehicle import Vehicle
from controller import AdaptiveCruiseController, CruiseController

# Your car
car = Vehicle(max_traction_force=9200)
car.velocity = 25.0
my_controller = AdaptiveCruiseController(target_speed=30.0, time_gap=2.0)

# Lead car - starts well ahead
lead_car = Vehicle(max_traction_force=9200)
lead_car.velocity = 25.0
lead_car.position = 60.0  # starts 60m ahead
lead_controller = CruiseController(target_speed=25.0, Kp=0.15, Ki=0.03, Kd=0.1)
dt = 0.1
duration = 60
print_interval = 10

print(f"{'Time':>6} {'Car(mph)':>10} {'Lead(mph)':>10} {'Gap(m)':>8} {'Throttle':>9} {'Brake':>7}")
print("-" * 60)

for step in range(int(duration / dt)):
    t = step * dt

   # Lead car phase logic - designed to test every behaviour
    if t < 8:
        lead_controller.target_speed = 35.0      # fast - lets our car sit at its 67mph ceiling
    elif t < 16:
        lead_controller.target_speed = 18.0      # slows down - forces gap-following
    elif t < 22:
        lead_controller.target_speed = 18.0      # holds slow - steady gap-following
    elif t < 30:
        lead_controller.target_speed = 34.0      # speeds back up past our ceiling - ceiling test
    elif t < 36:
        lead_controller.target_speed = 34.0      # holds fast - our car should hold at 67mph
    elif t < 40:
        lead_controller.target_speed = 8.0       # SUDDEN hard brake - emergency response test
    elif t < 46:
        lead_controller.target_speed = 8.0       # holds very slow - close gap-following
    elif t < 54:
        lead_controller.target_speed = 30.0      # accelerates again - another cruise/gap cycle
    else:
        lead_controller.target_speed = 15.0      # final decel - one more braking event

    # Update lead car
    lead_throttle, lead_brake = lead_controller.compute(lead_car.velocity, dt)
    lead_car.update(lead_throttle, lead_brake, 0.0, dt)

    # Update your car (adaptive)
    throttle, brake = my_controller.compute(car.velocity, lead_car.position, car.position, dt)
    car.update(throttle, brake, 0.0, dt)

    # Print periodically
    if step % print_interval == 0:
        gap = lead_car.position - car.position
        print(f"{t:>6.1f} {car.get_state()['speed_mph']:>10.2f} {lead_car.get_state()['speed_mph']:>10.2f} {gap:>8.1f} {throttle:>9.2f} {brake:>7.2f} {lead_throttle:>9.2f}")