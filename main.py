from vehicle import Vehicle
from controller import AdaptiveCruiseController, CruiseController
import matplotlib.pyplot as plt

def run_scenario(scenario_name, phase_function, my_target_speed=30.0, lead_target_speed=60.0, road_grade=0.0, cutin_gap=15.0, cutin_time=None, lead_start_position=60.0):
    # Your car
    car = Vehicle(max_traction_force=9200)
    car.velocity = 25.0
    my_controller = AdaptiveCruiseController(target_speed=30.0, time_gap=2.0)

    # Lead car - starts well ahead
    lead_car = Vehicle(max_traction_force=9200)
    lead_car.velocity = 25.0
    lead_car.position = lead_start_position  
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

    for step in range(int(duration / dt)):
        t = step * dt
        if cutin_time is not None and abs(t - cutin_time) < dt/2:
            lead_car.position = car.position + cutin_gap
        lead_controller.target_speed = phase_function(t)
        # Update lead car
        lead_throttle, lead_brake = lead_controller.compute(lead_car.velocity, dt)
        lead_car.update(lead_throttle, lead_brake, road_grade, dt)

        # Update your car (adaptive)
        throttle, brake = my_controller.compute(car.velocity, lead_car.position, car.position, dt)
        car.update(throttle, brake, road_grade, dt)

        actual_gap_history.append(lead_car.position - car.position)
        time_history.append(t)
        desired_gap_history.append(my_controller.time_gap * car.velocity)
        control_effort_history.append(throttle - brake)
        lead_speed_history.append(lead_car.get_state()['speed_mph'])
        car_speed_history.append(car.get_state()['speed_mph'])


    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    fig.suptitle(scenario_name, fontsize=14)

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


def traffic_slowdown_phases(t):
    # Lead car phase logic - designed to test every behaviour
    if t < 10:
        return 32.0      # fast - lets our car sit at its 67mph ceiling
    elif t < 20:
        return 22.0      # slows down - forces gap-following
    elif t < 28:
        return 22.0      # holds slow - steady gap-following
    elif t < 30:
        return 10.0      # speeds back up past our ceiling - ceiling test
    elif t < 38:
        return 10.0      # holds fast - our car should hold at 67mph
    elif t < 40:
        return 40.0      # slows down - forces gap-following
    elif t < 52:
        return 24.0      # holds slow - steady gap-following
    elif t < 54:
        return 12.0      # speeds back up past our ceiling - ceiling test    
    else:
        return 12.0      # final decel - one more braking event

def highway_cruise_phases(t):
    if t < 15:
        return 28.0
    elif t < 35:
        return 30.0
    else:
        return 29.0

def emergency_stop_phases(t):
    if t < 6:
        return 35.0
    elif t < 6.5:
        return 2.0
    else:
        return 2.0
    
def cutin_phases(t):
    return 22.0

def hillclimb_phases(t):
    if t < 15:
        return 26.0
    elif t < 30:
        return 18.0
    elif t < 45:
        return 26.0
    else:
        return 20.0

#run_scenario("Highway Cruise", highway_cruise_phases)
#run_scenario("Traffic Slowdown", traffic_slowdown_phases)
run_scenario("Emergency Stop", emergency_stop_phases, lead_start_position=20.0)
#run_scenario("Cut-in", cutin_phases, cutin_time=20.0, cutin_gap=15.0)
#run_scenario("Hill Climb", hillclimb_phases, road_grade=0.08)