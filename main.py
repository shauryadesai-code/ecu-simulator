from vehicle import Vehicle
from controller import AdaptiveCruiseController, CruiseController
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Polygon

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
    car_position_history = []
    lead_position_history = []
    car_state_history = []
    lead_state_history = []
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
        car_position_history.append(car.position)
        lead_position_history.append(lead_car.position)
        if throttle > 0:
            car_state_history.append('accelerating')
        elif brake > 0:
            car_state_history.append('braking')
        else:
            car_state_history.append('neutral')

        if lead_throttle > 0:
            lead_state_history.append('accelerating')
        elif lead_brake > 0:
            lead_state_history.append('braking')
        else:
            lead_state_history.append('neutral')


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
    return car_position_history, lead_position_history, time_history, car_state_history, lead_state_history

def animate_scenario(scenario_name, car_position_history, lead_position_history, time_history,
                      car_state_history, lead_state_history, two_lane=False, cutin_time=None):
    fig, ax = plt.subplots(figsize=(12, 3))
    ax.set_xlim(min(car_position_history) - 10, max(lead_position_history) + 10)
    ax.set_ylim(-6, 6)
    ax.set_title(scenario_name)
    ax.set_xlabel('Position (m)')
    ax.set_yticks([])

    ax.axhline(0, color='gray', linewidth=1.5)
    if two_lane:
        ax.axhline(4, color='gray', linewidth=1.5)

    car_shape = [(-2.0, -1.0), (-2.0, 1.0), (1.0, 1.0), (2.0, 0.5), (2.0, -0.5), (1.0, -1.0)]

    state_colors = {'accelerating': 'green', 'braking': 'red', 'neutral': 'steelblue'}

    car_patch = Polygon(car_shape, closed=True, facecolor='steelblue', edgecolor='black')
    lead_patch = Polygon(car_shape, closed=True, facecolor='darkorange', edgecolor='black')
    ax.add_patch(car_patch)
    ax.add_patch(lead_patch)

    def get_lead_lane_offset(t):
        if not two_lane:
            return 0.0
        transition_duration = 2.0
        if cutin_time is None:
            return 4.0
        if t < cutin_time - transition_duration:
            return 4.0
        elif t < cutin_time:
            progress = (t - (cutin_time - transition_duration)) / transition_duration
            return 4.0 * (1 - progress)
        else:
            return 0.0

    def update(frame):
        car_x = car_position_history[frame]
        lead_x = lead_position_history[frame]
        t = time_history[frame]

        lead_y = get_lead_lane_offset(t)

        car_pts = [(x + car_x, y) for x, y in car_shape]
        lead_pts = [(x + lead_x, y + lead_y) for x, y in car_shape]

        car_patch.set_xy(car_pts)
        lead_patch.set_xy(lead_pts)

        car_patch.set_facecolor(state_colors[car_state_history[frame]])
        lead_patch.set_facecolor(state_colors[lead_state_history[frame]])

        return car_patch, lead_patch

    ani = animation.FuncAnimation(fig, update, frames=len(time_history), interval=50, blit=False)
    plt.show()
    return ani


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

car_pos, lead_pos, times, car_state, lead_state = run_scenario("Highway Cruise", highway_cruise_phases)
animate_scenario("Highway Cruise (Animated)", car_pos, lead_pos, times, car_state, lead_state)

car_pos, lead_pos, times, car_state, lead_state = run_scenario("Traffic Slowdown", traffic_slowdown_phases)
animate_scenario("Traffic Slowdown (Animated)", car_pos, lead_pos, times, car_state, lead_state)

car_pos, lead_pos, times, car_state, lead_state = run_scenario("Emergency Stop", emergency_stop_phases, lead_start_position=20.0)
animate_scenario("Emergency Stop (Animated)", car_pos, lead_pos, times, car_state, lead_state)

car_pos, lead_pos, times, car_state, lead_state = run_scenario("Cut-in", cutin_phases, cutin_time=20.0, cutin_gap=15.0)
animate_scenario("Cut-in (Animated)", car_pos, lead_pos, times, car_state, lead_state, two_lane=True, cutin_time=20.0)

car_pos, lead_pos, times, car_state, lead_state = run_scenario("Hill Climb", hillclimb_phases, road_grade=0.08)
animate_scenario("Hill Climb (Animated)", car_pos, lead_pos, times, car_state, lead_state)