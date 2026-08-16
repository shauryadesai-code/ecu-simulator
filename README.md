# ECU / Adaptive Cruise Control Simulator

A Python simulation of vehicle longitudinal dynamics and a PID-based 
adaptive cruise control system, built to understand real automotive 
control systems from first principles.

The simulator models a following vehicle's real world physics: drag, rolling resistance, and traction - using an ACC(adaptive cruise control) that holds a set speed, wathced for a lead vehicle aheas and automatically slows down or accelerate to keep a safe distance, which is speed distance. It smoothly switches from "just cruise" to "follow that car" depending on whats happening on the road.

## What this project demonstrates

- vehicle dynamics(drag, rolling resistance, traction, grade force) calibrated against an approximation of a Jaguar F-type P300
- A PID cruise controller with anti-windup handling
- An ACC that blends cruise and gap-following control continuously and smoothly rather than abrupt changes 
- Five test scenarios
- Static graphs and a top down animated view for each scenario

## How to run

### Prerequisites

- Python 3.10 or later
- pip

### Setup

1. clone respirotory
git clone https://github.com/shauryadesai-code/ecu-simulator.git
cd ecu simulator
2. create and activate a virtual environment (recommended):
python -m venv venv
venv\Scripts\activate # Windows
source venv/bin/activate # macOS/Linux
3. install dependancies
pip install requirements.txt

### running

python main.py

running this executes all 5 scenarios in sequence. For each scnenario, a window with 3 static graphs(speed comaprision, following distance, and throttle/brake output) in sequence. closing that window opens a top-down animated view of the same scenario; closing that moves on to the next scenario.
In total the full run opens 10 windows

### Scenarios

| Scenario | What it tests |
|---|---|
| Highway Cruise | Steady-state cruising with a lead vehicle at a similar speed |
| Traffic Slowdown | Gradual and moderate lead-vehicle speed changes |
| Emergency Stop | A sudden, sharp drop in lead-vehicle target speed |
| Cut-in | A vehicle merging into your lane at close range |
| Hill Climb | Following behaviour on a graded (uphill) road |