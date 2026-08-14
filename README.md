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