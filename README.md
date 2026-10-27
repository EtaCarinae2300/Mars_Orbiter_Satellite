# Mars Orbiter Spacecraft Attitude Control

## Overview

This repository contains a MATLAB simulation of spacecraft attitude dynamics and control for a Low Mars Orbiter (LMO).

The project models spacecraft rotational motion using Modified Rodrigues Parameters (MRPs) and implements a proportional-derivative (PD) feedback controller for attitude tracking.

Three reference-pointing modes are supported:
- **Sun-Pointing:** Maintain a fixed reference attitude relative to the inertial frame.
- **Nadir-Pointing:** Orient the spacecraft toward the center of Mars.
- **GMO-Pointing:** Track a Geosynchronous Mars Orbiter (GMO).

The mission simulation switches between these modes based on the spacecraft's orbital position and the relative geometry of the two orbiters.

## Dependencies

- MATLAB
- Aerospace Toolbox (for `dcm2quat`)

## How to Execute Code

Before running the simulations, execute:

```matlab
startup
```

This adds the required helper-function directories to the MATLAB path.

The repository contains two main simulation scripts.

### 1. Mission Scenario Simulation

Run:

```matlab
Mission_Scenario_Simulation
```

This simulates 6500 seconds of spacecraft attitude motion with automatic transitions between Sun-pointing, Nadir-pointing, and GMO-pointing.

The script calculates attitude tracking errors, control torques, angular momentum, and rotational kinetic energy.

### 2. Simulator Testing

Run:

```matlab
Simulator_Testing
```

This script evaluates the numerical simulator under different operating conditions:

- **Torque-Free Motion:** Simulates spacecraft rotational motion without external control torques.
- **Constant Control Torque:** Simulates spacecraft motion under a fixed applied torque.
- **Sun-Pointing Control:** Tests attitude convergence toward a fixed inertial reference.
- **Nadir-Pointing Control:** Tests tracking of the time-varying nadir reference frame.
- **GMO-Pointing Control:** Tests tracking of a reference frame oriented toward the GMO.

Simulation duration and time step can be adjusted within the scripts.

## Folder Structure

**Coordinate System Transformations:** Contains functions for orbital coordinate transformations, reference-frame calculations, MRP/DCM conversions, and attitude tracking error calculations.

**Params & Other Helper Functions:** Contains the spacecraft and orbital parameters, angular momentum and kinetic energy calculations, and plotting utilities.

## Major Files of Relevance

| File | Description |
|---|---|
| `Mission_Scenario_Simulation.m` | Simulates spacecraft attitude control with automatic mission-mode switching. |
| `Simulator_Testing.m` | Tests attitude dynamics and control under different operating conditions. |
| `RK4_Integration.m` | Fourth-order Runge-Kutta integrator for spacecraft rotational dynamics and MRP kinematics. |
| `ParamsDefinition.m` | Master definition of Mars, orbital, and spacecraft parameters. |
| `Calc_PointingFrame.m` | Calculates reference attitudes and angular velocities for the three pointing modes. |
| `Calc_AttitudeError.m` | Computes attitude and angular velocity tracking errors. |
| `Calc_HillFrame.m` | Calculates the orbital Hill frame. |
| `PlotSimulation.m` | Visualizes attitude, angular velocity, angular momentum, kinetic energy, and control torque histories. |

## Future Work

- **Alternative Numerical Integrators:** Implement MATLAB's `ode45` solver and compare its performance and accuracy against the existing RK4 integrator.
- **Improved Attitude Control:** Explore alternative control laws and compare their tracking performance with the existing PD controller.
- **Mission Visualization:** Visualize mission-mode transitions and spacecraft pointing orientations throughout the orbit.
