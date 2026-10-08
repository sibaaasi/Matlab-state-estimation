# State Estimation and Kalman Filtering in MATLAB

Two academic projects completed as part of my Electrical Engineering studies at Tel Aviv University, exploring how the internal states of linear systems can be estimated from measured outputs.

## 1. Kalman Filtering

Designed continuous-time Kalman estimators for systems affected by process and measurement noise.

The project includes:
- Augmenting a state-space model to represent colored process noise.
- Computing Kalman gains and estimation-error covariance matrices.
- Comparing fixed observer gains using the Lyapunov equation.
- Evaluating estimation performance using the metric J = trace(P).
- Comparing minimum-phase and non-minimum-phase systems.

The MATLAB script prints estimator gains, covariance matrices, poles, and estimation-error metrics. It performs steady-state calculations rather than a simulation of noisy measurements.

Code: [kalman_filter_analysis.m](kalman-filtering/kalman_filter_analysis.m)

## 2. State Observers and Feedback Control

Designed state observers and simulated a controller that uses estimated states for feedback.

The project includes:
- Observer design using Ackermann's formula.
- Controllability and observability checks.
- Controller and observer pole placement.
- Simulation of the plant and observer using ode45.
- Visualization of actual and estimated states, estimation errors, outputs, and control inputs.

For the three-state system, controller poles are placed at -4, -5, and -6, while observer poles are placed at -6, -8, and -12.

Code: [observer_design_simulation.m](state-observers/observer_design_simulation.m)

## Tools and Methods

MATLAB, Control System Toolbox, state-space modeling, Kalman filtering, Lyapunov equations, Ackermann's formula, pole placement, and numerical simulation.

## How to Run

Requirements:
- MATLAB R2016b or newer.
- Control System Toolbox.
Download or clone this repository, set MATLAB's Current Folder to the repository root, and run either script:
```matlab
run('kalman-filtering/kalman_filter_analysis.m')
```
```matlab
run('state-observers/observer_design_simulation.m')
```
Each script clears the workspace and closes existing figures before running.

## Selected Kalman Results

The estimation-error metric is J = trace(P), representing the sum of steady-state estimation-error variances.

| Estimator / model | J |
| --- | ---: |
| Augmented system with colored process noise | 0.502803 |
| Fixed observer gain [2; 2] | 6.250000 |
| Fixed observer gain [3; 1] | 7.000000 |
| Optimal two-state Kalman estimator | 6.000000 |
| Minimum-phase model | 11.786731 |
| Non-minimum-phase model | 23.786731 |

For the systems studied here, the optimal Kalman estimator improves on both selected fixed gains. The non-minimum-phase model has a larger estimation-error metric than the minimum-phase model.

## Validation

The numerical covariance values, stability conditions, controllability and observability ranks, and pole-placement targets were independently checked using Python/SciPy. The MATLAB scripts have not yet been rerun after repository preparation.
