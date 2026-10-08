# State Observers and Feedback-Control Simulation

## Objective

State-feedback control requires knowledge of a system's internal states. When only the outputs are measured, an observer can reconstruct the states using the system model and the difference between measured and predicted outputs.

This implementation designs state observers and simulates a controller that uses estimated states for feedback.

## Method

### 1. Designing observer gains

Two single-output examples use Ackermann's formula to calculate observer gains for specified pole locations.

The eigenvalues of A - Ko*C are calculated to check the resulting observer dynamics.

### 2. Checking the three-state system

Before designing the controller and observer, the code constructs the controllability and observability matrices.

Both matrices have rank 3, confirming that the three-state system is controllable and observable.

### 3. Selecting controller and observer poles

The state-feedback controller places the plant poles at:
- -4, -5, and -6.

The observer poles are placed at:
- -6, -8, and -12.

The observer poles are chosen farther into the left half-plane to make the estimation-error dynamics decay faster than the selected plant dynamics.

### 4. Simulating estimated-state feedback

The control input uses the estimated state:

u = -Kc*xhat

The observer evolves according to:

dxhat/dt = A*xhat + B*u + Ko*(y - C*xhat)

The correction term compares the measured output with the output predicted by the observer.

The plant and observer are simulated together using ode45 over two seconds. The plant starts from [10; 12; 17], while the observer starts from zero, creating an initial estimation error.

## What the Plots Show

- **Actual and estimated states:** how the observer reconstructs the plant states.
- **Estimation errors:** how the difference between actual and estimated states evolves.
- **System outputs:** the measured signals available to the observer.
- **Control inputs:** the feedback action calculated from the estimated states.

The simulation is deterministic; process and measurement noise are not added.
