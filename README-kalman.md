# Kalman Filtering and Estimation-Error Analysis

## Objective

A system's internal states are not always directly measurable. Measurements can also contain noise, while disturbances affect the system itself.

This implementation explores how a continuous-time Kalman estimator combines a mathematical system model with noisy measurements to estimate those internal states.

## Method

### 1. Representing colored process noise

The first example uses a disturbance with frequency-dependent spectral density.

A first-order noise-shaping filter is included in the system model as an additional state. This produces an augmented model suitable for Kalman estimator design.

### 2. Comparing fixed observer gains

Two fixed observer gains are applied to the same two-state system.

For each gain, the code:
- Calculates the estimation-error dynamics.
- Checks the observer poles.
- Solves a continuous-time Lyapunov equation for the steady-state error covariance.
- Calculates J = trace(P), the sum of the estimation-error variances.

This comparison shows that observer stability alone does not determine estimation accuracy.

### 3. Computing an optimal Kalman gain

The code then computes the steady-state Kalman gain for the same system, using unit process and measurement noise intensities.

Its error metric is compared with those obtained using the fixed gains.

| Gain | J = trace(P) |
| --- | ---: |
| Fixed gain [2; 2] | 6.25 |
| Fixed gain [3; 1] | 7.00 |
| Kalman gain [2; 1] | 6.00 |

For this system and noise model, the Kalman estimator achieves the smallest error metric of the three.

### 4. Comparing minimum-phase and non-minimum-phase models

The final example compares two models with the same poles but different zero locations.

The non-minimum-phase model produces a larger estimation-error metric:
- Minimum-phase model: approximately 11.7867.
- Non-minimum-phase model: approximately 23.7867.

This illustrates how zero locations can affect estimation performance for the models studied.

## Outputs

The script displays Kalman gains, covariance matrices, estimator poles, and estimation-error metrics.

It performs steady-state calculations rather than simulating noisy trajectories.

