# Recursive Estimation Tracking Project

This folder contains my estimator implementation for a recursive-estimation
tracking project. The goal was to estimate a hidden 9-dimensional motion state
from noisy sensor measurements over time.

The setup is a nonlinear tracking problem. The target moves in 3D with a
heading, pitch angle, forward speed, angular rates, and acceleration. Only noisy
position-related measurements are available, so the estimator has to infer the
unobserved motion states recursively.

The state estimate contains position, orientation, speed, turn rates, and
acceleration:

```text
[x, y, z, psi, theta, p, r_psi, r_theta, a]
```

Two estimation problems are handled in `tracking_filters.py`:

- `GaussianTrackingEKF`: a textbook Extended Kalman Filter for the
  Gaussian-noise case. This path keeps the prediction, Jacobian, Kalman gain,
  and covariance update close to the standard EKF equations.
- `RobustNonGaussianTrackingFilter`: a more pragmatic recursive estimator for
  the non-Gaussian case. It keeps the same model structure but adds robust
  sonar handling, state normalization, and measurement weighting tuned for
  outliers.

## Approach

The estimator uses a model-based prediction/correction loop:

1. Predict the next state from the previous estimate and the nonlinear motion
   model.
2. Linearize that prediction with a Jacobian.
3. Propagate uncertainty through the linearized model.
4. Compare predicted measurements against actual sensor readings.
5. Apply a Kalman-style correction using the measurement residual.

The process model contains simple control-style feedback terms. For example,
the pitch-rate and acceleration states are not treated as arbitrary random
walks; they are driven toward target values through proportional/damping terms.
This gives the filter useful structure: even when measurements are noisy or
missing, the estimator still has a plausible motion prior.

The Gaussian path is intentionally conservative and textbook-like. It uses the
standard EKF equations directly: nonlinear prediction, transition Jacobian,
Kalman gain, and compact covariance update.

The non-Gaussian path is more engineering-driven. It keeps the same recursive
structure, but adds robustness where the measurement model is less friendly:

- missing-measurement handling for sensor dropouts,
- angle wrapping for orientation states,
- physical clamping for depth/speed-like states,
- stronger weighting of informative position measurements,
- outlier inflation when a sonar residual is implausibly large.

I also compared this Kalman-style robust estimator against a particle-filter
prototype. The particle filter was conceptually attractive for non-Gaussian
noise, but in this problem it was slower and less accurate because the full
state is high-dimensional and several states are only indirectly observed.

## What Is Included

```text
tracking_filters.py
visuals/
```

`tracking_filters.py` contains the public-facing implementation. The `visuals/`
folder contains generated trajectory/state plots from my own runs.

## What Is Not Included

The original project framework is not included here. In particular, this
folder does not contain the provided simulator, constants file,
Docker setup or assignment statement files from the course.

That means this folder is mainly for portfolio/review purposes rather than a
standalone runnable package.

## Visualizations

Typical interpretation:

- `plot_3d.png` shows the reconstructed 3D trajectory.
- `plot_ekf.png` shows the Gaussian EKF behavior.
- `plot_nge.png` shows the robust non-Gaussian estimator behavior.

## Notes

This repository folder is meant to demonstrate implementation work in recursive
estimation: nonlinear prediction, EKF linearization, covariance propagation,
missing measurements, and robust handling of non-Gaussian sensor noise.

The implementation is intentionally compact and numerical rather than
framework-heavy. Most of the work is in the modeling choices, Jacobian, noise
covariances, and robustness decisions.
