# kalman_filter

## Overview

This package contains the kalman filter with time delay and the calculation of the kalman filter.

## Design

The Kalman filter is a recursive algorithm used to estimate the state of a dynamic system. The Time Delay Kalman filter is based on the standard Kalman filter and takes into account possible time delays in the measured values.

### Standard Kalman Filter

#### System Model

Assume that the system can be represented by the following linear discrete model:

$$
x_{k} = A x_{k-1} + B u_{k} \\
y_{k} = C x_{k-1}
$$

where,

- $x_k$ is the state vector at time $k$.
- $u_k$ is the control input vector at time $k$.
- $y_k$ is the measurement vector at time $k$.
- $A$ is the state transition matrix.
- $B$ is the control input matrix.
- $C$ is the measurement matrix.

#### Prediction Step

The prediction step consists of updating the state and covariance matrices:

$$
x_{k|k-1} = A x_{k-1|k-1} + B u_{k} \\
P_{k|k-1} = A P_{k-1|k-1} A^{T} + Q
$$

where,

- $x_{k|k-1}$ is the priori state estimate.
- $P_{k|k-1}$ is the priori covariance matrix.

#### Update Step

When the measurement value \( y_k \) is received, the update steps are as follows:

$$
K_k = P_{k|k-1} C^{T} (C P_{k|k-1} C^{T} + R)^{-1} \\
x_{k|k} = x_{k|k-1} + K_k (y_{k} - C x_{k|k-1}) \\
P_{k|k} = (I - K_k C) P_{k|k-1}
$$

where,

- $K_k$ is the Kalman gain.
- $x_{k|k}$ is the posterior state estimate.
- $P_{k|k}$ is the posterior covariance matrix.

### Extension to Time Delay Kalman Filter

For the Time Delay Kalman filter, it is assumed that there may be a maximum delay of step ($d$) in the measured value. To handle this delay, we extend the state vector to include past states.

**Augmented State Vector:**

$$
(x_{k})_e = \begin{bmatrix}
x_k \\
x_{k-1} \\
\vdots \\
x_{k-d}
\end{bmatrix}
$$

**Augmented Transition Matrix ($A_e$) and Process Noise ($Q_e$):**

$$
A_e = \begin{bmatrix}
A & 0 & 0 & \cdots & 0 \\
I & 0 & 0 & \cdots & 0 \\
0 & I & 0 & \cdots & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
0 & 0 & 0 & \cdots & 0
\end{bmatrix}, \quad
Q_e = \begin{bmatrix}
Q & 0 & 0 & \cdots & 0 \\
0 & 0 & 0 & \cdots & 0 \\
0 & 0 & 0 & \cdots & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
0 & 0 & 0 & \cdots & 0
\end{bmatrix}
$$

#### Prediction Step

The prediction step shifts the history of states and predicts the new current state.

Update extended state:

$$
(x_{k|k-1})_e = A_e (x_{k-1|k-1})_e + \begin{bmatrix} B u_k \\ 0 \\ \vdots \end{bmatrix}
$$

Update extended covariance matrix:

$$
(P_{k|k-1})_e = A_e (P_{k-1|k-1})_e A_e^T + Q_e
$$

This operation essentially computes the new top-left block as $A P_{0,0} A^T + Q$ and shifts the cross-covariance blocks $P_{i,j}$ accordingly.

#### Update Step (Efficient Implementation)

When receiving a measurement value $y_k$ corresponding to a delayed state $x_{k-ds}$ (where $ds$ is the delay step), we exploit the sparsity of the observation matrix.

The effective observation matrix $H$ for the augmented state is sparse:
$$H = \begin{bmatrix} 0 & \cdots & C & \cdots & 0 \end{bmatrix}$$
where $C$ is located at the block index corresponding to the delay $ds$.

Instead of performing full matrix multiplications with zeros, we calculate the update using specific blocks:

**1. Innovation Covariance ($S$):**

Using the diagonal block of the covariance matrix corresponding to the delayed state ($P_{ds, ds}$):

$$
S = C P_{ds, ds} C^T + R
$$

**2. Kalman Gain ($K_k$):**

We calculate the gain using the column block of the covariance matrix ($P_{:, ds}$) which represents the correlation between **all states** (current and past) and the **delayed state**.

$$
P_{CT} = P_{:, ds} C^T
$$

$$
K_k = P_{CT} S^{-1}
$$

*Note: In implementation, $S^{-1}$ is solved via Cholesky decomposition.*

**3. Update State:**

The innovation is calculated against the delayed state estimate $x_{ds}$.

$$
e = y_k - C x_{ds}
$$

$$
(x_{k|k})_e = (x_{k|k-1})_e + K_k e
$$

**4. Update Covariance:**

We update the full covariance matrix using the computed terms. This is computationally equivalent to $P - K S K^T$ but more efficient.

$$
(P_{k|k})_e = (P_{k|k-1})_e - P_{CT} K_k^T
$$

where,

- $P_{ds, ds}$ is the covariance block of the delayed state.
- $P_{:, ds}$ is the rectangular block of covariance (entire column corresponding to delay).
- $P_{CT}$ corresponds to $P H^T$.

## Example Usage

This section describes Example Usage of KalmanFilter.

- Initialization

```cpp
#include "autoware/kalman_filter/kalman_filter.hpp"

// Define system parameters
int dim_x = 2; // state vector dimensions
int dim_y = 1; // measure vector dimensions

// Initial state
Eigen::MatrixXd x0 = Eigen::MatrixXd::Zero(dim_x, 1);
x0 << 0.0, 0.0;

// Initial covariance matrix
Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(dim_x, dim_x);
P0 *= 100.0;

// Define state transition matrix
Eigen::MatrixXd A = Eigen::MatrixXd::Identity(dim_x, dim_x);
A(0, 1) = 1.0;

// Define measurement matrix
Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, dim_x);
C(0, 0) = 1.0;

// Define process noise covariance matrix
Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(dim_x, dim_x);
Q *= 0.01;

// Define measurement noise covariance matrix
Eigen::MatrixXd R = Eigen::MatrixXd::Identity(dim_y, dim_y);
R *= 1.0;

// Initialize Kalman filter
autoware::kalman_filter::KalmanFilter kf;
kf.init(x0, P0);
