// Taken from: http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf

#ifndef MYKALMAN
#define MYKALMAN
#include <BasicLinearAlgebra.h>

#pragma once

template<int DIMN, int DIMM>
class KalmanFilter {

public:
  void BuildFilter(
    double get_dt,
    const Matrix<DIMN,DIMN>& get_A,
    const Matrix<DIMN,1>& get_B,
    const Matrix<DIMM,DIMN>& get_C,
    const Matrix<DIMN,DIMN>& get_Q,
    const Matrix<DIMM,DIMM>& get_R,
    const Matrix<DIMN,DIMN>& get_P0
  )
  {
    dt = get_dt;
    A = get_A;
    B = get_B;
    C = get_C;
    Q = get_Q;
    R = get_R;
    P0 = get_P0;

    m = C.GetRowCount();
    n = A.GetRowCount();
    initialized = false;
    I = Identity<DIMN,DIMN,float>();
    x_hat = n;
    x_hat_new = n;
  }

  void init(double t0, const Matrix<DIMN,1>& x0) {
    x_hat = x0;
    P = P0;
    this->t0 = t0;
    t = t0;
    initialized = true;
  }

  void update(const Matrix<DIMM,1>& y, const float& u) {
    // =================================
    // Prediction Phase

    // project the state ahead
    x_hat_new = A * x_hat + B * u;

    // project the error covariance ahead
    P = A*P*(~A) + Q;

    // =================================
    // Correction Phase

    // Compute the Kalman gain
    K = P*(~C)*(C*P*(~C) + R).Inverse();

    // update the estimate
    x_hat_new += K * (y - C*x_hat_new);

    // update the error covariance
    P = (I - K*C)*P;

    //==================================
    // prepare for next iteration

    // iterate state estimation
    x_hat = x_hat_new;

    // iterate time
    t += dt;
  }

  Matrix<DIMN, 1> state() { return x_hat; }
  double time() { return t; }

private:

  // Discrete time step
  double dt;

  // Matrices
  Matrix<DIMN, DIMN> A;   // system
  Matrix<DIMN, 1>    B;   // input
  Matrix<DIMM, DIMN> C;   // observation
  Matrix<DIMN, DIMN> Q;   // Process noise covariance
  Matrix<DIMM, DIMM> R;   // Measurement noise covariance
  Matrix<DIMN, DIMN> P;   // Estimate error covariance
  Matrix<DIMN, DIMN> P0;
  Matrix<DIMN, DIMM> K;

  // System dimensions
  int m, n;

  // Initial and current time
  double t0, t;

  // Is the filter initialized?
  bool initialized;

  // n-size identity
  Matrix<DIMN, DIMN> I;

  // Estimated states
  Matrix<DIMN, 1> x_hat, x_hat_new;
};

#endif
