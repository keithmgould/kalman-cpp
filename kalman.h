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
    const Matrix<DIMM,DIMN>& get_C,
    const Matrix<DIMN,DIMN>& get_Q,
    const Matrix<DIMM,DIMM>& get_R,
    const Matrix<DIMN,DIMN>& get_P
  )
  {
    dt = get_dt;
    A = get_A;
    C = get_C;
    Q = get_Q;
    R = get_R;
    P0 = get_P;

    m = C.GetRowCount();
    n = A.GetRowCount();
    initialized = false;
    I = A;
    I.SetIdentity();
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

  void update(const Matrix<DIMM,1>& y) {
    // =================================
    // Prediction Phase

    // project the state ahead
    Multiply(A,x_hat,x_hat_new);

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
  Matrix<DIMN, DIMN> A;
  Matrix<DIMM, DIMN> C;
  Matrix<DIMN, DIMN> Q;
  Matrix<DIMM, DIMM> R;
  Matrix<DIMN, DIMN> P;
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
