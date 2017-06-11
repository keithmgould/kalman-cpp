// Taken from: http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
#ifndef MYKALMAN
#define MYKALMAN
#include <BasicLinearAlgebra.h>

#pragma once

template<int DIMN, int DIMM>
class KalmanFilter {

public:

  KalmanFilter(
    double dt,
    const Matrix<DIMN,DIMN>& A,
    const Matrix<DIMM,DIMN>& C,
    const Matrix<DIMN,DIMN>& Q,
    const Matrix<DIMM,DIMM>& R,
    const Matrix<DIMN,DIMN>& P
  ) : dt(dt),
      A(A),
      C(C),
      Q(Q),
      R(R),
      P0(P),
      m(C.GetRowCount()),
      n(A.GetRowCount()),
      initialized(false),
      I(A),
      x_hat(n),
      x_hat_new(n)
  {
    I.SetIdentity();
  }

  void init(double t0, const Matrix<DIMN,1>& x0) {
    x_hat = x0;
    P = P0;
    this->t0 = t0;
    t = t0;
    initialized = true;
  }

  void update(const Matrix<DIMM,1>& y) {
    Multiply(A,x_hat,x_hat_new);
    P = A*P*(~A) + Q;
    K = P*(~C)*(C*P*(~C) + R).Inverse();
    x_hat_new += K * (y - C*x_hat_new);
    P = (I - K*C)*P;
    x_hat = x_hat_new;

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
  Matrix<DIMN, 1> K;

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
