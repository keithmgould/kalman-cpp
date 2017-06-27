#include <BasicLinearAlgebra.h>
#include <kalman.h>

#define STATES 4
#define OBVS 3


void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {}

  double dt = .02; // 50 Hz

  Matrix<STATES,STATES> A; // System dynamics matrix
  Matrix<STATES,1> B; // Input matrix
  Matrix<OBVS,STATES> C; // Output matrix
  Matrix<STATES,STATES> Q; // Process noise covariance
  Matrix<OBVS,OBVS> R; // Measurement noise covariance
  Matrix<STATES,STATES> P; // Estimate error covariance

  // Discrete LTI projectile motion, measuring position only
  A << 1, dt, 0, 0,
       0, 1,  0, 0,
       0, 0,  1, dt,
       0, 0,  0, 1;

  // input matrix
  B << 0, 0, 0, 0;

  // just observe x, y, ydot
  C << 1, 0, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 1;

  // Reasonable covariance matrices
  Q << .05, .05, .0,  .0,
       .05, .05, .0,  .0,
       .0,  .0,  .05, .05,
       .0,  .0,  .05, .05;

  R(0,0) = 5;

  P << .1, .1, .1,
       .1, 10000, 10,
       .1, 10, 100;

  Serial << "A: " <<  A << '\n';
  Serial << "B: " <<  B << '\n';
  Serial << "C: " <<  C << '\n';
  Serial << "Q: " <<  Q << '\n';
  Serial << "R: " <<  R << '\n';
  Serial << "P: " <<  P << '\n';

  // Construct the filter
  KalmanFilter<STATES, OBVS> kf;
  kf.BuildFilter(A, B, C, Q, R, P);

/*
 TRUE MOTION:

 0.0,0.0,8.0,10.0
0.4,0.488,8.0,9.976
0.8,0.951,8.0,9.902
1.2,1.39,8.0,9.78
1.6,1.804,8.0,9.608
2.0,2.194,8.0,9.388
2.4,2.559,8.0,9.118
2.8,2.9,8.0,8.8
3.2,3.216,8.0,8.432
3.6,3.508,8.0,8.015
4.0,3.775,8.0,7.55
4.4,4.018,8.0,7.036
4.8,4.236,8.0,6.472
5.2,4.43,8.0,5.859
5.6,4.599,8.0,5.198
6.0,4.744,8.0,4.487
6.4,4.864,8.0,3.728
6.8,4.96,8.0,2.919
7.2,5.031,8.0,2.062
7.6,5.078,8.0,1.155
8.0,5.1,8.0,0.2
8.4,5.098,8.0,-0.805
8.8,5.071,8.0,-1.858
9.2,5.02,8.0,-2.961
9.6,4.944,8.0,-4.112
10.0,4.844,8.0,-5.313
10.4,4.719,8.0,-6.562
10.8,4.57,8.0,-7.861
11.2,4.396,8.0,-9.208
11.6,4.198,8.0,-10.605
12.0,3.975,8.0,-12.05
12.4,3.728,8.0,-13.545
12.8,3.456,8.0,-15.088
13.2,3.16,8.0,-16.681
13.6,2.839,8.0,-18.322
14.0,2.494,8.0,-20.013
14.4,2.124,8.0,-21.752
14.8,1.73,8.0,-23.541
15.2,1.311,8.0,-25.378
15.6,0.868,8.0,-27.265

*/

  // List of noisy position measurements (y)
  float measurements [] = {
    1.04202710058,
    1.10726790452,
    1.2913511148,
    1.48485250951,
    1.72825901034,
    1.74216489744,
    2.11672039768,
    2.14529225112,
    2.16029641405,
    2.21269371128,
    2.57709350237,
    2.6682215744,
    2.51641839428,
    2.76034056782,
    2.88131780617,
    2.88373786518,
    2.9448468727,
    2.82866600131,
    3.0006601946,
    3.12920591669,
    2.858361783,
    2.83808170354,
    2.68975330958,
    2.66533185589,
    2.81613499531,
    2.81003612051,
    2.88321849354,
    2.69789264832,
    2.4342229249,
    2.23464791825,
    2.30278776224,
    2.02069770395,
    1.94393985809,
    1.82498398739,
    1.52526230354,
    1.86967808173,
    1.18073207847,
    1.10729605087,
    0.916168349913,
    0.678547664519,
    0.562381751596,
    0.355468474885,
    -0.155607486619,
    -0.287198661013,
    -0.602973173813
  };

  // Best guess of initial states
  Matrix<STATES, 1> x0;
  x0 << measurements[0], 0, -9.81;
  kf.init(x0);

  double t = 0;
  Matrix<OBVS, 1> y;

  Serial << ", x_hat[0]: " << ~kf.state() << '\n';
  for(int i = 0; i < 45; i++) {
    y << measurements[i];
    kf.update(y, 0);
    Serial.print(", y[");
    Serial.print(i);
    Serial.print("] = ");
    Serial << ~y;
    Serial.print(", x_hat[");
    Serial.print(i);
    Serial.print("] = ");
    Serial << ~kf.state() << '\n';
  }
}

void loop() {}
