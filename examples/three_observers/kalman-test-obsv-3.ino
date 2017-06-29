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
  B << 0,
       0,
       0.5 * dt * dt,
       dt;

  // observe x, y, ydot (not xdot)
  C << 1, 0, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 1;

  // Process Noise covariance matrices
  Q << .001, .0, .0,  .0,
       .0, .001, .0,  .0,
       .0,  .0,  .001, .0,
       .0,  .0,  .0, .001;

  // Measurement noise covariance
  R << .001, 0.0, 0.0,
       0.0, .001, .001,
       0.0, .001, .001;

  P.Fill(0);

  Serial << "A: " <<  A << '\n';
  Serial << "B: " <<  B << '\n';
  Serial << "C: " <<  C << '\n';
  Serial << "Q: " <<  Q << '\n';
  Serial << "R: " <<  R << '\n';
  Serial << "P: " <<  P << '\n';

  // Construct the filter
  KalmanFilter<STATES, OBVS> kf;
  kf.BuildFilter(A, B, C, Q, R, P);

  // List of noisy position measurements (x, y, y_dot)
  // to modify and regenerate, see the generate_data.rb script.
  float measurements[][3] = {
    {-0.324,-0.032,4.957},
    {0.165,0.102,4.89},
    {0.317,0.199,4.773},
    {0.463,0.278,4.537},
    {0.653,0.365,4.276},
    {0.833,0.467,4.044},
    {0.973,0.531,3.864},
    {1.1,0.607,3.643},
    {1.247,0.691,3.593},
    {1.422,0.766,3.215},
    {1.621,0.773,2.921},
    {1.798,0.903,2.823},
    {1.896,0.913,2.747},
    {2.009,0.932,2.549},
    {2.264,0.994,2.328},
    {2.324,1.022,2.052},
    {2.523,1.065,1.85},
    {2.722,1.128,1.714},
    {2.893,1.107,1.411},
    {3.192,1.212,1.296},
    {3.047,1.168,1.028},
    {3.512,1.283,0.925},
    {3.677,1.19,0.66},
    {3.769,1.223,0.494},
    {3.661,1.29,0.288},
    {3.858,1.238,0.102},
    {3.956,1.29,-0.096},
    {4.388,1.311,-0.299},
    {4.684,1.28,-0.477},
    {4.58,1.278,-0.714},
    {4.609,1.229,-0.839},
    {5.064,1.21,-1.05},
    {4.941,1.191,-1.244},
    {5.11,1.209,-1.49},
    {5.677,1.168,-1.662},
    {5.325,1.081,-1.84},
    {5.917,1.091,-2.03},
    {5.724,1.006,-2.283},
    {6.351,1.002,-2.43},
    {6.495,0.933,-2.595},
    {6.534,0.881,-2.826},
    {6.588,0.819,-3.097},
    {6.618,0.747,-3.13},
    {7.183,0.709,-3.406},
    {7.241,0.621,-3.732},
    {7.002,0.529,-3.789},
    {7.374,0.465,-3.895},
    {7.843,0.37,-4.356},
    {7.379,0.28,-4.303},
    {7.788,0.192,-4.8}
  };

  // Best guess of initial states
  Matrix<STATES, 1> x0;
  x0 << 0, 8, 0, 5;
  kf.init(x0);

  double t = 0;
  Matrix<OBVS, 1> y;
  Serial.println("\n\n");
  Serial.println("dt, obv(x), obv(y), obv(ydot), est(x), est(xdot), est(y), est(ydot)");
  for(int i = 0; i < 50; i++) {
    y << measurements[i][0], measurements[i][1], measurements[i][2];
    kf.update(y, -9.8);
    Serial.print(i * dt, 3);
    Serial.print(',');
    Serial.print(y(0,0), 3);
    Serial.print(',');
    Serial.print(y(1,0), 3);
    Serial.print(',');
    Serial.print(y(2,0), 3);
    Serial.print(',');
    Serial.print(kf.state()(0,0), 3);
    Serial.print(',');
    Serial.print(kf.state()(1,0), 3);
    Serial.print(',');
    Serial.print(kf.state()(2,0), 3);
    Serial.print(',');
    Serial.println(kf.state()(3,0), 3);
  }
}

void loop() {}
