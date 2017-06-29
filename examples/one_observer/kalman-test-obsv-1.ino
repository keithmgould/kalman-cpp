#include <BasicLinearAlgebra.h>
#include <kalman.h>

#define MYDIMN 3
#define MYDIMM 1


void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {}

  double dt = 1.0/30; // Time step

  Matrix<MYDIMN,MYDIMN> A; // System dynamics matrix
  Matrix<MYDIMN,1> B; // Input matrix
  Matrix<MYDIMM,MYDIMN> C; // Output matrix
  Matrix<MYDIMN,MYDIMN> Q; // Process noise covariance
  Matrix<MYDIMM,MYDIMM> R; // Measurement noise covariance
  Matrix<MYDIMN,MYDIMN> P; // Estimate error covariance

  // Discrete LTI projectile motion
  // with states x, xdot, xddot
  A << 1, dt, 0,
       0, 1, dt,
       0, 0, 1;

  // input matrix: nada
  B << 0, 0, 0;

  // just observe position
  C << 1, 0, 0;

  // Reasonable covariance matrices
  Q << .05, .05, .0,
       .05, .05, .0,
       .0,  .0,  .0;

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
  KalmanFilter<MYDIMN, MYDIMM> kf;
  kf.BuildFilter(A, B, C, Q, R, P);

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
  Matrix<MYDIMN, 1> x0;
  x0 << measurements[0], 0, -9.81;
  kf.init(x0);

  double t = 0;
  Matrix<MYDIMM, 1> y;

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
