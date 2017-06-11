Arduino Compatible Kalman Filter
================================
This is an NxN basic Kalman filter using the [Arduino Basic Linear Algebra library](https://github.com/keithmgould/BasicLinearAlgebra).

It implements the algorithm directly as found in [An Introduction to the Kalman Filter](http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf).

This is a port/rewrite of [this](https://github.com/hmartiro/kalman-cpp) Kalman filter library, which uses the Eigen C++ library (which in turn required a STL port to AVR which felt like overkill, took up space, and was a bit buggy.)

There is a test script you can run to see functionality: `kalman-test.ino`
