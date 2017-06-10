Arduino Kalman Filter
=====================
This is a basic nxn Kalman filter using an [Arduino Linear Algebra library](https://github.com/tomstewart89/BasicLinearAlgebra).

It implements the algorithm directly as found in [An Introduction to the Kalman Filter](http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf).

This is a fork of [this](https://github.com/hmartiro/kalman-cpp) Kalman filter library, which used the Eigen C++ library, which in turn required a STL port to AVR which felt like overkill, took up space, and was a bit buggy.
