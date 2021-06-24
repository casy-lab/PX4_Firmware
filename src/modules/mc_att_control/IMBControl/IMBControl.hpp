#pragma once

#include <matrix/matrix/math.hpp>
#include <px4_module.h>
#include <px4_module_params.h>

class IMBControl{
    public:
    // Class Constructor
    IMBControl();

    // Class Destuctor
    ~IMBControl() = default;

    // Set Parameters
    void setParams();

    // Update Control Law
    matrix::Matrix<double, 4, 1> update(matrix::Matrix<double, 12, 1> actualState,
                                        matrix::Matrix<double, 3, 1> setPoint, double dt);

    // Reset
    void reset();

    private:
    /* Attributes */
    // Roll Matrices
    matrix::Matrix<double, 3, 3> Ar;
    matrix::Matrix<double, 3, 1> Br;
    matrix::Matrix<double, 3, 3> Cr;
    matrix::Matrix<double, 3, 1> Dr;
    matrix::Matrix<double, 1, 7> Kr;
    matrix::Matrix<double, 3, 1> zRoll;

    // Pitch Matrices
    matrix::Matrix<double, 3, 3> Ap;
    matrix::Matrix<double, 3, 1> Bp;
    matrix::Matrix<double, 3, 3> Cp;
    matrix::Matrix<double, 3, 1> Dp;
    matrix::Matrix<double, 1, 7> Kp;
    matrix::Matrix<double, 3, 1> zPitch;

    // Yaw Matrix
    matrix::Matrix<double, 1, 2> Ky;

    // Vertical Matrices
    matrix::Matrix<double, 3, 3> Av;
    matrix::Matrix<double, 3, 1> Bv;
    matrix::Matrix<double, 3, 3> Cv;
    matrix::Matrix<double, 3, 1> Dv;
    matrix::Matrix<double, 1, 5> Kv;
    matrix::Matrix<double, 3, 1> zV;

    // Roll Controller
    double rollCtrl(double y, double yDot, double phi, double phiDot, double yRef);

    // Pitch Controller
    double pitchCtrl(double x, double xDot, double theta, double thetaDot, double xRef);

    // Yaw Controller
    double yawCtrl(double psi, double psiDot);

    // Vertical Dynamics Controller
    double verticalCtrl(double z, double zDot, double zRef);
};