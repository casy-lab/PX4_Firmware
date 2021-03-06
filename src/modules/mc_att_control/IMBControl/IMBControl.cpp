#include <IMBControl.hpp>

IMBControl::IMBControl(){
    zRoll.setZero(); 
    zPitch.setZero();

    // Matrices Initialization
    // Roll
    Ar(0,0) = 1;        Ar(0,1) = 4e-3;  
    Ar(1,0) = -0.0017;  Ar(1,1) = 1;     

    Br(0,0) = 0;        Br(1,0) = 4e-3;

    Cr(0,0) = 1;        Cr(0,1) = 0.002;
    Cr(1,0) = -0.0008;  Cr(1,1) = 1;   

    Dr(0,0) = 0;        Dr(1,0) = 0.002;

    Kr(0,0) = 0.4359;   Kr(0,1) = 1.7830;   Kr(0,2) = 0.3617;
    Kr(0,3) = 0.0807;   Kr(0,4) = 0.4725;

    // Pitch
    Ap(0,0) = 1;        Ap(0,1) = 4e-3;
    Ap(1,0) = -0.0017;  Ap(1,1) = 1;   

    Bp(0,0) = 0;        Bp(1,0) = 0.004;

    Cp(0,0) = 1;        Cp(0,1) = 0.002;
    Cp(1,0) = -0.0008;  Cp(1,1) = 1;   

    Dp(0,0) = 0;        Dp(1,0) = 0.002;

    Kp(0,0) = -0.4400;  Kp(0,1) = 1.7997;   Kp(0,2) = 0.3651;
    Kp(0,3) = -0.0815;  Kp(0,4) = -0.4769;

    // Yaw
    Ky(0,0) = 0.3298;
    Ky(0,1) = 0.2755;

    // Vertical Dynamics
    Av = 1.0; Bv = 4e-3; Cv = 1.0; Dv = 2e-3;

    Kv(0,0) = -6.5968;
    Kv(0,1) = -5.5532;
    Kv(0,2) = -2.5853;
}

matrix::Matrix<double, 4, 1> IMBControl::update(matrix::Matrix<double, 12, 1> actualState,
                                                matrix::Matrix<double, 3, 1> setPoint, double dt){
    // States are Memorized As
    // 0:x / 1:y / 2:z / 3:xDot / 4:yDot / 5:zDot
    // 6:phi / 7:theta / 8:psi / 9:phiDot / 10:thetaDot / 11:psiDot
    matrix::Matrix<double, 4, 1> controlAction;
    
    // Tau X
    controlAction(0,0) = rollCtrl(actualState(2,0),     /*yDot*/
                                  actualState(4,0),     /*phi*/
                                  actualState(7,0),     /*phiDot*/
                                  setPoint(1,0));       /*yRef*/

    // Tau Y
    controlAction(1,0) = pitchCtrl(actualState(1,0),    /*xDot*/
                                   actualState(5,0),    /*theta*/
                                   actualState(8,0),   /*thetaDot*/
                                   setPoint(0,0));      /*xRef*/

    // Tau Z
    controlAction(2,0) = yawCtrl(actualState(6,0),      /*psi*/
                                 actualState(9,0));    /*psiDot*/

    // Thrust
    controlAction(3,0) = verticalCtrl(actualState(0,0), /*z*/
                                      actualState(3,0), /*zDot*/
                                      setPoint(2,0));   /*zRef*/

    return controlAction;
}

double IMBControl::rollCtrl(double yDot, double phi, double phiDot, double yRef){
    // Control Update
    matrix::Matrix<double, 2, 1> etaRoll;
    double e = yDot - yRef;

    etaRoll = Cr*zRoll + Dr*e;
    zRoll = Ar*zRoll + Br*e;

    matrix::Matrix<double, 5, 1> X;
    X(0,0) = yDot;          X(1,0) = phi;
    X(2,0) = phiDot;        X(3,0) = etaRoll(0,0);
    X(4,0) = etaRoll(1,0);

    return (-Kr*X)(0,0); // Tau X
}

double IMBControl::pitchCtrl(double xDot, double theta, double thetaDot, double xRef){
    // Control Update
    matrix::Matrix<double, 2, 1> etaPitch;
    double e = xDot - xRef;

    etaPitch = Cp*zPitch + Dp*e;
    zPitch = Ap*zPitch + Bp*e;

    matrix::Matrix<double, 5, 1> X;
    X(0,0) = xDot;          X(1,0) = theta;
    X(2,0) = thetaDot;      X(3,0) = etaPitch(0,0);
    X(4,0) = etaPitch(1,0);

    return (-Kp*X)(0,0); // Tau Y
}

double IMBControl::yawCtrl(double psi, double psiDot){
    // Control Update
    matrix::Matrix<double, 2, 1> X;
    X(0,0) = psi;   X(1,0) = psiDot;

    return (-Ky*X)(0,0); // Tau Z
}

double IMBControl::verticalCtrl(double z, double zDot, double zRef){
    // Control Update
    double etaV;
    double e = z - zRef;

    etaV = Cv*zV + Dv*e;
    zV = Av*zV + Bv*e;

    matrix::Matrix<double, 3, 1> X;
    X(0,0) = z; X(1,0) = zDot;  X(2,0) = etaV;

    return (-Kv*X)(0,0); // Thrust
}

void IMBControl::reset(){
    zRoll.setZero();
    zPitch.setZero();
    zV = 0.0;
}