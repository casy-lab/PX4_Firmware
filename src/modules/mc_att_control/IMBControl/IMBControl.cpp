#include <IMBControl.hpp>

IMBControl::IMBControl(){
    zRoll.setZero(); 
    zPitch.setZero();

    // Matrices Initialization
    // Roll
    Ar(0,0) = 1;    Ar(0,1) = 4e-3;     Ar(0,2) = 0;  
    Ar(1,0) = 0;    Ar(1,1) = 1;        Ar(1,2) = 4e-3; 
    Ar(2,0) = 0;    Ar(2,1) = -1.7e-3;  Ar(2,2) = 1;

    Br(0,0) = 0;    Br(0,0) = 0;    Br(2,0) = 4e-3;

    Cr(0,0) = 1;    Cr(0,1) = 0.002;    Cr(0,2) = 0;
    Cr(1,0) = 0;    Cr(1,1) = 1;        Cr(1,2) = 0.002;
    Cr(2,0) = 0;    Cr(2,1) = -8e-4;    Cr(2,2) = 1;  

    Dr(0,0) = 0;    Dr(0,0) = 0;    Dr(1,0) = 0.002;
/*
    Kr(0,0) = 0.4909;   Kr(0,1) = 0.3286;   Kr(0,2) = 1.3420;
    Kr(0,3) = 0.3190;   Kr(0,4) = 0.0978;   Kr(0,5) = 0.1406;
    Kr(0,6) = 0.4291;
*/
    Kr(0,0) = 0.3845;   Kr(0,1) = 0.2632;   Kr(0,2) = 1.1030;
    Kr(0,3) = 0.2757;   Kr(0,4) = 0.0694;   Kr(0,5) = 0.0933;
    Kr(0,6) = 0.3222;

    // Pitch
    Ap(0,0) = 1;    Ap(0,1) = 4e-3;     Ap(0,2) = 0;
    Ap(1,0) = 0;    Ap(1,1) = 1;        Ap(1,2) = 4e-3;
    Ap(2,0) = 0;    Ap(2,1) = -1.7e-3;  Ap(2,2) = 1;

    Bp(0,0) = 0;    Bp(1,0) = 0;    Bp(2,0) = 0.004;

    Cp(0,0) = 1;    Cp(0,1) = 0.002;    Cp(0,2) = 0;
    Cp(1,0) = 0;    Cp(1,1) = 1;        Cp(1,2) = 0.002; 
    Cp(2,0) = 0;    Cp(2,1) = -8e-4;    Cp(2,2) = 1;   

    Dp(0,0) = 0;    Dp(1,0) = 0;    Dp(2,0) = 0.002;
/*
    Kp(0,0) = -0.4915;  Kp(0,1) = -0.3293;   Kp(0,2) = 1.3461;
    Kp(0,3) = 0.3205;   Kp(0,4) = -0.0978;  Kp(0,5) = -0.1405;
    Kp(0,6) = -0.4293;
*/
    Kp(0,0) = -0.3851;  Kp(0,1) = -0.2638;   Kp(0,2) = 1.1071;
    Kp(0,3) = 0.2772;   Kp(0,4) = -0.0694;  Kp(0,5) = -0.0932;
    Kp(0,6) = -0.3224;

    // Yaw
    Ky(0,0) = 0.5830;
    Ky(0,1) = 0.3628;

    // Vertical Dynamics
    Av(0,0) = 1;    Av(0,1) = 4e-3;     Av(0,2) = 0;
    Av(1,0) = 0;    Av(1,1) = 1;        Av(1,2) = 4e-3;
    Av(2,0) = 0;    Av(2,1) = -1.7e-3;  Av(2,2) = 1;

    Bv(0,0) = 0;    Bv(1,0) = 0;    Bv(2,0) = 0.004;

    Cv(0,0) = 1;    Cv(0,1) = 0.002;    Cv(0,2) = 0;
    Cv(1,0) = 0;    Cv(1,1) = 1;        Cv(1,2) = 0.002; 
    Cv(2,0) = 0;    Cv(2,1) = -8e-4;    Cv(2,2) = 1;  

    Dv(0,0) = 0;    Dv(1,0) = 0;    Dv(2,0) = 0.002;

    Kv(0,0) = -4.1371;
    Kv(0,1) = -3.5797;
    Kv(0,2) = -0.4451;
    Kv(0,3) = -0.1875;
    Kv(0,4) = -2.4647;
}

matrix::Matrix<double, 4, 1> IMBControl::update(matrix::Matrix<double, 12, 1> actualState,
                                                matrix::Matrix<double, 3, 1> setPoint, double dt){
    // States are Memorized As
    // 0:x / 1:y / 2:z / 3:xDot / 4:yDot / 5:zDot
    // 6:phi / 7:theta / 8:psi / 9:phiDot / 10:thetaDot / 11:psiDot
    matrix::Matrix<double, 4, 1> controlAction;
    
    // Tau X
    controlAction(0,0) = rollCtrl(actualState(1,0),     /*y*/
                                  actualState(4,0),     /*yDot*/
                                  actualState(6,0),     /*phi*/
                                  actualState(9,0),     /*phiDot*/
                                  setPoint(1,0));       /*yRef*/

    // Tau Y
    controlAction(1,0) = pitchCtrl(actualState(0,0),    /*x*/
                                   actualState(3,0),    /*xDot*/
                                   actualState(7,0),    /*theta*/
                                   actualState(10,0),   /*thetaDot*/
                                   setPoint(0,0));      /*xRef*/

    // Tau Z
    controlAction(2,0) = yawCtrl(actualState(8,0),      /*psi*/
                                 actualState(11,0));    /*psiDot*/

    // Thrust
    controlAction(3,0) = verticalCtrl(actualState(2,0), /*z*/
                                      actualState(5,0), /*zDot*/
                                      setPoint(2,0));   /*zRef*/

    return controlAction;
}

double IMBControl::rollCtrl(double y, double yDot, double phi, double phiDot, double yRef){
    // Control Update
    matrix::Matrix<double, 3, 1> etaRoll;
    double e = y - yRef;

    etaRoll = Cr*zRoll + Dr*e;
    zRoll = Ar*zRoll + Br*e;

    matrix::Matrix<double, 7, 1> X;
    X(0,0) = y;
    X(1,0) = yDot;
    X(2,0) = phi;
    X(3,0) = phiDot;
    X(4,0) = etaRoll(0,0);
    X(5,0) = etaRoll(1,0);
    X(6,0) = etaRoll(2,0);

    return (-Kr*X)(0,0); // Tau X
}

double IMBControl::pitchCtrl(double x, double xDot, double theta, double thetaDot, double xRef){
    // Control Update
    matrix::Matrix<double, 3, 1> etaPitch;
    double e = x - xRef;

    etaPitch = Cp*zPitch + Dp*e;
    zPitch = Ap*zPitch + Bp*e;

    matrix::Matrix<double, 7, 1> X;
    X(0,0) = x;
    X(1,0) = xDot;
    X(2,0) = theta;
    X(3,0) = thetaDot;
    X(4,0) = etaPitch(0,0);
    X(5,0) = etaPitch(1,0);
    X(6,0) = etaPitch(2,0);

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
    matrix::Matrix<double, 3, 1> etaV;
    double e = z - zRef;

    etaV = Cv*zV + Dv*e;
    zV = Av*zV + Bv*e;

    matrix::Matrix<double, 5, 1> X;
    X(0,0) = z;
    X(1,0) = zDot; 
    X(2,0) = etaV(0,0);
    X(3,0) = etaV(1,0);
    X(4,0) = etaV(2,0);

    return (-Kv*X)(0,0); // Thrust
}

void IMBControl::reset(){
    zRoll.setZero();
    zPitch.setZero();
    zV.setZero();
}