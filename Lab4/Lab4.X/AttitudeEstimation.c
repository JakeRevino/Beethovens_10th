#include "BOARD.h"
#include "BNO055.h"
#include "Ascii.h"
#include "timers.h"
#include "serial.h"
#include "Oled.h"
#include "MatrixMath.h"

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <math.h>
#include <string.h>

/* 
 * function: int EulerAngles(float mat[3][3], float theta, float psi, float phi, float angles[3])
 * - INPUT: mat[3][3] is an arbitrary 3x3 matrix containing the angles to extract 
 * - OUTPUT: angles[3] is the 3x1 matri returned holding the Euler Angles (?,?,?) 
 *  */
int EulerAngles(float mat[3][3], float angles[3]);

/* 
 * function: float Norm(float mat[3])
 * This function takes the 3D norm of a vector
 * - INPUT: mat[3] the vector to be normalized
 * - OUTPUT: normalized vector
 * */
float Norm(float mat[3]);

/*
 * function: int rcross(float r[3], float rx[3][3])
 * forms the skew symmetric x-product matrix of a 3x1 vector
 * - INPUT: r[3] matrix to be skewed
 * - OUTPUT: rx[3][3] skewed-symmetric matrix
 * */
int rcross(float r[3], float rx[3][3]);

/* 
 * function: int Rexp(float w[3], float deltaT, float R_exp[3][3])
 * creates an Exponential Attitude Propagation Matrix
 * - INPUT: w[3] Rotation-rate matrix 
 * - INPUT: deltaT time step
 * - OUTPUT: R_exp[3][3] Exponential Attitude Propagation Matrix 
 * */
int Rexp(float w[3], float deltaT, float R_exp[3][3]);

/* 
 * function: int IntegrateOpenLoop(float Rminus[3][3], float gyros[3], float deltaT, float Rplus[3][3])
 * - INPUT: Rminus[3][3]
 * - INPUT: gyros[3] these will be the X,Y,Z readings from the gyroscope
 * - INPUT: deltaT time step
 * - OUTPUT: Rplus[3][3]
 * */
int IntegrateOpenLoop(float Rminus[3][3], float gyros[3], float deltaT, float Rplus[3][3]);

// Module Defines
#define rad2deg 180 / 3.141592654
#define deg2rad 3.141592654 / 180
#define _20ms 20
#define angleScaler 46 // this value came from trial and error 
#define GyroXOffset 7.75 // values from Matlab after taking the mean of the 10s data
#define GyroYOffset 17.913
#define GyroZOffset 29.072
#define Raw2AngleConv 131.068
#define DELTA_T 0.02

int EulerAngles(float mat[3][3], float angles[3]) {
    float roll, pitch, yaw; //  (X,Y,Z) 

    /*
    // convert to radians
    theta = theta * (pi / 180);
    psi = psi * (pi / 180);
    phi = phi * (pi / 180);

    // Populate the DCM matrix
    mat[0][0] = cos(theta) * cos(psi); // R(0,0) of DCM
    mat[0][1] = (sin(phi) * sin(theta) * cos(psi)) - (cos(phi) * sin(psi)); // R(0,1) of DCM
    mat[0][2] = (cos(phi) * sin(theta) * cos(psi)) + (sin(phi) * sin(psi)); // R(0,2) of DCM
    mat[1][0] = cos(theta) * sin(psi); // R(1,0) of DCM
    mat[1][1] = (sin(phi) * sin(theta) * sin(psi)) + (cos(phi) * cos(psi)); // R(1,1) of DCM
    mat[1][2] = (cos(phi) * sin(theta) * sin(psi)) - (sin(phi) * cos(psi)); // R(1,2) of DCM
    mat[2][0] = -sin(theta); // R(2,0) of DCM
    mat[2][1] = sin(phi) * cos(theta); // R(2,1) of DCM
    mat[2][2] = cos(phi) * cos(theta); // R(2,2) of DCM
     * */

    // extract the angles and convert to degrees
    angles[1] = asinf(-mat[0][2]) * rad2deg; // theta = -arcsin(R[0][2]) ..... Y
    angles[0] = atan2f(mat[1][2], mat[2][2]) * rad2deg; // phi = arctan2(R[1][2], R[2][2]) .... X
    angles[2] = atan2f(mat[0][1], mat[0][0]) * rad2deg; // psi = arctan2(R[0][1], R[0][0]) .... Z

    roll = angles[0];
    pitch = angles[1];
    yaw = angles[2];

    return SUCCESS;
}

float Norm(float mat[3]) {
    return sqrt(pow(mat[0], 2) + pow(mat[1], 2) + pow(mat[2], 2));
}

int rcross(float r[3], float rx[3][3]) {
    rx[0][0] = 0;
    rx[0][1] = -r[3];
    rx[0][2] = r[2];
    rx[1][0] = r[3];
    rx[1][1] = 0;
    rx[1][2] = -r[1];
    rx[2][0] = -r[2];
    rx[2][1] = r[1];
    rx[2][2] = 0;

    return SUCCESS;
}

int Rexp(float w[3], float deltaT, float R_exp[3][3]) {
    float wnorm;
    float wx[3][3];
    float sincW;
    float oneMinusCosW;
    float result1[3][3] = {
        {},
        {},
        {}
    };
    float result2[3][3] = {
        {},
        {},
        {}
    };
    float result3[3][3] = {
        {},
        {},
        {}
    };
    float eye3[3][3] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };
    wnorm = Norm(w); // take the norm of w (wnorm))
    rcross(w, wx); // call the rcross (wx) function
    if (wnorm < 0.2) {
        sincW = deltaT - (pow(deltaT, 3) * pow(wnorm, 2)) / 6.0 + (pow(deltaT, 5) * pow(wnorm, 4)) / 120.0;
        oneMinusCosW = (pow(deltaT, 2)) / 2.0 - (pow(deltaT, 4) * pow(wnorm, 2)) / 24.0 + (pow(deltaT, 6) * pow(wnorm, 4)) / 720.0;
    } else {
        sincW = sin(wnorm * deltaT) / wnorm;
        oneMinusCosW = (1.0 - cos(wnorm * deltaT)) / pow(wnorm, 2);
    }

    // Break up this eq. "R_exp = [1 0 0;0 1 0;0 0 1] - sincW * wx + oneMinusCosW * wx * wx" into steps below
    MatrixScalarMultiply(sincW, wx, result1); // result1 = sincW * wx
    MatrixMultiply(wx, wx, result2); // result2 = wx * wx
    MatrixScalarMultiply(oneMinusCosW, result2, result2); // result2 = oneMinusCosW * wx * wx
    MatrixSubtract(eye3, result1, result1); // result1 = [1 0 0;0 1 0;0 0 1] - (sincW * wx)
    MatrixAdd(result1, result2, R_exp); // R_exp = [1 0 0;0 1 0;0 0 1] - (sincW * wx) + oneMinusCosW * wx * wx

    return SUCCESS;

}

int IntegrateOpenLoop(float Rminus[3][3], float gyros[3], float deltaT, float Rplus[3][3]) {
    float GyroCross[3][3];
    float R_exp[3][3];
    float result1[3][3];
    int UseMatrixExponential = 1; // set to 0 for forward integration
    if (UseMatrixExponential == 1) {
        Rexp(gyros, deltaT, R_exp); // R_exp = gyros x deltaT
        MatrixMultiply(R_exp, Rminus, Rplus); // Rplus = R_exp * Rminus
    } else {
        rcross(gyros, GyroCross);
        MatrixMultiply(GyroCross, Rminus, result1); // result1 = GyroCross * Rminus
        MatrixScalarMultiply(deltaT, result1, result1); // result1 = deltaT * result1
        MatrixSubtract(Rminus, result1, Rplus); // Rplus = Rminus - result1
    }
    return SUCCESS;
}

int main(void) {
    BOARD_Init();
    BNO055_Init();
    OledInit();
    OledClear(OLED_COLOR_BLACK);
    OledUpdate();
    TIMERS_Init();

    int time = 0;
    char OledOutput[100];

    float RotationMatrix[3][3];
    float eulerAngles[3];
    float MatrixExponential;
    float gyroXYZ[3];
    float Rplus[3][3] = {};
    float Rminus[3][3] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };

    while (1) {
        gyroXYZ[0] = ((BNO055_ReadGyroX() + GyroXOffset) / Raw2AngleConv) * deg2rad; // Read and convert X
        gyroXYZ[1] = ((BNO055_ReadGyroY() + GyroYOffset) / Raw2AngleConv) * deg2rad; // Read and convert Y
        gyroXYZ[2] = ((BNO055_ReadGyroZ() - GyroZOffset) / Raw2AngleConv) * deg2rad; // Read and convert Z

        IntegrateOpenLoop(Rminus, gyroXYZ, DELTA_T, Rplus);
        EulerAngles(Rplus, eulerAngles);
        memcpy(Rminus, Rplus, 3 * 3 * sizeof (float));

        sprintf(OledOutput, "Roll: %f\nPitch: %f\nYaw: %f\r\n", eulerAngles[1], eulerAngles[0], eulerAngles[2]); // print roll, pitch, yaw
        OledDrawString(OledOutput);
        OledUpdate();

        time = TIMERS_GetMilliSeconds(); // make a start time
        while ((TIMERS_GetMilliSeconds() - time) < _20ms); // delay
    }
    return 0;
}