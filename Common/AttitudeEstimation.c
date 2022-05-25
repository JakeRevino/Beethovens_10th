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
#define NUMSTEPS 200
#define PART6

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

float NormCol(float mat[3][1]) {
    return sqrt(pow(mat[0][0], 2) + pow(mat[1][0], 2) + pow(mat[2][0], 2));
}

int rcross(float r[3], float rx[3][3]) {
    rx[0][0] = 0;
    rx[0][1] = -r[2];
    rx[0][2] = r[1];
    rx[1][0] = r[2];
    rx[1][1] = 0;
    rx[1][2] = -r[0];
    rx[2][0] = -r[1];
    rx[2][1] = r[0];
    rx[2][2] = 0;

    return SUCCESS;
}

int rcross31(float r[3][1], float rx[3][3]) {
    rx[0][0] = 0;
    rx[0][1] = -r[2][0];
    rx[0][2] = r[1][0];
    rx[1][0] = r[2][0];
    rx[1][1] = 0;
    rx[1][2] = -r[0][0];
    rx[2][0] = -r[1][0];
    rx[2][1] = r[0][0];
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

int RexpCol(float w[3][1], float deltaT, float R_exp[3][3]) {
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
    wnorm = NormCol(w); // take the norm of w (wnorm))
    rcross31(w, wx); // call the rcross (wx) function
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

int IntegrateClosedLoop(float Rminus[3][3], float Bminus[3][1], float gyros[3][1], float mags[3][1],
        float accels[3][1], float magInertial[3][1], float accelInertial[3][1], float Kp_a,
        float Ki_a, float Kp_m, float Ki_m, float deltaT, float Bplus[3][1], float Rplus[3][3]) {
    float gyroInputWithBias[3][1];
    float wmeas_a[3][1];
    float wmeas_m[3][1];
    float wx[3][3];
    float mx[3][3];
    float result1[3][1];
    float result2[3][1];
    float result3[3][1];
    float result4[3][1];
    float result5[3][1];
    float gyroInputWithFeedback[3][1];
    float bdot[3][1];
    float R_exp1[3][3];
    for (int i = 0; i < DIM; i++) {

        accels[i][0] = accels[i][0] / NormCol(accels);

        mags[i][0] = mags[i][0] / NormCol(mags);
        magInertial[i][0] = magInertial[i][0] / NormCol(magInertial);
        accelInertial[i][0] = accelInertial[i][0] / NormCol(accelInertial);
        gyroInputWithBias[i][0] = gyros[i][0] - Bminus[i][0];
    }

    //wmeas_a = rcross(accels)*(Rminus * accelInertial)
    rcross31(accels, wx);

    MatrixMultiply31(Rminus, accelInertial, result1);

    MatrixMultiply31(wx, result1, wmeas_a);

    //wmeas_m = rcross(mags) * (Rminus * magInertial);   
    rcross31(mags, mx);
    MatrixMultiply31(Rminus, magInertial, result2);
    MatrixMultiply31(mx, result2, wmeas_m);


    //gyroInputWithFeedback = gyroInputWithBias + Kp_a*wmeas_a + Kp_m*wmeas_m;
    MatrixScalarMultiply31(Kp_a, wmeas_a, result3);
    MatrixScalarMultiply31(Kp_m, wmeas_m, result4);
    MatrixAdd31(result3, result4, result5);
    MatrixAdd31(result5, gyroInputWithBias, gyroInputWithFeedback);

    //bdot=-Ki_a*wmeas_a - Ki_m*wmeas_m;
    MatrixScalarMultiply31(-Ki_a, wmeas_a, result4);
    MatrixScalarMultiply31(-Ki_m, wmeas_m, result3);
    MatrixAdd31(result4, result3, bdot);

    //Rplus = Rexp(gyroInputWithFeedback, deltaT) * Rminus;
    RexpCol(gyroInputWithFeedback, deltaT, R_exp1);
    MatrixMultiply(R_exp1, Rminus, Rplus);

    //Bplus = Bminus + bdot*deltaT;
    MatrixScalarMultiply31(deltaT, bdot, result3);
    MatrixAdd31(Bminus, result3, Bplus);

    return SUCCESS;
}

int main(void) {
    BOARD_Init();
    BNO055_Init();
    //    OledInit();
    //    OledClear(OLED_COLOR_BLACK);
    //    OledUpdate();
    //    TIMERS_Init();
    //
    //    int time = 0;
    //    char OledOutput[100];
    //
    //    float RotationMatrix[3][3];
    //    float eulerAngles[3];
    //    float MatrixExponential;
    //    float gyroXYZ[3];
    //    float Rplus[3][3] = {};
    //    float Rminus[3][3] = {
    //        {1, 0, 0},
    //        {0, 1, 0},
    //        {0, 0, 1}
    //    };
    //
    //    // these are the values returned from the "part7_AccelMag_Misalignment.m" function
    //    float Rmisalignment[3][3] = {
    //        {0.8967, 0.0234, 0.4421},
    //        {0.2606, 0.7793, -0.5699},
    //        {-0.3579, 0.6262, 0.6927}
    //    };
    //
    //    float Raligned[3];
    //    EulerAngles(Rmisalignment, Raligned);
    //    printf("%f, %f, %f\r\n", Raligned[0], Raligned[1], Raligned[2]);
    //    int j;
    // the result from the above is: -39.444901, -26.237944, 1.494833

    //    while (1) {
    //        //printf("\n\n\n");
    //        //for (int i = 0; i <= 500; i++) {
    //
    //        //  printf("%d, %d, %d, %d, %d, %d\r\n", BNO055_ReadAccelX(), BNO055_ReadAccelY(), BNO055_ReadAccelZ(), BNO055_ReadMagX(), BNO055_ReadMagY(), BNO055_ReadMagZ());
    //        gyroXYZ[0] = ((BNO055_ReadGyroX() + GyroXOffset) / Raw2AngleConv) * deg2rad; // Read and convert X
    //        gyroXYZ[1] = ((BNO055_ReadGyroY() + GyroYOffset) / Raw2AngleConv) * deg2rad; // Read and convert Y
    //        gyroXYZ[2] = ((BNO055_ReadGyroZ() - GyroZOffset) / Raw2AngleConv) * deg2rad; // Read and convert Z
    //
    //        IntegrateOpenLoop(Rminus, gyroXYZ, DELTA_T, Rplus);
    //        EulerAngles(Rplus, eulerAngles);
    //        memcpy(Rminus, Rplus, 3 * 3 * sizeof (float));
    //
    //        sprintf(OledOutput, "Roll: %f\nPitch: %f\nYaw: %f\r\n", eulerAngles[1], eulerAngles[0], eulerAngles[2]); // print roll, pitch, yaw
    //        OledDrawString(OledOutput);
    //        OledUpdate();
    //
    //        time = TIMERS_GetMilliSeconds(); // make a start time
    //        while ((TIMERS_GetMilliSeconds() - time) < _20ms); // delay
    //    }

#ifdef PART6
    float gyroInput[3][1] = {
        {0},
        {0},
        {0}
    };
    float biasEstimate[3][1] = {
        {0},
        {0},
        {0}
    };
    float nvector[3][1] = {
        {1},
        {0},
        {0}
    };
    float evector[3][1] = {
        {0},
        {-1},
        {0}
    };
    float dvector[3][1] = {
        {0},
        {0},
        {-1}
    };
    float biasTerms[3][1] = {
        {0.1},
        {0.1},
        {0.1}
    };
    float accelInertial[3][1] = {
        {0},
        {0},
        {-1}
    };
    float accelReading[3][1] = {
        {0},
        {0},
        {-1}
    };
    float magInertial[3][1] = {
        {1},
        {0},
        {0}
    };
    float magReading[3][1] = {
        {1},
        {0},
        {0}
    };
    float Kp_a = 10, Ki_a = 1, Kp_m = 0, Ki_m = 0,
            angleX = 0.5236, angleY = 0.5236, angleZ = 0.5236,
            deltaT = .1;
    float rotX[3][3] = {
        {1, 0, 0},
        {0, 0.8660, -0.5000},
        {0, 0.5000, 0.8660}
    };
    float rotY[3][3] = {
        { 0.8660, 0, 0.5000},
        {0, 1, 0},
        {-0.5, 0, 0.8660}
    };
    float rotZ[3][3] = {
        {0.8660, -0.5000, 0},
        {0.5000, 0.8660, 0},
        {0, 0, 1}
    };
    float result1[3][3], R[3][3];
    float gyroInputWithBias[3][1];
    float Bplus[3][1];
    float Rplus[3][3];

    MatrixMultiply(rotX, rotY, result1);
    MatrixMultiply(result1, rotZ, R);
    printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\r");
    MatrixPrint(R);
    for (int k = 0; k < NUMSTEPS; k++) {
        MatrixAdd31(gyroInput, biasTerms, gyroInputWithBias);
        IntegrateClosedLoop(R, biasEstimate, gyroInputWithBias, magReading, accelReading, magInertial, accelInertial, Kp_a,
                Ki_a, Kp_m, Ki_m, deltaT, Bplus, Rplus);
        int i, j;
        for (i = 0; i < DIM; i++) {
            for (j = 0; j < DIM; j++) {
                R[i][j] = Rplus[i][j];
                biasEstimate[i][j] = Bplus[i][j];
            }
        }
        MatrixPrint(R);
    }
#endif

    return 0;
}