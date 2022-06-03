/* 
 * File:   TheTenthSymphony.c
 * Author: zoeal
 *
 * Created on May 30, 2022, 11:38 AM
 */

#include <stdio.h>
#include <stdlib.h>
#include "BNO055.h"
#include "BOARD.h"
#include "AD.h"
#include "serial.h"
#include "ToneGeneration.h"
#include "Oled.h"
#include "OledDriver.h"
#include "Ascii.h"
#include "I2C.h"
#include "timers.h"
#include <math.h>

#define DELAY(x)    {int wait; for (wait = 0; wait <= x; wait++) {asm("nop");}}
#define A_LOT       183000
#define A_BIT       1830
#define _20ms 20

void TonePick(int angleX, int angleY, int angleZ, int note) {
    long double frequency;
    int step = 1;
    int frequency_int;
    printf("Roll: %d   Pitch: %d   Yaw: %d\r\n", abs(angleX), abs(angleY), abs(angleZ));

    if ((abs(angleX) > 40) || (abs(angleY) > 40) || (abs(angleZ) > 40)) {
        printf("1Roll1: %d   1Pitch1: %d   1Yaw1: %d\r\n", abs(angleX), abs(angleY), abs(angleZ));

        if (abs(angleX) > abs(angleY) && abs(angleX) > abs(angleZ)) {
            if (angleX < 0) {
                step = 10;
            } else {
                step = 5;
            }
        } else if (abs(angleY) > abs(angleX) && abs(angleY) > abs(angleZ)) {
            if (angleY < 0) {
                step = 9;
            } else {
                step = 4;
            }
        } else {
            if (angleZ < 0) {
                step = 7;
            } else {
                step = 2;
            }
        }
    } else {
        step = 0;
    }
    frequency = 440 * pow(1.059463, step);
    frequency_int = (int) frequency;
    printf("BBBBFrequency %d\r\n", frequency_int);
    ToneGeneration_SetFrequency(frequency_int);
}

int main(void) {
    /* Initializations*/
    int angleZ = 0;
    int angleY = 0;
    int angleX = 0;
    int note = 49;
    printf("HERE1");

    BOARD_Init(); // initialize board and IMU components
    DELAY(A_LOT);
    printf("HERE2");

    BNO055_Init();
    DELAY(A_BIT);
    TIMERS_Init();
    AD_Init();
    ToneGeneration_Init();
    ToneGeneration_SetFrequency(200);
    ToneGeneration_ToneOn();
    printf("HERE");

    int flexVal = 0;
    int flexAngle = 0;
    AD_AddPins(AD_A1);
    while (1) {
        int valX = (BNO055_ReadGyroX() + 11.02) / 131; // read gyro data, scale it and then print the correct value with a 20 ms delay
        int valY = (BNO055_ReadGyroY() + 24.95) / 131;
        int valZ = (BNO055_ReadGyroZ() - 12.15) / 131;
        angleZ = (angleZ + valZ);
        angleY = (angleY + valY);
        angleX = (angleX + valX);
<<<<<<< HEAD
        // printf("Roll: %d   Pitch: %d   Yaw: %d\r\n", angleX/40, angleY/40, angleZ/ 38);
=======
        printf("Roll: %d   Pitch: %d   Yaw: %d\r\n", (angleX/40) % 180, (angleY/40) % 180, (angleZ/38) % 180 );
>>>>>>> 0933f126394cbcfc049a3d739167ac9ca2f72da1
        int time = TIMERS_GetMilliSeconds(); // make a start time
        while ((TIMERS_GetMilliSeconds() - time) < _20ms); // delay
        if (AD_IsNewDataReady()) {
            flexVal = AD_ReadADPin(AD_A1); // read and store the reading
            flexAngle = (3.27 * flexVal) + 349; // Eq. from Excel, using a 67k 
        }

        printf("Flexangle %d\r\n", flexAngle);
        if (flexAngle > 2000) {
            ToneGeneration_ToneOff();
        } else {
            ToneGeneration_ToneOn();

        }
        TonePick(angleX / 40, angleY / 40, angleZ / 38, note);

    }
}

