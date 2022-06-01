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

#define _20ms 20

/*
 * 
 */
int main(void) {
    /* Initializations*/
    int angleZ = 0;
    int angleY = 0;
    int angleX = 0;
    BOARD_Init(); // initialize board and IMU components
    BNO055_Init();
    TIMERS_Init();
    while (1){
        int valX = (BNO055_ReadGyroX() + 11.02) / 131; // read gyro data, scale it and then print the correct value with a 20 ms delay
        int valY = (BNO055_ReadGyroY() + 24.95) / 131;
        int valZ = (BNO055_ReadGyroZ() - 12.15) / 131;
        angleZ = (angleZ + valZ);
        angleY = (angleY + valY);
        angleX = (angleX + valX);
        printf("Roll: %d   Pitch: %d   Yaw: %d\r\n", (angleX/40) % 180, (angleY/40) % 180, (angleZ/38) % 180 );
        int time = TIMERS_GetMilliSeconds(); // make a start time
        while ((TIMERS_GetMilliSeconds() - time) < _20ms); // delay
    }
}

