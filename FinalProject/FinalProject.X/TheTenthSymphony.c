/* 
 * File:   TheTenthSymphony.c
 * We designed and built a device which mimics a conductor?s wand with the goal
 * to play music over a speaker based on the attitude of an inertial
 * measurement unit (IMU). We will use the 3-axis gyroscope to create musical
 * pitch differences based on the pitch and roll of the system. Every scale has
 * seven unique notes that make it up. The goal is to map each of these 
 * main notes to a positive and negative pitch, and a positive or negative roll,
 * while mapping the notes in-between to angles in-between.
 * Created on May 30, 2022, 11:38 AM
 */

/*Libraries*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
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

/*Defines*/
#define DELAY(x)        {int wait; for (wait = 0; wait <= x; wait++) {asm("nop");}}
#define A_LOT           183000
#define A_BIT           1830
#define _20ms           20
#define ANGLE_BOUND     40
#define SCALER_40       40
#define SCALER_38       38
#define TWELFTHROOT_OF2 1.059463
#define TONE_A          440
#define OCTAVE_JUMP     12
#define ROOT            0
#define SECOND          2
#define THIRD           4
#define FOURTH          5
#define FIFTH           7 
#define SIXTH           9
#define M_SEVENTH       10
#define SEVENTH         11


/* This function chooses the tone that will come out of the speaker based on the
 * roll pitch or yaw of the gyroscope based on the B major scale. If any of these
 * angles are above 40 degrees, the tone will be chosen based on the axis with 
 * the most extreme attitude*/
void TonePick(int angleX, int angleY, int angleZ, int octave) {
    long double frequency;
    int step = 0;
    int frequency_int;
    
   // If the yaw, pitch, or roll is at a more extreme angle than 40 degrees
   // pick the note above the root based on the most extreme attitude
    if ((abs(angleX) > ANGLE_BOUND) || (abs(angleY) > ANGLE_BOUND) || (abs(angleZ) > ANGLE_BOUND)) {
        if (abs(angleX) > abs(angleY) && abs(angleX) > abs(angleZ)) {
            if (angleX < 0) {
                step = SEVENTH;
            } else {
                step = FOURTH;
            }
        } else if (abs(angleY) > abs(angleX) && abs(angleY) > abs(angleZ)) {
            if (angleY < 0) {
                step = SIXTH;
            } else {
                step = THIRD;
            }
        } else {
            if (angleZ < 0) {
                step = FIFTH;
            } else {
                step = SECOND;
            }
        }
    //else the chosen tone will be the root note
    } else {
        step = ROOT;
    }
    //f(n)= 440* 12th_root(2)^(key_number)
    frequency = TONE_A * pow(TWELFTHROOT_OF2, (step + OCTAVE_JUMP * octave) 
            - M_SEVENTH);
  
    //Frequency is converted from float to int and sent to the pin 3
    frequency_int = (int) frequency;
    ToneGeneration_SetFrequency(frequency_int);
}

int main(void) {
    /* Initializations*/
    BOARD_Init(); // initialize board and IMU components
    DELAY(A_LOT);

    BNO055_Init();
    DELAY(A_BIT);
    TIMERS_Init();
    AD_Init();
    ToneGeneration_Init();
    ToneGeneration_SetFrequency(200);
    ToneGeneration_ToneOn();
    
    /* Variables */
    int angleZ = 0;
    int angleY = 0;
    int angleX = 0;
    int flexVal = 0;
    int flexVal2 = 0;
    int flexAngle = 0;
    int flexAngle2 = 0;
    int octave = 0;

    /* Analog to digital pins for reading voltage */
    AD_AddPins(AD_A1);
    AD_AddPins(AD_A2);

    while (1) {
        // read gyro data, scale it and then print the correct value with a 20 ms delay
        int valX = (BNO055_ReadGyroX() + 11.02) / 131; 
        int valY = (BNO055_ReadGyroY() + 24.95) / 131;
        int valZ = (BNO055_ReadGyroZ() - 12.15) / 131;
        angleZ = (angleZ + valZ);
        angleY = (angleY + valY);
        angleX = (angleX + valX);
    

        int time = TIMERS_GetMilliSeconds(); // make a start time
        while ((TIMERS_GetMilliSeconds() - time) < _20ms); // delay
        if (AD_IsNewDataReady()) {
            flexVal = AD_ReadADPin(AD_A1); // read and store the reading
            flexVal2 = AD_ReadADPin(AD_A2);
            flexAngle = (3.27 * flexVal) + 349; // Eq. from Excel, using a 67k 
            flexAngle2 = (3.27 * flexVal2) + 349; // Eq. from Excel, using a 67k 
        }

        // turn off tone when bend sensor is bent
        if (flexAngle > 2000) {
            ToneGeneration_ToneOff();
        } else {
            ToneGeneration_ToneOn();
        }
        
        // raise the tone by an octave when 2nd bend sensor is bent
        if (flexAngle2 > 2500) {
            octave = 1;
        } else {
            octave = 0;
        }
        //pick the tone using calibrated scaling factors
        TonePick(angleX / SCALER_40, angleY / SCALER_40, angleZ / SCALER_38, octave);
    }
}

