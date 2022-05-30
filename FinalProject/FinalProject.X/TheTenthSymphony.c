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

/*
 * 
 */
int main(int argc, char** argv) {
    /* Initializations*/
    BOARD_Init();
    BNO055_Init();
    AD_Init();
    AD_AddPins(AD_A1);
    
    
    /* Volume Control*/

    return (EXIT_SUCCESS);
}

