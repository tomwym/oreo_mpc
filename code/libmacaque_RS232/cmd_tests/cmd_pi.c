#include "libmacaque_RS232/macaque_linux.h"
#include "TML_RS232_lib/include/TML_RS232_lib.h"
#include "sock_interface/sock.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define BUFF_SIZE       (255)

int main()
{
    Start();

    // 
    StartEyeCal(EYE_YAW_LEFT_AXIS, 10.5);

    SetNeckPosn(NECK_PITCH_AXIS, -0.95);
    SetNeckSpeed(NECK_ROLL_AXIS, 1.342);
    SetNeckAccel(NECK_YAW_AXIS, 1.054);

    while(1) {};

    Cleanup();
}