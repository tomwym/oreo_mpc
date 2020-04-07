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
    int cnt = 0;
    while(1) {
        if(cnt < 32) {
            DisableEyeCtrl();
            //DisableNeckCtrl();
            usleep(1000);
            cnt++;
        }
    };  
}