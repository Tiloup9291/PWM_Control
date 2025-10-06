/*
 *The source code from the project PWM_Control
 *Copyright (C) 2025  John Doe

 *This program is free software: you can redistribute it and/or modify
 *it under the terms of the GNU General Public License as published by
 *the Free Software Foundation, version 3 of the License, GPL-3.0-only.

 *This program is distributed in the hope that it will be useful,
 *but WITHOUT ANY WARRANTY; without even the implied warranty of
 *MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *GNU General Public License for more details.

 *You should have received a copy of the GNU General Public License
 *along with this program.  If not, see <https://www.gnu.org/licenses/>
*/
#include <stdio.h>
#include <stdlib.h>
#include "pwm_servo.h"

int main()
{
    pwm_servo_t servo1;
    pwm_servo_init(&servo1,9,500,2500,50,330,0,270,0);
    while (1) {
        pwm_servo_update(&servo1);

        pwm_servo_set_angle(&servo1, 180);

        printf("angle pulse width : %" PRIu32 "\n",servo1.currentAnglePulseWidth);
        printf("pulse width max: %" PRIu32 "\n",servo1.pulse_width_us);
        printf("period : %" PRIu32 "\n",servo1.period_us);
        printf("state : %d\n",servo1.state);
        printf("last time : %" PRIu64 "\n",servo1.last_change);
        usleep(100);
    }
    return 0;
}
