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
#include "pwm_stepper.h"

int main()
{
    pwm_stepper_t stepper1;
    pwm_stepper_init(&stepper1, 1,2,3,4,200,3000.0,6000.0,12000.0,1000000);
    pwm_stepper_set_direction(&stepper1, FORWARD);
    pwm_stepper_set_speed(&stepper1,3000);
    while(1){
        pwm_stepper_update(&stepper1,START);
        for (int i = 0; i < 40; ++i) {
        pwm_stepper_update(&stepper1, START);

        printf("[t=%lu µs] RPM: %.2f | Etape: %d | Dir: %s | A+:%d A-:%d B+:%d B-:%d\n",
               currentTime(),
               stepper1.currentRPM,
               stepper1.stepIndex,
               stepper1.direction == FORWARD ? "FORWARD" : "REVERSE",
               stepper1.stateAplus,
               stepper1.stateAminus,
               stepper1.stateBplus,
               stepper1.stateBminus);

        usleep(50000);
    }
    for (int i = 0; i < 40; ++i) {
        pwm_stepper_update(&stepper1, STOP);

        printf("[t=%lu µs] RPM: %.2f | Etape: %d | Dir: %s | A+:%d A-:%d B+:%d B-:%d\n",
               currentTime(),
               stepper1.currentRPM,
               stepper1.stepIndex,
               stepper1.direction == FORWARD ? "FORWARD" : "REVERSE",
               stepper1.stateAplus,
               stepper1.stateAminus,
               stepper1.stateBplus,
               stepper1.stateBminus);

        usleep(50000);
    }
    break;
    }
    return 0;
}
