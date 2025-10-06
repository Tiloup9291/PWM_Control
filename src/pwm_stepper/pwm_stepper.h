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
#ifndef PWM_STEPPER_H
#define PWM_STEPPER_H
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <stdio.h>

enum DIRECTION{FORWARD = 0, REVERSE = 1};
enum MotorState{STOP = 0, START = 1};
enum KinematicVector{ACCELERATE = 1, DECELERATE = -1};
enum KinematicState{ACCEL = 1, DECEL = 2, CRUISE = 3, STOPPED = 0};

static const int sequence[4][4] = {
    {1,0,1,0},
    {0,1,1,0},
    {0,1,0,1},
    {1,0,0,1}
};

typedef struct {
    int gpio_aplus;
    int gpio_aminus;
    int gpio_bplus;
    int gpio_bminus;
    uint32_t stepByTurn;
    float maxRPM;
    float maxRPM_limit;
    float RPM;
    float currentRPM;
    float initialRPM;
    float maxAcceleration;
    float maxAcceleration_limit;
    float maxJerk;
    float maxJerk_limit;
    uint32_t waitTime;
    uint64_t last_change;
    uint64_t accelStartTime;
    int direction;
    int stateAplus;
    int stateAminus;
    int stateBplus;
    int stateBminus;
    int start;
    int accelDone;
    int stepIndex;
    float totalAccelTime_us;
    int kinematicState;
    float Tj;
    float Ta;
    float T_total;
    int kinematicVector;
} pwm_stepper_t;

uint64_t currentTime(void);
float fmaxf(float a, float b);
float fminf(float a, float b);
float sqrtf(float x);
float fabsf(float x);
float cbrtf(float x);
void pwm_stepper_init(pwm_stepper_t* stepper, int gpio_aplus, int gpio_aminus, int gpio_bplus,int gpion_bminus,uint32_t stepByTurn,float maxRPM,float maxAcceleration,float maxJerk, float totalAccelTime_us);
void pwm_stepper_set_speed(pwm_stepper_t* stepper, float rpm);
void pwm_stepper_set_direction(pwm_stepper_t* stepper, int direction);
void computeAccelerationProfile(pwm_stepper_t* stepper, float t_us);
void applyStep(pwm_stepper_t* stepper);
void pwm_stepper_update(pwm_stepper_t* stepper, int motorState);

#endif /* PWM_STEPPER_H  */
