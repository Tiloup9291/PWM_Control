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
#include "pwm_stepper.h"

uint64_t currentTime(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)(ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000);
}

float fmaxf(float a, float b){
    return (a > b) ? a : b;
}

float fminf(float a, float b){
    return (a < b) ? a : b;
}

float sqrtf(float x){
    if (x <= 0.0f) return 0.0f;

    float guess = x / 2.0f;
    for (int i = 0; i < 10; ++i) {
        guess = 0.5f * (guess + x / guess);
    }
    return guess;
}

float fabsf(float x){
    return (x < 0.0f) ? -x : x;
}

float cbrtf(float x) {
    if (x == 0.0f) return 0.0f;

    float guess = x > 1.0f ? x / 3.0f : x;

    for (int i = 0; i < 20; i++) {
        guess = (2.0f * guess + x / (guess * guess)) / 3.0f;
    }

    return guess;
}

void pwm_stepper_init(pwm_stepper_t* stepper, int gpio_aplus, int gpio_aminus, int gpio_bplus,int gpio_bminus,uint32_t stepByTurn,float maxRPM,float maxAcceleration,float maxJerk, float totalAccelTime_us){
    stepper->gpio_aplus = gpio_aplus;
    stepper->gpio_aminus = gpio_aminus;
    stepper->gpio_bplus = gpio_bplus;
    stepper->gpio_bminus = gpio_bminus;
    stepper->stepByTurn = stepByTurn;
    stepper->maxRPM = maxRPM;
    stepper->maxRPM_limit = maxRPM;
    stepper->currentRPM = 0.0f;
    stepper->RPM = 0.0f;
    stepper->initialRPM = 0.0f;
    stepper->maxAcceleration = maxAcceleration;
    stepper->maxAcceleration_limit = maxAcceleration;
    stepper->maxJerk = maxJerk;
    stepper->maxJerk_limit = maxJerk;
    stepper->waitTime = 0;
    stepper->last_change = 0;
    stepper->accelStartTime = 0;
    stepper->direction = FORWARD;
    stepper->stateAplus = 0;
    stepper->stateAminus = 0;
    stepper->stateBplus = 0;
    stepper->stateBminus = 0;
    stepper->start = STOP;
    stepper->accelDone = 0;
    stepper->stepIndex = 0;
    stepper->totalAccelTime_us = totalAccelTime_us;
    stepper->kinematicVector = 0;
    stepper->kinematicState = STOPPED;
    stepper->Tj = 0;
    stepper->Ta = 0;
    stepper->T_total = 0;
}

void pwm_stepper_set_speed(pwm_stepper_t* stepper, float rpm){
    if (rpm == stepper->RPM) return;
    rpm = fminf(fmaxf(rpm, 0.0f), stepper->maxRPM_limit);
    if (rpm == stepper->RPM) return;

    stepper->initialRPM = stepper->currentRPM;
    stepper->RPM = rpm;
    stepper->accelStartTime = currentTime();

    float deltaV = fabsf(stepper->RPM - stepper->initialRPM);
    float a_max = stepper->maxAcceleration;
    float j_max = stepper->maxJerk;

    stepper->kinematicVector = (stepper->RPM > stepper->initialRPM) ? ACCELERATE : DECELERATE;

    float Tj_min = a_max / j_max;

    float Tj, Ta, T_total;

    if (deltaV < j_max * Tj_min * Tj_min) {

        Tj = sqrtf(deltaV / j_max);
        Ta = 0.0f;
        T_total = 2.0f * Tj;

        float adjusted_jerk = deltaV / (Tj * Tj);
        if (adjusted_jerk > stepper->maxJerk_limit) {
            adjusted_jerk = stepper->maxJerk_limit;
            Tj = sqrtf(deltaV / adjusted_jerk);
            T_total = 2.0f * Tj;
        }

        stepper->maxJerk = adjusted_jerk;

    } else {
        Tj = Tj_min;
        Ta = (deltaV / a_max) - Tj;
        T_total = 2.0f * Tj + Ta;
        stepper->maxJerk = j_max;
    }

    float T_target = stepper->totalAccelTime_us / 1e6f;
    if (T_total < T_target && T_target > 0.0f) {
        Tj = T_target / 2.0f;
        Ta = 0.0f;

        float adjusted_jerk = deltaV / (Tj * Tj);

        if (adjusted_jerk > stepper->maxJerk_limit) {
            adjusted_jerk = stepper->maxJerk_limit;
            Tj = sqrtf(deltaV / adjusted_jerk);
            T_total = 2.0f * Tj;
        }

        stepper->maxJerk = adjusted_jerk;
        T_total = 2.0f * Tj;
    }

    stepper->Tj = Tj;
    stepper->Ta = Ta;
    stepper->T_total = T_total;

    stepper->kinematicState = (stepper->RPM == 0.0f) ? DECEL : ACCEL;

}

void pwm_stepper_set_direction(pwm_stepper_t* stepper, int direction){
    if (direction == FORWARD){
        stepper->direction = FORWARD;
    } else if (direction == REVERSE){
        stepper->direction = REVERSE;
    } else {
        stepper->direction = FORWARD;
    }
}

void computeAccelerationProfile(pwm_stepper_t* stepper, float t){
    float deltaV = stepper->RPM - stepper->initialRPM;
    float sign = (deltaV > 0) ? 1.0f : -1.0f;
    deltaV = fabsf(deltaV);

    if (t < 0) {
        stepper->currentRPM = stepper->initialRPM;
        return;
    }
    if (t >= stepper->T_total) {
        stepper->currentRPM = stepper->RPM;
        return;
    }

    float v = 0.0f;
    float j = stepper->maxJerk;
    float a = stepper->maxAcceleration;
    float Tj = stepper->Tj;
    float Ta = stepper->Ta;

    if (Ta == 0.0f) {
        if (t < Tj) {
            v = stepper->initialRPM + sign * 0.5f * j * t * t;
        } else {
            float t2 = t - Tj;
            float vTj = 0.5f * j * Tj * Tj;
            v = stepper->initialRPM + sign * (vTj + j * Tj * t2 - 0.5f * j * t2 * t2);
        }
    } else {

        if (t < Tj) {
            v = stepper->initialRPM + sign * 0.5f * j * t * t;
        } else if (t < (Tj + Ta)) {
            float t1 = t - Tj;
            float v1 = 0.5f * j * Tj * Tj;
            v = stepper->initialRPM + sign * (v1 + a * t1);
        } else {
            float t2 = t - Tj - Ta;
            float v1 = 0.5f * j * Tj * Tj;
            float v2 = a * Ta;
            v = stepper->initialRPM + sign * (v1 + v2 + a * t2 - 0.5f * j * t2 * t2);
        }
    }

    stepper->currentRPM = fmaxf(0.0f, v);

}

void applyStep(pwm_stepper_t* stepper){
    stepper->stateAplus = sequence[stepper->stepIndex][0];
    stepper->stateAminus = sequence[stepper->stepIndex][1];
    stepper->stateBplus = sequence[stepper->stepIndex][2];
    stepper->stateBminus = sequence[stepper->stepIndex][3];
}

void pwm_stepper_update(pwm_stepper_t* stepper, int motorState){
    uint64_t now = currentTime();
    float t = (float)(now - stepper->accelStartTime) / 1e6f;

    if (motorState == STOP) {
        if (stepper->RPM != 0.0f && stepper->kinematicState != DECEL) {
            pwm_stepper_set_speed(stepper, 0.0f);
        }

        now = currentTime();
        t = (float)(now - stepper->accelStartTime) / 1e6f;

        computeAccelerationProfile(stepper, t);

        if (t >= stepper->T_total || stepper->currentRPM <= 0.0f) {
            stepper->currentRPM = 0.0f;
            stepper->kinematicState = STOPPED;
            return;
        }
    } else if (motorState == START) {
        if (stepper->kinematicState != STOPPED) {
            computeAccelerationProfile(stepper, t);

            if (t >= stepper->T_total) {
                stepper->currentRPM = stepper->RPM;
                stepper->kinematicState = (stepper->RPM == 0.0f) ? STOPPED : CRUISE;
            }
        }
    }

    if (stepper->currentRPM > 0.0f) {
        float period_s = 60.0f / (stepper->currentRPM * stepper->stepByTurn);
        unsigned long period_us = (unsigned long)(period_s * 1e6f);
        if (now - stepper->last_change >= period_us) {
            applyStep(stepper);
            stepper->stepIndex = (stepper->direction == FORWARD)
                                 ? (stepper->stepIndex + 1) % 4
                                 : (stepper->stepIndex + 3) % 4;
            stepper->last_change = now;
        }
    }
}
