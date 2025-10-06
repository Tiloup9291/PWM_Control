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
#include "pwm_servo.h"
uint64_t currentTime(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)(ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000);
}

void pwm_servo_init(pwm_servo_t* servo, int gpio, uint32_t min_width, uint32_t max_width, uint32_t min_freq, uint32_t max_freq,float min_angle,float max_angle, float currentAngle){
    servo->gpio = gpio;
    if (min_width > max_width){
        servo->min_width = max_width;
        servo->max_width = min_width;
    }else {
        servo->min_width = min_width;
        servo->max_width = max_width;
    }
    servo->pulse_width_us = servo->max_width - servo->min_width;
    if (min_freq > max_freq){
        servo->min_freq = max_freq;
        servo->max_freq = min_freq;
    } else if (min_freq == max_freq){
	servo->min_freq = 0;
	servo->max_freq = max_freq*2;
    } else {
        servo->min_freq = min_freq;
        servo->max_freq = max_freq;
    }
    if ((servo->max_freq - servo->min_freq) == 0){
	servo->min_freq = 0;
	servo->max_freq = DEFAULT_FREQ;
    }
    if (min_angle > max_angle){
        servo->min_angle = max_angle;
        servo->max_angle = min_angle;
    }else {
        servo->min_angle = min_angle;
        servo->max_angle = max_angle;
    }
    if (currentAngle < servo->min_angle){
        servo->currentAngle = servo->min_angle;
    }else if (currentAngle > servo->max_angle){
        servo->currentAngle = servo->max_angle;
    }else {
        servo->currentAngle = currentAngle;
    }
    servo->period_us = (uint32_t)((1.0/(servo->min_freq + ((servo->max_freq - servo->min_freq)/2.0)))*1000000);
    if ( min_angle == max_angle){
	servo->currentAnglePulseWidth = servo->min_width;
	servo->currentAngle = servo->min_angle;
    } else {
	float ratio = ((servo->currentAngle - servo->min_angle)/(servo->max_angle-servo->min_angle));
    	servo->currentAnglePulseWidth = servo->min_width + (uint32_t)(ratio*servo->pulse_width_us);
	servo->currentAngle = currentAngle;
    }
    servo->last_change = 0;
    servo->state = 0;
}

void pwm_servo_set_angle(pwm_servo_t* servo, float angle_deg){
    if (angle_deg < servo->min_angle) angle_deg = servo->min_angle;
    if (angle_deg > servo->max_angle) angle_deg = servo->max_angle;

    if (servo->min_angle == servo->max_angle){
	servo->currentAnglePulseWidth = servo->min_width;
	servo->currentAngle = servo->min_angle;
    } else {
	float ratio = ((angle_deg - servo->min_angle) / (servo->max_angle-servo->min_angle));
    	servo->currentAnglePulseWidth = servo->min_width + (uint32_t)(ratio * servo->pulse_width_us);
	servo->currentAngle = angle_deg;
    }
}

void pwm_servo_update(pwm_servo_t* servo) {
    uint64_t now = currentTime();

    if (servo->state == 0) {
        if (now - servo->last_change >= (servo->period_us - servo->currentAnglePulseWidth)) {
            servo->last_change = now;
            servo->state = 1;
        }
    } else {
        if (now - servo->last_change >= servo->currentAnglePulseWidth) {
            servo->last_change = now;
            servo->state = 0;
        }
    }
}
