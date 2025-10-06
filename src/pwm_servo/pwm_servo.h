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
#ifndef PWM_SERVO_H
#define PWM_SERVO_H
#define DEFAULT_FREQ 100
#include <time.h>
#include <stdint.h>
#include <inttypes.h>
#include <unistd.h>

typedef struct {
    int gpio;
    uint32_t min_width;
    uint32_t max_width;
    uint32_t min_freq;
    uint32_t max_freq;
    float min_angle;
    float max_angle;
    float currentAngle;
    uint32_t currentAnglePulseWidth;
    uint32_t pulse_width_us;
    uint32_t period_us;
    uint64_t last_change;
    int state;
} pwm_servo_t;

uint64_t currentTime(void);
void pwm_servo_init(pwm_servo_t* servo, int gpio, uint32_t min_width, uint32_t max_width, uint32_t min_freq, uint32_t max_freq, float min_angle, float max_angle, float currentAngle);
void pwm_servo_set_angle(pwm_servo_t* servo, float angle_deg);
void pwm_servo_update(pwm_servo_t* servo);
#endif /* PWM_SERVO_H */
