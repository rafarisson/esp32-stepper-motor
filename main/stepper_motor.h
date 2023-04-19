#ifndef __STEPPER_MOTOR_H__
#define __STEPPER_MOTOR_H__

#include "driver/ledc.h"
#include "driver/pulse_cnt.h"

#define MOTOR_EN_LEVEL		0

typedef struct {
	pcnt_unit_handle_t pcnt_unit;
	pcnt_channel_handle_t pcnt_ch;
	const int en_io;
	const int dir_io;
	const int step_io;
	const int num_of_steps;
	int steps;
	int steps_left;
}
stepper_motor_t;

void motor_init(stepper_motor_t *motor);
void motor_stop(stepper_motor_t *motor);
bool motor_start(stepper_motor_t *motor, int steps, int rpm);

#endif
