/*
 * system_cmd.c
 *
 *  Created on: Jul 31, 2019
 *      Author: wt
 */
#include "protocal_data_handle.h"
#include "cmsis_os.h"
#include "system_cmd.h"
#include "lens.h"
#include "ceju_data_handle.h"
#include "drv_uart.h"
#include "cmd.h"
#include "2d_bracket.h"
#include "protect_structure.h"
#include "m62429.h"

struct manifold_cmd manifold_cmd;

static int8_t auto_focus(uint8_t *buff, uint16_t len);
static int8_t left_motor_move(uint8_t *buff, uint16_t len);
static int8_t right_motor_move(uint8_t *buff, uint16_t len);
static int8_t ud_stepper_move(uint8_t *buff, uint16_t len);
static int8_t lr_stepper_move(uint8_t *buff, uint16_t len);
static int8_t protect_stepper_move(uint8_t *buff, uint16_t len);
static int8_t set_volume(uint8_t *buff, uint16_t len);
static int8_t get_distance(uint8_t *buff, uint16_t len);

int16_t stride;

lens_t plens;
bracket_t pbracket;
protect_structure_t pprotect_structure;
void system_cmd_task(void const * argument)
{
	osDelay(1000);
	stop_distance_measure();

	plens = lens_find("lens");
	pbracket = bracket_find("bracket");
	pprotect_structure = protect_structure_find("protect_structure");
	osEvent event;

	protocol_rcv_cmd_register(AUTO_FOCUS,auto_focus);

	protocol_rcv_cmd_register(LEFT_MOTOR_MANUAL_FOCUS_PLUS,left_motor_move);
	protocol_rcv_cmd_register(LEFT_MOTOR_MANUAL_FOCUS_MINUS,left_motor_move);

	protocol_rcv_cmd_register(RIGHT_MOTOR_MANUAL_FOCUS_PLUS,right_motor_move);
	protocol_rcv_cmd_register(RIGHT_MOTOR_MANUAL_FOCUS_MINUS,right_motor_move);

	protocol_rcv_cmd_register(M3_FORWARD,ud_stepper_move);
	protocol_rcv_cmd_register(M3_REVERSAL,ud_stepper_move);

	protocol_rcv_cmd_register(M4_FORWARD,lr_stepper_move);
	protocol_rcv_cmd_register(M4_REVERSAL,lr_stepper_move);

	protocol_rcv_cmd_register(PROTECT_STEPPER_MOVE,protect_stepper_move);

	protocol_rcv_cmd_register(SET_VOLUME,set_volume);

	protocol_rcv_cmd_register(MEASURE_DISTANCE,get_distance);

  for(;;)
  {
  	event = osSignalWait(MANIFOLD2_DISTANCE_SIGNAL, osWaitForever);

		if (event.status == osEventSignal)
		{
			if(event.value.signals & MANIFOLD2_DISTANCE_SIGNAL)
			{
				osThreadSuspend(LensTaskHandle);
				plens->auto_focus_flag = TRUE;
				plens->motor[left_motor].goto_zp_flag = TRUE;
				plens->motor[right_motor].goto_zp_flag = TRUE;
				__HAL_TIM_SET_COMPARE(plens->motor[left_motor].pwm_tim_handle,plens->motor[left_motor].pwm_channel,1000);
				__HAL_TIM_SET_COMPARE(plens->motor[right_motor].pwm_tim_handle,plens->motor[right_motor].pwm_channel,1000);
				lens_set_motors_target_by_distance(manifold_cmd.distance,plens);
				stop_distance_measure();
			}
		}

    osDelay(1);
  }
}


static int8_t auto_focus(uint8_t *buff, uint16_t len)
{
	start_distance_measure();
	return 0;
}

static int8_t left_motor_move(uint8_t *buff, uint16_t len)
{
	if(buff[0] == LEFT_MOTOR_MANUAL_FOCUS_PLUS)
	{
		stride = 500;
	}
	else if(buff[0] == LEFT_MOTOR_MANUAL_FOCUS_MINUS)
	{
		stride = -500;
	}
	else
	{
		return -1;
	}

	plens->motor[left_motor].target += stride;
	VAL_LIMIT(plens->motor[left_motor].target,MOTOR_TARGET_MIN,MOTOR_TARGET_MAX);
	return 0;
}


static int8_t right_motor_move(uint8_t *buff, uint16_t len)
{
	if(buff[0] == RIGHT_MOTOR_MANUAL_FOCUS_PLUS)
		{
			stride = 500;
		}
		else if(buff[0] == RIGHT_MOTOR_MANUAL_FOCUS_MINUS)
		{
			stride = -500;
		}
		else
		{
			return -1;
		}
		plens->motor[right_motor].target += stride;
		VAL_LIMIT(plens->motor[right_motor].target,MOTOR_TARGET_MIN,MOTOR_TARGET_MAX);
	return 0;
}

static int8_t ud_stepper_move(uint8_t *buff, uint16_t len)
{
	if(buff[0] == M3_FORWARD)
	{
		stride = -500;
	}
	else if(buff[0] == M3_REVERSAL)
	{
		stride = 500;
	}
	else
	{
		return -1;
	}

	stepper_run(&pbracket->stepper[up_down_stepper], stride);

	return 0;
}

static int8_t lr_stepper_move(uint8_t *buff, uint16_t len)
{
	if(buff[0] == M4_FORWARD)
	{
		stride = 500;
	}
	else if(buff[0] == M4_REVERSAL)
	{
		stride = -500;
	}
	else
	{
		return -1;
	}

	stepper_run(&pbracket->stepper[left_right_stepper], stride);

	return 0;
}


static int8_t protect_stepper_move(uint8_t *buff, uint16_t len)
{

	if(buff[0] == PROTECT_STEPPER_MOVE)
	{
		stride = (int16_t)(buff[2]<<8 | buff[3]);
	}
	else
	{
		return -1;
	}

//	protect_structure_stepper_run(pprotect_structure, stride);
	protect_structure_motor_run(stride);
	return 0;
}

static int8_t set_volume(uint8_t *buff, uint16_t len)
{
	if(buff[0] == SET_VOLUME)
	{
		m62429_control(&buff[1],1);
	}
	return 0;
}

static int8_t get_distance(uint8_t *buff, uint16_t len)
{
	if(buff[0] == MEASURE_DISTANCE)
	{
		cj_get_distance();
	}
	return 0;
}


