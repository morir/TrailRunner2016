/*
 * PID.c
 *
 * Created: 2014/10/14 9:29:00
 *  Author: Administrator
 */ 

#include "pid.h"
#include "stdint.h"
#include "DebugLog.h"

// PID Param
#define K_P     0.70     // P param 1.00�FPID����̃p�����[�^(������ō��킹���񂾂̂Ő����ł��܂���)
#define K_I     0.50    // I param 5.00�FPID����̃p�����[�^(������ō��킹���񂾂̂Ő����ł��܂���)
#define K_D     0.50  // D param 0.0003�FPID����̃p�����[�^(������ō��킹���񂾂̂Ő����ł��܂���)
//�����̐��l��傫������ƁA���i�y�я����Ȃ��鎞�̑��x��������B
#define pid_base 500   // base speed P_CW_SPEED_NOMAL 500
#define pid_lim 30     // control value 30�FPID����̃p�����[�^(������ō��킹���񂾂̂Ő����ł��܂���)
#define DELTA_T 0.002   // delta T 0.002�FPID����̃p�����[�^(������ō��킹���񂾂̂Ő����ł��܂���)
//PID����̌v�Z���ʂŁA�������傫������Ƃ��̏���A�����l�B
//�x�[�X���x�ɑ΂��Ă��̐��l���܂ł�+-�����e���܂�
#define offSet_val 250 //450;//300

int diff[2]    = {0,0};
float integral = 0.0;

// Set up PID controller parameters
void pid_init(int16_t p_factor, int16_t i_factor, int16_t d_factor, struct PID_DATA *pid) {
	// Start values for PID controller
	pid->sumError = 0;
	pid->lastProcessValue = 0;
	// Tuning constants for PID loop
	pid->P_Factor = p_factor;
	pid->I_Factor = i_factor;
	pid->D_Factor = d_factor;
	// Limits to avoid overflow
	pid->maxError = MAX_INT / (pid->P_Factor + 1);
	pid->maxSumError = MAX_I_TERM / (pid->I_Factor + 1);
}

int16_t pid_Controller(int16_t setPoint, int16_t processValue, struct PID_DATA *pid_st) {
	int16_t error, p_term, d_term;
	int32_t i_term, ret, temp;
	
	error = setPoint - processValue;
	
	// Calculate Pterm and limit error overflow
	if (error > pid_st->maxError) {
		p_term = MAX_INT;
	} else if (error < -pid_st->maxError) {
		p_term = -MAX_INT;
	} else {
		p_term = pid_st->P_Factor * error;
	}
	
	// Calculate Iterm and limit integral runaway
	temp = pid_st->sumError + error;
	if (temp > pid_st->maxSumError) {
		i_term = MAX_I_TERM;
		pid_st->sumError = pid_st->maxSumError;
	} else if (temp < -pid_st->maxSumError) {
		i_term = -MAX_I_TERM;
		pid_st->sumError = -pid_st->maxSumError;
	} else {
		pid_st->sumError = temp;
		i_term = pid_st->I_Factor * pid_st->sumError;
	}
	
	// Calculate Dterm
	d_term = pid_st->D_Factor * (pid_st->lastProcessValue - processValue);
	
	pid_st->lastProcessValue = processValue;
	
	ret = (p_term + i_term + d_term) / SCALING_FACTOR;
	if (ret > MAX_INT) {
		ret = MAX_INT;
	} else if (ret < -MAX_INT) {
		ret = -MAX_INT;
	}
	
	return((int16_t)ret);
}

void pid_Reset_Integrator(pidData_t *pid_st) {
	pid_st->sumError = 0;
}

/**
 * PID����̐���l���X�V
 * @brief PID����̐���l���X�V
 * @param (int target_val) 0�Œ�
 * @param (int sencer_val) PID_ctlr
 * @param (int *out_rightVal) �E���[�^�̒l
 * @param (int *out_leftVal) �����[�^�̒l
 * @return �Ȃ�
 */
void PID_ctlr_Update(int target_val, int sencer_val, int *out_rightVal, int *out_leftVal) {
	float p,i,d;
	
	diff[0] = diff[1];
	diff[1] = sencer_val - target_val;
	
	integral += ((diff[0] + diff[1]) / 2.0 * DELTA_T);
	
	p = K_P * diff[1];
	i = K_I * integral;
	d = K_D * ((diff[1] - diff[0]) / DELTA_T);
	
	int MV = (int)(pid_lim * (p + i + d));
	
	LOG_INFO( "MV = %d\r\n", MV );
	
	int right_val = pid_base - MV;
	int left_val = pid_base + MV;
	if (right_val > pid_base + offSet_val) {//300
		right_val = pid_base + offSet_val;
		} else if (right_val < pid_base - offSet_val) {
		right_val = pid_base - offSet_val;
	}
	if (left_val > pid_base + offSet_val) {
		left_val = pid_base + offSet_val;
		} else if (left_val < pid_base - offSet_val) {
		left_val = pid_base - offSet_val;
	}
	
	*out_rightVal = right_val + 1023;
	*out_leftVal  = left_val;
}

/**
 * PID��diff[]��integral��0�ŏ���������B
 * @brief PID��diff[]��0�ŏ���������B
 * @return �Ȃ�
 */
void PID_reset_diff_integral()
{
	diff[0] = 0;
	diff[1] = 0;
	integral = 0.0;
}