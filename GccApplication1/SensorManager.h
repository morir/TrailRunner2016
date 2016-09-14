/*
 * SensorManager.h
 *
 * Created: 2014/08/19 18:57:13
 *  Author: Administrator
 */ 

#ifndef SENSORMANAGER_H_
#define SENSORMANAGER_H_

#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/common.h>

/* �ԊO���Z���T�̃|�[�g�ԍ� */
#define     ADC_PORT_1  (1)
#define     ADC_PORT_2  (2)
#define     ADC_PORT_3  (3)
#define     ADC_PORT_4  (4)
#define     ADC_PORT_5  (5)
#define     ADC_PORT_6  (6)

/* �ԊO���Z���T�̖���
 * ���L�̓Z���T�̔z�u
 * �i�s�������@LEFT_OUTSIDE | LEFT_INSIDE | CENTER     | RIGHT_INSIDE | RIGHT_INSIDE | RIGHT_OUTSIDE |
 *                                          GOAL_JUDGE
 */
#define LEFT_OUTSIDE	(ADC_PORT_4)	//!< �i�s�������O���̃Z���T
#define LEFT_INSIDE		(ADC_PORT_1)	//!< �i�s���������̃Z���T
#define CENTER			(ADC_PORT_6)	//!< ���S�̃Z���T
#define RIGHT_INSIDE	(ADC_PORT_3)	//!< �i�s�����E���̃Z���T
#define RIGHT_OUTSIDE	(ADC_PORT_5)	//!< �i�s�����E�O���̃Z���T
#define GOAL_JUDGE		(ADC_PORT_2)	//!< �S�[������p�̃Z���T


/* �ԊO���Z���T�̏��(BIT�p�^�[��)��BIT�}�X�N */
#define BIT_000000 (0x0000) //!< 2�i���F000000
#define BIT_000001 (0x0001) //!< 2�i���F000001
#define BIT_000010 (0x0002) //!< 2�i���F000010
#define BIT_000011 (0x0003) //!< 2�i���F000011
#define BIT_000100 (0x0004) //!< 2�i���F000100
#define BIT_000101 (0x0005) //!< 2�i���F000101
#define BIT_000110 (0x0006) //!< 2�i���F000110
#define BIT_000111 (0x0007) //!< 2�i���F000111
#define BIT_001000 (0x0008) //!< 2�i���F001000
#define BIT_001001 (0x0009) //!< 2�i���F001001
#define BIT_001010 (0x000A) //!< 2�i���F001010
#define BIT_001011 (0x000B) //!< 2�i���F001011
#define BIT_001100 (0x000C) //!< 2�i���F001100
#define BIT_001101 (0x000D) //!< 2�i���F001101
#define BIT_001110 (0x000E) //!< 2�i���F001110
#define BIT_001111 (0x000F) //!< 2�i���F001111
#define BIT_010000 (0x0010) //!< 2�i���F010000
#define BIT_010001 (0x0011) //!< 2�i���F010001
#define BIT_010010 (0x0012) //!< 2�i���F010010
#define BIT_010011 (0x0013) //!< 2�i���F010011
#define BIT_010100 (0x0014) //!< 2�i���F010100
#define BIT_010101 (0x0015) //!< 2�i���F010101
#define BIT_010110 (0x0016) //!< 2�i���F010110
#define BIT_010111 (0x0017) //!< 2�i���F010111
#define BIT_011000 (0x0018) //!< 2�i���F011000
#define BIT_011001 (0x0019) //!< 2�i���F011001
#define BIT_011010 (0x001A) //!< 2�i���F011010
#define BIT_011011 (0x001B) //!< 2�i���F011011
#define BIT_011100 (0x001C) //!< 2�i���F011100
#define BIT_011101 (0x001D) //!< 2�i���F011101
#define BIT_011110 (0x001E) //!< 2�i���F011110
#define BIT_011111 (0x001F) //!< 2�i���F011111
#define BIT_100000 (0x0020) //!< 2�i���F100000
#define BIT_100001 (0x0021) //!< 2�i���F100001
#define BIT_100010 (0x0022) //!< 2�i���F100010
#define BIT_100011 (0x0023) //!< 2�i���F100011
#define BIT_100100 (0x0024) //!< 2�i���F100100
#define BIT_100101 (0x0025) //!< 2�i���F100101
#define BIT_100110 (0x0026) //!< 2�i���F100110
#define BIT_100111 (0x0027) //!< 2�i���F100111
#define BIT_101000 (0x0028) //!< 2�i���F101000
#define BIT_101001 (0x0029) //!< 2�i���F101001
#define BIT_101010 (0x002A) //!< 2�i���F101010
#define BIT_101011 (0x002B) //!< 2�i���F101011
#define BIT_101100 (0x002C) //!< 2�i���F101100
#define BIT_101101 (0x002D) //!< 2�i���F101101
#define BIT_101110 (0x002E) //!< 2�i���F101110
#define BIT_101111 (0x002F) //!< 2�i���F101111
#define BIT_110000 (0x0030) //!< 2�i���F110000
#define BIT_110001 (0x0031) //!< 2�i���F110001
#define BIT_110010 (0x0032) //!< 2�i���F110010
#define BIT_110011 (0x0033) //!< 2�i���F110011
#define BIT_110100 (0x0034) //!< 2�i���F110100
#define BIT_110101 (0x0035) //!< 2�i���F110101
#define BIT_110110 (0x0036) //!< 2�i���F110110
#define BIT_110111 (0x0037) //!< 2�i���F110111
#define BIT_111000 (0x0038) //!< 2�i���F111000
#define BIT_111001 (0x0039) //!< 2�i���F111001
#define BIT_111010 (0x003A) //!< 2�i���F111010
#define BIT_111011 (0x003B) //!< 2�i���F111011
#define BIT_111100 (0x003C) //!< 2�i���F111100
#define BIT_111101 (0x003D) //!< 2�i���F111101
#define BIT_111110 (0x003E) //!< 2�i���F111110
#define BIT_111111 (0x003F) //!< 2�i���F111111

/* �ԊO���Z���T�̏��(BIT�p�^�[��)�̃t���O */
#define BIT_GOAL_JUDGE_ON		BIT_000001	 //!< 2�i���F000001
#define BIT_RIGHT_OUTSIDE_ON	BIT_000010	 //!< 2�i���F000010
#define BIT_RIGHT_INSIDE_ON		BIT_000100	 //!< 2�i���F000100
#define BIT_CENTER_ON			BIT_001000	 //!< 2�i���F001000
#define BIT_LEFT_INSIDE_ON		BIT_010000	 //!< 2�i���F010000
#define BIT_LEFT_OUTSIDE_ON		BIT_100000	 //!< 2�i���F100000

//#define COMPARE_VALUE 450//450
#define COMPARE_VALUE 300//450

unsigned int values;

void initIRSensor(void);
unsigned int ReadIRSensor(unsigned int ch);
void ReadIRSensors(unsigned int * sensors);

#endif /* SENSORMANAGER_H_ */
