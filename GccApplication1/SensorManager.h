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

/* 赤外線センサのポート番号 */
#define     ADC_PORT_1  (1)
#define     ADC_PORT_2  (2)
#define     ADC_PORT_3  (3)
#define     ADC_PORT_4  (4)
#define     ADC_PORT_5  (5)
#define     ADC_PORT_6  (6)

/* 赤外線センサの役割
 * 下記はセンサの配置
 * 進行方向↑　LEFT_OUTSIDE | LEFT_INSIDE | CENTER     | RIGHT_INSIDE | RIGHT_INSIDE | RIGHT_OUTSIDE |
 *                                          GOAL_JUDGE
 */
#define LEFT_OUTSIDE	(ADC_PORT_4)	//!< 進行方向左外側のセンサ
#define LEFT_INSIDE		(ADC_PORT_1)	//!< 進行方向左側のセンサ
#define CENTER			(ADC_PORT_6)	//!< 中心のセンサ
#define RIGHT_INSIDE	(ADC_PORT_3)	//!< 進行方向右側のセンサ
#define RIGHT_OUTSIDE	(ADC_PORT_5)	//!< 進行方向右外側のセンサ
#define GOAL_JUDGE		(ADC_PORT_2)	//!< ゴール判定用のセンサ


/* 赤外線センサの状態(BITパターン)のBITマスク */
#define BIT_000000 (0x0000) //!< 2進数：000000
#define BIT_000001 (0x0001) //!< 2進数：000001
#define BIT_000010 (0x0002) //!< 2進数：000010
#define BIT_000011 (0x0003) //!< 2進数：000011
#define BIT_000100 (0x0004) //!< 2進数：000100
#define BIT_000101 (0x0005) //!< 2進数：000101
#define BIT_000110 (0x0006) //!< 2進数：000110
#define BIT_000111 (0x0007) //!< 2進数：000111
#define BIT_001000 (0x0008) //!< 2進数：001000
#define BIT_001001 (0x0009) //!< 2進数：001001
#define BIT_001010 (0x000A) //!< 2進数：001010
#define BIT_001011 (0x000B) //!< 2進数：001011
#define BIT_001100 (0x000C) //!< 2進数：001100
#define BIT_001101 (0x000D) //!< 2進数：001101
#define BIT_001110 (0x000E) //!< 2進数：001110
#define BIT_001111 (0x000F) //!< 2進数：001111
#define BIT_010000 (0x0010) //!< 2進数：010000
#define BIT_010001 (0x0011) //!< 2進数：010001
#define BIT_010010 (0x0012) //!< 2進数：010010
#define BIT_010011 (0x0013) //!< 2進数：010011
#define BIT_010100 (0x0014) //!< 2進数：010100
#define BIT_010101 (0x0015) //!< 2進数：010101
#define BIT_010110 (0x0016) //!< 2進数：010110
#define BIT_010111 (0x0017) //!< 2進数：010111
#define BIT_011000 (0x0018) //!< 2進数：011000
#define BIT_011001 (0x0019) //!< 2進数：011001
#define BIT_011010 (0x001A) //!< 2進数：011010
#define BIT_011011 (0x001B) //!< 2進数：011011
#define BIT_011100 (0x001C) //!< 2進数：011100
#define BIT_011101 (0x001D) //!< 2進数：011101
#define BIT_011110 (0x001E) //!< 2進数：011110
#define BIT_011111 (0x001F) //!< 2進数：011111
#define BIT_100000 (0x0020) //!< 2進数：100000
#define BIT_100001 (0x0021) //!< 2進数：100001
#define BIT_100010 (0x0022) //!< 2進数：100010
#define BIT_100011 (0x0023) //!< 2進数：100011
#define BIT_100100 (0x0024) //!< 2進数：100100
#define BIT_100101 (0x0025) //!< 2進数：100101
#define BIT_100110 (0x0026) //!< 2進数：100110
#define BIT_100111 (0x0027) //!< 2進数：100111
#define BIT_101000 (0x0028) //!< 2進数：101000
#define BIT_101001 (0x0029) //!< 2進数：101001
#define BIT_101010 (0x002A) //!< 2進数：101010
#define BIT_101011 (0x002B) //!< 2進数：101011
#define BIT_101100 (0x002C) //!< 2進数：101100
#define BIT_101101 (0x002D) //!< 2進数：101101
#define BIT_101110 (0x002E) //!< 2進数：101110
#define BIT_101111 (0x002F) //!< 2進数：101111
#define BIT_110000 (0x0030) //!< 2進数：110000
#define BIT_110001 (0x0031) //!< 2進数：110001
#define BIT_110010 (0x0032) //!< 2進数：110010
#define BIT_110011 (0x0033) //!< 2進数：110011
#define BIT_110100 (0x0034) //!< 2進数：110100
#define BIT_110101 (0x0035) //!< 2進数：110101
#define BIT_110110 (0x0036) //!< 2進数：110110
#define BIT_110111 (0x0037) //!< 2進数：110111
#define BIT_111000 (0x0038) //!< 2進数：111000
#define BIT_111001 (0x0039) //!< 2進数：111001
#define BIT_111010 (0x003A) //!< 2進数：111010
#define BIT_111011 (0x003B) //!< 2進数：111011
#define BIT_111100 (0x003C) //!< 2進数：111100
#define BIT_111101 (0x003D) //!< 2進数：111101
#define BIT_111110 (0x003E) //!< 2進数：111110
#define BIT_111111 (0x003F) //!< 2進数：111111

/* 赤外線センサの状態(BITパターン)のフラグ */
#define BIT_GOAL_JUDGE_ON		BIT_000001	 //!< 2進数：000001
#define BIT_RIGHT_OUTSIDE_ON	BIT_000010	 //!< 2進数：000010
#define BIT_RIGHT_INSIDE_ON		BIT_000100	 //!< 2進数：000100
#define BIT_CENTER_ON			BIT_001000	 //!< 2進数：001000
#define BIT_LEFT_INSIDE_ON		BIT_010000	 //!< 2進数：010000
#define BIT_LEFT_OUTSIDE_ON		BIT_100000	 //!< 2進数：100000

//#define COMPARE_VALUE 450//450
#define COMPARE_VALUE 300//450
#define COMPARE_VALUE_OTHER 500//ライントレース用の追加した1個のセンサー
#define COMPARE_VALUE_GOAL 700//ゴール判定用のセンサー用

// ゴールセンサの検知最大数
#define GOAL_DETECTED_MAX_COUNT 10

unsigned int values;

void initIRSensor(void);
unsigned int ReadIRSensor(unsigned int ch);
void ReadIRSensors(unsigned int * sensors);

#endif /* SENSORMANAGER_H_ */
