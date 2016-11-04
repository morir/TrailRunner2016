/*
 * MotorManager.h
 *
 * Created: 2014/10/29 14:46:58
 *  Author: Administrator
 */

#ifndef MOTOR_MANAGER_H_
#define MOTOR_MANAGER_H_
 
#include <stdio.h>
#include <math.h>
#include <util/delay.h>
#include "include/dynamixel.h"

// ------------------ Defined ------------------
#define DEFAULT_BAUDNUM     1 // 1Mbps

//#define DINAMIXEL_AX_12       // Dinamixel AX-12 use
#define LOW_SPEED_MODE          // Low speed mode
//#define HIGH_SPEED_MODE         // High speed mode
//#define MAX_SPEED_MODE          // Max speed mode
//#define TEST_SPEED_MODE        //TEST 20150612

// Motor Settings Address
#define P_CW_ANGLE_LIMIT_L  6
#define P_CW_ANGLE_LIMIT_H  7
#define P_CCW_ANGLE_LIMIT_L 8
#define P_CCW_ANGLE_LIMIT_H 9
#define P_GOAL_POSITION_L   30
#define P_GOAL_POSITION_H   31
#define P_GOAL_SPEED_L      32
#define P_GOAL_SPEED_H      33
#define P_EEP_LOCK          47

#ifdef DINAMIXEL_AX_12
#define RIGHT_MOTOR         2       // Right Motor address
#define LEFT_MOTOR          3       // Left Motor address
#define DELAY_MSEC          10      // Delay time
#define OVER_RUN_TIME       1500    // Over run time
#else // DINAMIXEL_AX_12
#define RIGHT_MOTOR         32      // Right Motor address
#define LEFT_MOTOR          33      // Left Motor address
#define CENTER_AXS1_SENSOR	100		// AX-S1 Sensor address
#define DELAY_MSEC          1       // Delay time
#define OVER_RUN_TIME       500     // Over run time
#endif // DINAMIXEL_AX_12

#define PETBOTTOLE_MOTOR	18		// Cover Motor address

// MX-12W Control Table Address
#define CTRL_TBL_ADDR_PRESENT_SPEED_L	(38)	// Lowest byte of Current Speed 
#define CTRL_TBL_ADDR_PRESENT_SPEED_H	(39)	// Highest byte of Current Speed

// AX-S1  Control Table Address
#define AXS1_ADDR_IR_LEFT_FIRE_DATA		(26)	// 左側の赤外線センサ値
#define AXS1_ADDR_IR_CENTER_FIRE_DATA	(27)	// 中央の赤外線センサ値
#define AXS1_ADDR_IR_RIGHT_FIRE_DATA	(28)	// 右側の赤外線センサ値
#define AXS1_ADDR_IR_LIGHT_LEFT_DATA	(29)	// 左側の照度センサ値
#define AXS1_ADDR_IR_LIGHT_CENTER_DATA	(30)	// 中央の照度センサ値
#define AXS1_ADDR_IR_LIGHT_RIGHT_DATA	(31)	// 右側の照度センサ値

// Motor Speed Value
#ifdef DINAMIXEL_AX_12
#define P_CW_SPEED_NOMAL    1023
#define P_CCW_SPEED_NOMAL   2047
#define P_CW_SPEED_TURN     920
#define P_CCW_SPEED_TURN    1943
#define P_CW_SPEED_TURN_2   818
#define P_CCW_SPEED_TURN_2  1841
#else // DINAMIXEL_AX_12
// DINAMIXEL_AX_12W
#if defined (LOW_SPEED_MODE)
// LOW_SPEED_MODE
#define P_CW_SPEED_NOMAL    609
#define P_CCW_SPEED_NOMAL   1632
#define P_CW_SPEED_TURN     509
#define P_CCW_SPEED_TURN    1532
#define P_CW_SPEED_TURN_2   409
#define P_CCW_SPEED_TURN_2  1432
#elif defined (HIGH_SPEED_MODE)
// HIGH_SPEED_MODE
#define P_CW_SPEED_NOMAL    809
#define P_CCW_SPEED_NOMAL   1832
#define P_CW_SPEED_TURN     709
#define P_CCW_SPEED_TURN    1732
#define P_CW_SPEED_TURN_2   609
#define P_CCW_SPEED_TURN_2  1632
#elif defined (TEST_SPEED_MODE)
// TEST_SPEED_MODE
#define P_CW_SPEED_NOMAL    200
#define P_CCW_SPEED_NOMAL   1223
#define P_CW_SPEED_TURN     1203
#define P_CCW_SPEED_TURN    180
#define P_CW_SPEED_TURN_2   1153
#define P_CCW_SPEED_TURN_2  150
#else
// MAX_SPEED_MODE
#define P_CW_SPEED_NOMAL    1023
#define P_CCW_SPEED_NOMAL   2047
#define P_CW_SPEED_TURN     920
#define P_CCW_SPEED_TURN    1943
#define P_CW_SPEED_TURN_2   818
#define P_CCW_SPEED_TURN_2  1841
#endif // LOW_SPEED_MODE
#endif // DINAMIXEL_AX_12

// Trace pattern
#define TRACE_STRAIGHT			0	// 直進
#define TRACE_L_STRAIGHT		1	// 左前進
#define TRACE_L_ROUND_SOFT		7	// 左カーブ(Soft)
#define TRACE_L_ROUND_MIDDLE	2	// 左カーブ(Middle)
#define TRACE_L_ROUND_TIGHT		8	// 左カーブ(Tight)
#define TRACE_L_TURN_START		111	// 左旋回開始
#define TRACE_L_TURN			5	// 左旋回
#define TRACE_L_TURN_END		112	// 左旋回終了
#define TRACE_R_STRAIGHT		3	// 右前進
#define TRACE_R_ROUND_SOFT		9	// 右カーブ(Soft)
#define TRACE_R_ROUND_MIDDLE	4	// 右カーブ(Middle)
#define TRACE_R_ROUND_TIGHT		10	// 右カーブ(Tight)
#define TRACE_R_TURN_START		113	// 右旋回開始
#define TRACE_R_TURN			6	// 右旋回
#define TRACE_R_TURN_END		114	// 右旋回終了
#define TRACE_SLOW_STRAIGHT		11	// 直進
#define TRACE_FINALACTION		999	// ゴール動作

#define MOTOR_MOVE_UP_VAL		(20)	//モーターが低速だった場合この値分を付加して駆動させる
#define TURN_SPEED_BASE			(100)	//旋回のベース速度
#define BASE_SPEED_INIT_VAL		(100)	//ベース速度の初期値
#define TURN_SPEED_JUDGE_VAL	(400)	//定常旋回する基準速度

#define MOTOR_MOVE_UP_VAL	(20)	//モーターが低速だった場合この値分を付加して駆動させる
#define SLOW_STRAIGHT_VAL	(120)	//横切るライン通過時の速度

// ------------------ Method Definition ------------------
void MotorInit(void);
void MotorControl(int id, int power);
void MotorControlJoint(int id, int speed, int position);
void Execute(int type);
void setParamMoveAction(int right, int left);
void StopMove(void);
void StraightMove(void);
void StraightLowMove(void);
void StraightMoveRightShift(void);
void StraightMoveLeftShift(void);
void StraightMoveRightShift2(void);
void StraightMoveLeftShift2(void);
void TurnMoveRight(void);
void TurnMoveLeft(void);
void TurnLowMoveRight(void);
void TurnLowMoveLeft(void);
void BackMove(void);
void BackLowMove(void);
void Move(int leftSpeed, int rightSpeed);
void LeftStraightMove(void);
void RightStraightMove(void);
void LeftSoftRoundMove(void);
void LeftMiddleRoundMove(void);
void LeftTightRoundMove(void);
void RightSoftRoundMove(void);
void RightMiddleRoundMove(void);
void RightTightRoundMove(void);
void LeftTurnMove(void);
void RightTurnMove(void);
void LeftTurnByBaseSpeedAdjust(void);
void RightTurnByBaseSpeedAdjust(void);
void LeftTurnSlowMove(int rate);
void RightTurnSlowMove(int rate);
void PrintErrorCode(void);
void PrintCommStatus(int CommStatus);

int GetCurrentSpeedR(void);
int GetCurrentSpeedL(void);

void GetAXS1SensorFireData(int *out_fire_data_left, int *out_fire_data_center, int *out_fire_data_right);

int mCount;
int BaseSpeed;

#endif // MOTOR_MANAGER_H_