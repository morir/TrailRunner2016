//##########################################################
//##                      R O B O T I S                   ##
//## CM-700 (Atmega2561) Example code for Dynamixel.      ##
//##                                           2009.11.10 ##
//##########################################################

#define F_CPU   16000000L   //16MHz
#include <stdio.h>
#include <math.h>
#include <util/delay.h>
#include "SerialManager.h"
#include "include/dynamixel.h"
#include "SensorManager.h"
#include "MotorManager.h"
#include "DebugLog.h"

#include "pid.h"

// ------------------ Defined ------------------
// Line Sensor
#define LINE_STATE_BLACK    0//センサー値でラインが白判定
#define LINE_STATE_WHITE    1//センサー値でラインが黒判定

#define _LED_ON_
//#define _MODE_SKIP_			// ショートカットモード

#define DELAY_MAX_TIME      (100)//delay時間の最大値(ミリ秒)
#define STOP_JUDGE_MAX_LIMIT	(10)//停止判定の上限値
#define SLOW_TURN_RATE_BY_BASE	(20)//ベースの20%の速さ

// ------------------ Method Definition ------------------
void executeTraceProcess(void);
int getSensorPattern(void);
void initPETbottlesMotor(void);
void placePETbottles(void);
void stopMoveLessThanVal(int val);

int decideMoveAction(void);
int getAction(void);

void getSensors(void);

void updateAction(int currentAction, int nextAction);

int executeLeftTurn(void);
int executeRightTurn(void);

int initLeftTurnAction(int maxVal);
int initRightTurnAction(int maxVal);

void executeFinalAction(void);

void initEmergencyStop(void);
void executeSkipAction(void);

void setLED(void);
void LED_on(int i);
void LED_off(int i);

// ------------------ Global Variables Definition ------------------

// Serial Message Buffer
int serCmd[SERIAL_BUFFER_SIZE] = {0};

// Goal Judgment counter
int goalCounter = 0;

// Move State
int mCurrentAction = MOVE_SELECTION_TYPE_STRAIGHT;

// Next Move State
int mBeforeMoveState = MOVE_SELECTION_TYPE_STRAIGHT;

// IR Sensor 
unsigned int IR[ADC_PORT_6 + 1] = {0,0,0,0,0,0,0};

// IRの状態(BITパターン)
int IR_BitPattern = 0;

int mMoveCount = 1;


// PID Param
float pGain = 200;   //Proportional Gain
float iGain =  0.2;  //Integral Gain
float dGain =  120;  //Differential Gain
int delay = 10;
int32_t eInteg = 0;  //Integral accumulator
int32_t ePrev  =0;   //Previous Error

int PID_ctlr = 0;	//!< PID制御用変数。中心のセンサからの距離を入力することで、直進時のブレを抑制する制御を行う。

// ------------------ Method ------------------

// ------------------ Table ------------------
int StraightTable[] = {
	/* 00:BIT_00000x */	TRACE_STRAIGHT,
	/* 01:BIT_00001x */	TRACE_R_ROUND_TIGHT,
	/* 02:BIT_00010x */	TRACE_R_ROUND_SOFT,
	/* 03:BIT_00011x */	TRACE_R_TURN,
	/* 04:BIT_00100x */	TRACE_STRAIGHT,
	/* 05:BIT_00101x */	TRACE_R_TURN,
	/* 06:BIT_00110x */	TRACE_R_STRAIGHT,
	/* 07:BIT_00111x */	TRACE_R_TURN,
	/* 08:BIT_01000x */	TRACE_L_ROUND_SOFT,
	/* 09:BIT_01001x */	TRACE_STRAIGHT,
	/* 10:BIT_01010x */	TRACE_STRAIGHT,
	/* 11:BIT_01011x */	TRACE_R_TURN,
	/* 12:BIT_01100x */	TRACE_L_STRAIGHT,
	/* 13:BIT_01101x */	TRACE_R_TURN,
	/* 14:BIT_01110x */	TRACE_STRAIGHT,
	/* 15:BIT_01111x */	TRACE_R_TURN,
	/* 16:BIT_10000x */	TRACE_L_ROUND_TIGHT,
	/* 17:BIT_10001x */	TRACE_STRAIGHT,
	/* 18:BIT_10010x */	TRACE_STRAIGHT,
	/* 19:BIT_10011x */	TRACE_STRAIGHT,
	/* 20:BIT_10100x */	TRACE_L_TURN,
	/* 21:BIT_10101x */	TRACE_STRAIGHT,
	/* 22:BIT_10110x */	TRACE_L_TURN,
	/* 23:BIT_10111x */	TRACE_STRAIGHT,
	/* 24:BIT_11000x */	TRACE_L_TURN,
	/* 25:BIT_11001x */	TRACE_STRAIGHT,
	/* 26:BIT_11010x */	TRACE_L_TURN,
	/* 27:BIT_11011x */	TRACE_STRAIGHT,
	/* 28:BIT_11100x */	TRACE_L_TURN,
	/* 29:BIT_11101x */	TRACE_STRAIGHT,
	/* 30:BIT_11110x */	TRACE_L_TURN,
	/* 31:BIT_11111x */	TRACE_STRAIGHT
};

int LeftStraightTable[] = {
	/* 00:BIT_00000x */	TRACE_L_STRAIGHT,
	/* 01:BIT_00001x */	TRACE_R_ROUND_TIGHT,
	/* 02:BIT_00010x */	TRACE_R_ROUND_MIDDLE,
	/* 03:BIT_00011x */	TRACE_R_TURN,
	/* 04:BIT_00100x */	TRACE_STRAIGHT,
	/* 05:BIT_00101x */	TRACE_R_TURN,
	/* 06:BIT_00110x */	TRACE_R_STRAIGHT,
	/* 07:BIT_00111x */	TRACE_R_TURN,
	/* 08:BIT_01000x */	TRACE_L_ROUND_MIDDLE,
	/* 09:BIT_01001x */	TRACE_L_STRAIGHT,
	/* 10:BIT_01010x */	TRACE_L_STRAIGHT,
	/* 11:BIT_01011x */	TRACE_R_TURN,
	/* 12:BIT_01100x */	TRACE_R_STRAIGHT,
	/* 13:BIT_01101x */	TRACE_R_TURN,
	/* 14:BIT_01110x */	TRACE_L_STRAIGHT,
	/* 15:BIT_01111x */	TRACE_R_TURN,
	/* 16:BIT_10000x */	TRACE_L_ROUND_TIGHT,
	/* 17:BIT_10001x */	TRACE_L_STRAIGHT,
	/* 18:BIT_10010x */	TRACE_L_STRAIGHT,
	/* 19:BIT_10011x */	TRACE_L_STRAIGHT,
	/* 20:BIT_10100x */	TRACE_L_TURN,
	/* 21:BIT_10101x */	TRACE_L_STRAIGHT,
	/* 22:BIT_10110x */	TRACE_L_TURN,
	/* 23:BIT_10111x */	TRACE_L_STRAIGHT,
	/* 24:BIT_11000x */	TRACE_L_TURN,
	/* 25:BIT_11001x */	TRACE_L_STRAIGHT,
	/* 26:BIT_11010x */	TRACE_L_TURN,
	/* 27:BIT_11011x */	TRACE_L_STRAIGHT,
	/* 28:BIT_11100x */	TRACE_L_TURN,
	/* 29:BIT_11101x */	TRACE_L_STRAIGHT,
	/* 30:BIT_11110x */	TRACE_L_TURN,
	/* 31:BIT_11111x */	TRACE_STRAIGHT,
};

int LeftRoundTable[] = {
	/* 00:BIT_00000x */	TRACE_L_ROUND_MIDDLE,
	/* 01:BIT_00001x */	TRACE_R_ROUND_TIGHT,
	/* 02:BIT_00010x */	TRACE_R_STRAIGHT,
	/* 03:BIT_00011x */	TRACE_R_TURN,
	/* 04:BIT_00100x */	TRACE_STRAIGHT,
	/* 05:BIT_00101x */	TRACE_R_TURN,
	/* 06:BIT_00110x */	TRACE_L_ROUND_MIDDLE,
	/* 07:BIT_00111x */	TRACE_R_TURN,
	/* 08:BIT_01000x */	TRACE_L_STRAIGHT,
	/* 09:BIT_01001x */	TRACE_L_ROUND_MIDDLE,
	/* 10:BIT_01010x */	TRACE_L_ROUND_MIDDLE,
	/* 11:BIT_01011x */	TRACE_R_TURN,
	/* 12:BIT_01100x */	TRACE_L_ROUND_SOFT,
	/* 13:BIT_01101x */	TRACE_R_TURN,
	/* 14:BIT_01110x */	TRACE_L_ROUND_MIDDLE,
	/* 15:BIT_01111x */	TRACE_R_TURN,
	/* 16:BIT_10000x */	TRACE_L_ROUND_TIGHT,
	/* 17:BIT_10001x */	TRACE_L_ROUND_MIDDLE,
	/* 18:BIT_10010x */	TRACE_L_ROUND_MIDDLE,
	/* 19:BIT_10011x */	TRACE_L_ROUND_MIDDLE,
	/* 20:BIT_10100x */	TRACE_L_TURN,
	/* 21:BIT_10101x */	TRACE_L_ROUND_MIDDLE,
	/* 22:BIT_10110x */	TRACE_L_TURN,
	/* 23:BIT_10111x */	TRACE_L_ROUND_MIDDLE,
	/* 24:BIT_11000x */	TRACE_L_TURN,
	/* 25:BIT_11001x */	TRACE_L_ROUND_MIDDLE,
	/* 26:BIT_11010x */	TRACE_L_TURN,
	/* 27:BIT_11011x */	TRACE_L_ROUND_MIDDLE,
	/* 28:BIT_11100x */	TRACE_L_TURN,
	/* 29:BIT_11101x */	TRACE_L_ROUND_MIDDLE,
	/* 30:BIT_11110x */	TRACE_L_TURN,
	/* 31:BIT_11111x */	TRACE_STRAIGHT,
};

int LeftTurnTable[] = {
	/* 00:BIT_00000x */	TRACE_L_TURN,
	/* 01:BIT_00001x */	TRACE_L_TURN,
	/* 02:BIT_00010x */	TRACE_L_TURN,
	/* 03:BIT_00011x */	TRACE_L_TURN,
	/* 04:BIT_00100x */	TRACE_STRAIGHT,
	/* 05:BIT_00101x */	TRACE_L_TURN,
	/* 06:BIT_00110x */	TRACE_L_TURN,
	/* 07:BIT_00111x */	TRACE_L_TURN,
	/* 08:BIT_01000x */	TRACE_L_TURN_END,
	/* 09:BIT_01001x */	TRACE_L_TURN,
	/* 10:BIT_01010x */	TRACE_L_TURN,
	/* 11:BIT_01011x */	TRACE_L_TURN,
	/* 12:BIT_01100x */	TRACE_L_TURN,
	/* 13:BIT_01101x */	TRACE_L_TURN,
	/* 14:BIT_01110x */	TRACE_L_TURN,
	/* 15:BIT_01111x */	TRACE_L_TURN,
	/* 16:BIT_10000x */	TRACE_L_TURN,
	/* 17:BIT_10001x */	TRACE_L_TURN,
	/* 18:BIT_10010x */	TRACE_L_TURN,
	/* 19:BIT_10011x */	TRACE_L_TURN,
	/* 20:BIT_10100x */	TRACE_L_TURN,
	/* 21:BIT_10101x */	TRACE_L_TURN,
	/* 22:BIT_10110x */	TRACE_L_TURN,
	/* 23:BIT_10111x */	TRACE_L_TURN,
	/* 24:BIT_11000x */	TRACE_L_TURN,
	/* 25:BIT_11001x */	TRACE_L_TURN,
	/* 26:BIT_11010x */	TRACE_L_TURN,
	/* 27:BIT_11011x */	TRACE_L_TURN,
	/* 28:BIT_11100x */	TRACE_L_TURN,
	/* 29:BIT_11101x */	TRACE_L_TURN,
	/* 30:BIT_11110x */	TRACE_L_TURN,
	/* 31:BIT_11111x */	TRACE_STRAIGHT,
};

int RightStraightTable[] = {
	/* 00:BIT_00000x */	TRACE_R_STRAIGHT,
	/* 01:BIT_00001x */	TRACE_R_ROUND_TIGHT,
	/* 02:BIT_00010x */	TRACE_R_ROUND_MIDDLE,
	/* 03:BIT_00011x */	TRACE_R_TURN,
	/* 04:BIT_00100x */	TRACE_STRAIGHT,
	/* 05:BIT_00101x */	TRACE_R_TURN,
	/* 06:BIT_00110x */	TRACE_L_STRAIGHT,
	/* 07:BIT_00111x */	TRACE_R_TURN,
	/* 08:BIT_01000x */	TRACE_L_ROUND_MIDDLE,
	/* 09:BIT_01001x */	TRACE_R_STRAIGHT,
	/* 10:BIT_01010x */	TRACE_R_STRAIGHT,
	/* 11:BIT_01011x */	TRACE_R_TURN,
	/* 12:BIT_01100x */	TRACE_R_STRAIGHT,
	/* 13:BIT_01101x */	TRACE_R_TURN,
	/* 14:BIT_01110x */	TRACE_R_STRAIGHT,
	/* 15:BIT_01111x */	TRACE_R_TURN,
	/* 16:BIT_10000x */	TRACE_L_ROUND_TIGHT,
	/* 17:BIT_10001x */	TRACE_R_STRAIGHT,
	/* 18:BIT_10010x */	TRACE_R_STRAIGHT,
	/* 19:BIT_10011x */	TRACE_R_STRAIGHT,
	/* 20:BIT_10100x */	TRACE_L_TURN,
	/* 21:BIT_10101x */	TRACE_R_STRAIGHT,
	/* 22:BIT_10110x */	TRACE_L_TURN,
	/* 23:BIT_10111x */	TRACE_R_STRAIGHT,
	/* 24:BIT_11000x */	TRACE_L_TURN,
	/* 25:BIT_11001x */	TRACE_R_STRAIGHT,
	/* 26:BIT_11010x */	TRACE_L_TURN,
	/* 27:BIT_11011x */	TRACE_R_STRAIGHT,
	/* 28:BIT_11100x */	TRACE_L_TURN,
	/* 29:BIT_11101x */	TRACE_R_STRAIGHT,
	/* 30:BIT_11110x */	TRACE_L_TURN,
	/* 31:BIT_11111x */	TRACE_STRAIGHT,
};

int RightRoundTable[] = {
	/* 00:BIT_00000x */	TRACE_R_ROUND_MIDDLE,
	/* 01:BIT_00001x */	TRACE_R_ROUND_TIGHT,
	/* 02:BIT_00010x */	TRACE_R_STRAIGHT,
	/* 03:BIT_00011x */	TRACE_R_TURN,
	/* 04:BIT_00100x */	TRACE_STRAIGHT,
	/* 05:BIT_00101x */	TRACE_R_TURN,
	/* 06:BIT_00110x */	TRACE_R_ROUND_SOFT,
	/* 07:BIT_00111x */	TRACE_R_TURN,
	/* 08:BIT_01000x */	TRACE_L_STRAIGHT,
	/* 09:BIT_01001x */	TRACE_R_ROUND_MIDDLE,
	/* 10:BIT_01010x */	TRACE_R_ROUND_MIDDLE,
	/* 11:BIT_01011x */	TRACE_R_TURN,
	/* 12:BIT_01100x */	TRACE_R_ROUND_MIDDLE,
	/* 13:BIT_01101x */	TRACE_R_TURN,
	/* 14:BIT_01110x */	TRACE_R_ROUND_MIDDLE,
	/* 15:BIT_01111x */	TRACE_R_TURN,
	/* 16:BIT_10000x */	TRACE_L_ROUND_TIGHT,
	/* 17:BIT_10001x */	TRACE_R_ROUND_MIDDLE,
	/* 18:BIT_10010x */	TRACE_R_ROUND_MIDDLE,
	/* 19:BIT_10011x */	TRACE_R_ROUND_MIDDLE,
	/* 20:BIT_10100x */	TRACE_R_ROUND_MIDDLE,
	/* 21:BIT_10101x */	TRACE_R_ROUND_MIDDLE,
	/* 22:BIT_10110x */	TRACE_R_ROUND_MIDDLE,
	/* 23:BIT_10111x */	TRACE_R_ROUND_MIDDLE,
	/* 24:BIT_11000x */	TRACE_L_TURN,
	/* 25:BIT_11001x */	TRACE_R_ROUND_MIDDLE,
	/* 26:BIT_11010x */	TRACE_R_ROUND_MIDDLE,
	/* 27:BIT_11011x */	TRACE_R_ROUND_MIDDLE,
	/* 28:BIT_11100x */	TRACE_L_TURN,
	/* 29:BIT_11101x */	TRACE_R_ROUND_MIDDLE,
	/* 30:BIT_11110x */	TRACE_L_TURN,
	/* 31:BIT_11111x */	TRACE_STRAIGHT,
};

int RightTurnTable[] = {
	/* 00:BIT_00000x */	TRACE_R_TURN,
	/* 01:BIT_00001x */	TRACE_R_TURN,
	/* 02:BIT_00010x */	TRACE_R_TURN_END,
	/* 03:BIT_00011x */	TRACE_R_TURN,
	/* 04:BIT_00100x */	TRACE_STRAIGHT,
	/* 05:BIT_00101x */	TRACE_R_TURN,
	/* 06:BIT_00110x */	TRACE_R_TURN,
	/* 07:BIT_00111x */	TRACE_R_TURN,
	/* 08:BIT_01000x */	TRACE_R_TURN,
	/* 09:BIT_01001x */	TRACE_R_TURN,
	/* 10:BIT_01010x */	TRACE_R_TURN,
	/* 11:BIT_01011x */	TRACE_R_TURN,
	/* 12:BIT_01100x */	TRACE_R_TURN,
	/* 13:BIT_01101x */	TRACE_R_TURN,
	/* 14:BIT_01110x */	TRACE_R_TURN,
	/* 15:BIT_01111x */	TRACE_R_TURN,
	/* 16:BIT_10000x */	TRACE_R_TURN,
	/* 17:BIT_10001x */	TRACE_R_TURN,
	/* 18:BIT_10010x */	TRACE_R_TURN,
	/* 19:BIT_10011x */	TRACE_R_TURN,
	/* 20:BIT_10100x */	TRACE_R_TURN,
	/* 21:BIT_10101x */	TRACE_R_TURN,
	/* 22:BIT_10110x */	TRACE_R_TURN,
	/* 23:BIT_10111x */	TRACE_R_TURN,
	/* 24:BIT_11000x */	TRACE_R_TURN,
	/* 25:BIT_11001x */	TRACE_R_TURN,
	/* 26:BIT_11010x */	TRACE_R_TURN,
	/* 27:BIT_11011x */	TRACE_R_TURN,
	/* 28:BIT_11100x */	TRACE_R_TURN,
	/* 29:BIT_11101x */	TRACE_R_TURN,
	/* 30:BIT_11110x */	TRACE_R_TURN,
	/* 31:BIT_11111x */	TRACE_STRAIGHT
};

int *MatrixTable[] = {
	StraightTable,
	LeftStraightTable,
	LeftRoundTable,
	RightStraightTable,
	RightRoundTable,
	LeftTurnTable,
	RightTurnTable,
	LeftRoundTable,
	LeftRoundTable,
	RightRoundTable,
	RightRoundTable
};

/**
* エントリーポイント
* @brief エントリーポイント
* @return 0：メイン処理の継続
* @return 1：メイン処理の終了
*/
int main(void) {
    
    initEmergencyStop();
    setLED();
    initIRSensor();
    MotorInit();
    initSerial();
	initPETbottlesMotor();

	// ロボ動作開始

    // ショートカットモードを作る場合はここに入れる。
#ifdef _MODE_SKIP_
	executeSkipAction();
#endif /* _MODE_SKIP_ */

	// トレース動作開始
	executeTraceProcess();

    // ゴール判定後の動作実質ここから開始？
	executeFinalAction();
}

/**
* ライントレース動作
* @brief ライントレース動作
* @return なし
* @detail ゴール判定条件を満たすまでライントレース動作を行う。
*/
void executeTraceProcess(void) {
	static int previousTraceAction = TRACE_STRAIGHT;
	static int currentTraceAction = TRACE_STRAIGHT;
	static int sensorPattern = BIT_000000;
	int waitMaxCount = 1;

	//初期動作（少しだけ直進）
	StraightMove();
	_delay_ms(100);	// 10ms 間隔を空ける

	while (1) {

		// センサ値のビットパターンを取得する。
		sensorPattern = getSensorPattern();

		// センサ値のパターンが最終動作であればループを抜ける。
		if (sensorPattern == TRACE_FINALACTION) {
			break;
		}

		// 前回の動作とセンサ値のパターンの組み合わせから今回の動作を決定する。
		currentTraceAction = MatrixTable[previousTraceAction][(sensorPattern / 2)];
//		LOG_INFO("(sensorPattern / 2) %3d\r\n", (sensorPattern / 2));
//		LOG_INFO("previousTraceAction %3d: sensorPattern %3d: currentTraceAction: %3d \r\n",
//		         previousTraceAction, sensorPattern, currentTraceAction);
//		if(currentTraceAction != previousTraceAction)
//		{
			// LEDを設定
			setLED();

			if(currentTraceAction == TRACE_L_TURN)
			{
				//旋回実行
				currentTraceAction = executeLeftTurn();

			}
			else if (currentTraceAction == TRACE_R_TURN)
			{
				//旋回実行
				currentTraceAction = executeRightTurn();

			}

			Execute(currentTraceAction);
/* お試し miyano ここから */
/* 試して意味なしだったら削除します。
			//ステータスの変化間隔が短い場合、delayを大きくして曲げを大きくする
			//delayの最大値は走行させて検証必要！！！
			waitMaxCount = DELAY_MAX_TIME/mMoveCount;
			if (waitMaxCount == 0) {
				waitMaxCount = 1;
			}

			//_delay_ms(1)を繰り返して、待ち時間を可変に確保する
			int waitCount = 0;
			while(1) {
				_delay_ms(1);// 1msのdelayTimeの間隔を空ける
				waitCount++;
				if (waitCount >= waitMaxCount) {
					//カウント数に達したら_delay_ms(1)を止める。
					break;
				}
			}
			
			mMoveCount = 1;
		} else {
			//ステータスが変化するまでの回数をカウント
			mMoveCount++;
*/
/* お試し miyano ここまで */
//		}

		_delay_ms(1);// delayTimeの間隔を空ける

		// 今回の動作を前回の動作に退避する。
		previousTraceAction = currentTraceAction;
	}
}

/**
* センサー値のBitパターンを取得する。
* @brief センサー値を参照し、対応するアクションを取得する。
* @return 戻り値の説明
*/
int getSensorPattern(void) {
    int ptn = 0;
	
	// LEDを設定
	//setLED();
	
	// センサー値を取得
	getSensors();
	
	// 判定条件数を減らすためゴール判定用センサ値をフィルタリングする。
	ptn = ((IR_BitPattern >> 1) << 1);

	// ゴール判定（ゴール用センサを連続で規定数回検知し且つトレース用センサーが黒のとき）
	if (IR[GOAL_JUDGE] >= 700) {
		//goalCounter++;
		if (goalCounter >= 30 && 
			( (IR_BitPattern == BIT_000011 ) ||
			  (IR_BitPattern == BIT_000111 ) ||
			  (IR_BitPattern == BIT_001111 ) ||
			  (IR_BitPattern == BIT_011111 ) ||
			  (IR_BitPattern == BIT_111111 )
			)){
			ptn = TRACE_FINALACTION;
		}
	} else {
		//一度でも白なら判定解除
		goalCounter = 0;
	}

	return ptn;
}

/**
 * センサー値を取得
 * @brief センサー値を取得
 * @return なし
 * @detail センサー値を取得し、IR[]およびIR_BitPatternを更新する。
 */
void getSensors(void) {
	/* センサー値を取得 */
    ReadIRSensors(IR);
	
	/* IR状態をBITパターンに変換 */
	IR_BitPattern = 0;
	if ( IR[GOAL_JUDGE]		>= COMPARE_VALUE_GOAL )	IR_BitPattern |= BIT_GOAL_JUDGE_ON;
	if ( IR[RIGHT_OUTSIDE]	>= COMPARE_VALUE )	IR_BitPattern |= BIT_RIGHT_OUTSIDE_ON;
	if ( IR[RIGHT_INSIDE]	>= COMPARE_VALUE )	IR_BitPattern |= BIT_RIGHT_INSIDE_ON;
	if ( IR[CENTER]			>= COMPARE_VALUE )	IR_BitPattern |= BIT_CENTER_ON;
	if ( IR[LEFT_INSIDE]	>= COMPARE_VALUE )	IR_BitPattern |= BIT_LEFT_INSIDE_ON;
	if ( IR[LEFT_OUTSIDE]	>= COMPARE_VALUE_OTHER )	IR_BitPattern |= BIT_LEFT_OUTSIDE_ON;

    LOG_INFO("sensor %3d: %3d: %3d: %3d: %3d: %3d \r\n",
	       IR[LEFT_OUTSIDE], IR[LEFT_INSIDE], IR[CENTER], IR[RIGHT_INSIDE], IR[RIGHT_OUTSIDE], IR[GOAL_JUDGE]);
	LOG_DEBUG("IR[R %1d%1d%1d%1d%1d L] GOAL[%1d]\r\n",
				((IR[LEFT_OUTSIDE]	>= COMPARE_VALUE)?  1 : 0),
				((IR[LEFT_INSIDE]	>= COMPARE_VALUE)?  1 : 0),
				((IR[CENTER]		>= COMPARE_VALUE)?  1 : 0),
				((IR[RIGHT_INSIDE]	>= COMPARE_VALUE)?  1 : 0),
				((IR[RIGHT_OUTSIDE]	>= COMPARE_VALUE)?  1 : 0),
				((IR[GOAL_JUDGE]	>= COMPARE_VALUE)?  1 : 0));
	
}

/**
* ゴール到達時の処理
* @brief ゴール到達時の処理
* @return なし
*/
void executeFinalAction(void)
{
	LOG_INFO("executeFinalAction!!\r\n");

	MotorControl( RIGHT_MOTOR, 300 );
	MotorControl( LEFT_MOTOR, 300 );
	_delay_ms(900);
	Execute(MOVE_SELECTION_TYPE_STOP);
	_delay_ms(100);

	//ペットボトル設置を実行
	placePETbottles();

	MotorControl( RIGHT_MOTOR, 1623 );
	MotorControl( LEFT_MOTOR, 600 );
	_delay_ms(500);
	Execute(MOVE_SELECTION_TYPE_STOP);
}

/************************************************************************/
// ペットボトル用モータの初期設定
// ペットボトル設置用モーターを少し前方に傾ける。
/************************************************************************/
void initPETbottlesMotor(void) {
	//最大速度で、642の位置へ動かす
	MotorControlJoint( PETBOTTOLE_MOTOR, 0, 642 );
}

/************************************************************************/
// ペットボトル設置
/************************************************************************/
void placePETbottles(void) {
	_delay_ms(1000);//1秒待つ⇒動作に合わせて変更してください
	MotorControlJoint( PETBOTTOLE_MOTOR, 30, 352 );//モーターを後方にゆっくり傾ける
	_delay_ms(6000);//6秒継続
	MotorControlJoint( PETBOTTOLE_MOTOR, 100, 512 );//モーターをセンター位置に戻す
	_delay_ms(3000);//3秒待つ⇒動作に合わせて変更してください

}

/**
 * 速度が0～入力値以下になるまで、停止動作を継続する 
 * @param maxVal 停止判定の上限値
 */
void stopMoveLessThanVal(int maxVal){
	StopMove();//停止を実行
	int judgeSpeed = 0;
	while(1) {
		judgeSpeed = GetCurrentSpeedR();//モーターの速度を取得
		if( (judgeSpeed >= 0 || judgeSpeed <= maxVal) ||
		  (judgeSpeed >= 1024 || judgeSpeed <= (1024 + maxVal)) ) {
			//速度がmaxVal以下ならstop()抜ける
			break;
		}
	}
}

/**
 * 左旋回実行
 * 旋回動作をさせて、センサーが中央になったら直進を指定して抜ける
 */
int executeLeftTurn(void){
	int sensorPattern = BIT_000000;
	LED_on(1);

	//旋回判定されたら停止を実行
	int initResult = initLeftTurnAction(STOP_JUDGE_MAX_LIMIT);
	if (initResult == TRACE_STRAIGHT) {
		return TRACE_STRAIGHT;
	}

	//停止が確定したら旋回動作開始
	LeftTurnMove();
	while(1) {
		sensorPattern = getSensorPattern();
		//旋回動作を抜けるための条件を判定
		if (sensorPattern == BIT_010000 || sensorPattern == BIT_010001 ||
			sensorPattern == BIT_001000 || sensorPattern == BIT_001001 ||
			sensorPattern == BIT_000100 || sensorPattern == BIT_000101
			) {
			LED_on(2);
			//中央のセンサーが黒なら停止を実行
			stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
			break;
		}
	}

	//旋回停止判定後の止まった位置でセンサーが中央なら逆旋回終了
	sensorPattern = getSensorPattern();
	if (sensorPattern == BIT_001000 || sensorPattern == BIT_001001) {
		//中央センサーなので、直進に設定して抜ける
		return TRACE_STRAIGHT;
	}
	
	LED_on(3);
	//センサーを中央に戻すため遅い旋回を実行
	RightTurnSlowMove(SLOW_TURN_RATE_BY_BASE);
	while(1) {
		//逆旋回動作を抜けるための条件を判定
		sensorPattern = getSensorPattern();
		if (sensorPattern == BIT_001000 || sensorPattern == BIT_001001) {
			stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
			return TRACE_STRAIGHT;
		} else if ( sensorPattern == BIT_010000 ||	sensorPattern == BIT_010001 ||
					sensorPattern == BIT_100000 ||	sensorPattern == BIT_100001 ) {
			//既に逆側まで旋回していたら（想定よりも早く解除できてしまった場合など）
			break;
		}
	}

	//再度センサーを中央に戻すため遅い旋回を実行（ここまでは実行されない想定）
	LED_on(4);
	LeftTurnSlowMove(SLOW_TURN_RATE_BY_BASE);
	while(1) {
		//逆旋回動作を抜けるための条件を判定
		sensorPattern = getSensorPattern();
		if (sensorPattern == BIT_001000 || sensorPattern == BIT_001001) {
			stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
			return TRACE_STRAIGHT;
		}
	}
	return TRACE_STRAIGHT;
}

/**
 * 右旋回実行
 * 旋回動作をさせて、センサーが中央になったら直進を指定して抜ける
 */
int executeRightTurn(void){
	int sensorPattern = BIT_000000;
	LED_on(1);

	int initResult = initRightTurnAction(STOP_JUDGE_MAX_LIMIT);
	if (initResult == TRACE_STRAIGHT) {
		return TRACE_STRAIGHT;
	}

	//停止が確定したら旋回動作開始
	RightTurnMove();
	while(1) {
		sensorPattern = getSensorPattern();
		//旋回動作を抜けるための条件を判定
		if (sensorPattern == BIT_000100 || sensorPattern == BIT_000101 ||
		sensorPattern == BIT_001000 || sensorPattern == BIT_001001 ||
		sensorPattern == BIT_010000 || sensorPattern == BIT_010001
		) {
			LED_on(2);
			//中央のセンサーが黒なら停止を実行
			stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
			break;
		}
	}

	//旋回停止判定後の止まった位置でセンサーが中央なら逆旋回終了
	sensorPattern = getSensorPattern();
	if (sensorPattern == BIT_001000 || sensorPattern == BIT_001001) {
		//中央センサーなので、直進に設定して抜ける
		return TRACE_STRAIGHT;
	}
		
	LED_on(3);
	//センサーを中央に戻すため遅い旋回を実行
	LeftTurnSlowMove(SLOW_TURN_RATE_BY_BASE);
	while(1) {
		//逆旋回動作を抜けるための条件を判定
		sensorPattern = getSensorPattern();
		if (sensorPattern == BIT_001000 || sensorPattern == BIT_001001) {
			stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
			return TRACE_STRAIGHT;
		} else if ( sensorPattern == BIT_010000 ||
					sensorPattern == BIT_010001 ||
					sensorPattern == BIT_100000 ||
					sensorPattern == BIT_100001 ) {
			//既に逆側まで旋回していたら（想定よりも早く解除できてしまった場合）
			break;

		}
	}
		
	//再度センサーを中央に戻すため遅い旋回を実行（ここまでは実行されない想定）
	LED_on(4);
	RightTurnSlowMove(SLOW_TURN_RATE_BY_BASE);
	while(1) {
		//逆旋回動作を抜けるための条件を判定
		sensorPattern = getSensorPattern();
		if (sensorPattern == BIT_001000 || sensorPattern == BIT_001001) {
			stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
			return TRACE_STRAIGHT;
		}
	}
	return TRACE_STRAIGHT;
}

/**
 * 左旋回動作の初期化
 * 停止を実行して、途中で全て黒になったら直進モードにする
 * 基準以下の速度まで減速できたら、旋回を継続する
 */
int initLeftTurnAction(int maxVal) {
	int sensorPattern = BIT_000000;

	StopMove();//停止を実行
	int judgeSpeed = 0;
	while(1) {
		sensorPattern = getSensorPattern();//センサー値を取得
		if(sensorPattern == BIT_111110 || sensorPattern == BIT_111111) {
			//旋回判定後の停止中に黒ラインになったら旋回を止めて、直進する
			//旋回を止める条件は、センサー値がBIT_XXXX1Xでも良いかな。。。
			return TRACE_STRAIGHT;
		}

		judgeSpeed = GetCurrentSpeedR();//モーターの速度を取得
		if( (judgeSpeed >= 0 || judgeSpeed <= maxVal) ||
			(judgeSpeed >= 1024 || judgeSpeed <= (1024 + maxVal)) ) {
			//速度がmaxVal以下ならstop()抜ける
			return TRACE_L_TURN;
		}
	}
}

/**
 * 右旋回動作の初期化
 * 停止を実行して、途中で全て黒になったら直進モードにする
 * 基準以下の速度まで減速できたら、旋回を継続する
 */
int initRightTurnAction(int maxVal) {
	int sensorPattern = BIT_000000;

	StopMove();//停止を実行
	int judgeSpeed = 0;
	while(1) {
		sensorPattern = getSensorPattern();//センサー値を取得
		if(sensorPattern == BIT_111110 || sensorPattern == BIT_111111) {
			//旋回判定後の停止中に黒ラインになったら旋回を止めて、直進する
			//旋回を止める条件は、センサー値がBIT_1XXXXXでも良いかな。。。
			return TRACE_STRAIGHT;
		}

		judgeSpeed = GetCurrentSpeedR();//モーターの速度を取得
		if( (judgeSpeed >= 0 || judgeSpeed <= maxVal) ||
			(judgeSpeed >= 1024 || judgeSpeed <= (1024 + maxVal)) ) {
			//速度がmaxVal以下ならstop()抜ける
			return TRACE_R_TURN;
		}
	}
}

/**
* ショートカットの処理
* @brief ショートカットの処理
* @return なし
*/
void executeSkipAction(void) {
	LOG_INFO("***** executeSkipAction START!! *****\r\n");

	static int sensorPattern = BIT_000000;

	// 初期動作（直進）
	StraightMove();
	_delay_ms(100);	// 10ms 間隔を空ける

	while (1) {

		// センサ値のビットパターンを取得する。
		sensorPattern = getSensorPattern();

		// センサ値のパターンが全黒であればループを抜ける。
		if (sensorPattern == BIT_111111 || sensorPattern == BIT_111110) {
			break;
		}
		_delay_ms(5);// delayTimeの間隔を空ける
	}

	_delay_ms(5);// delayTimeの間隔を空ける

	// 左旋回
	LeftTurnMove();
	_delay_ms(500);	// 500ms 間隔を空ける

	while (1) {

		// センサ値のビットパターンを取得する。
		sensorPattern = getSensorPattern();

		// センサ値のパターンが真ん中黒であればループを抜ける。
		if (sensorPattern == BIT_001000 || sensorPattern == BIT_001001) {
			break;
		}
		_delay_ms(5);// delayTimeの間隔を空ける
	}

	//左旋回中復帰時の動作
	RightTurnMove();//逆回転
	_delay_ms(100);	// 100ms 逆回転を入力（強さと時間は調整必要）

	LOG_INFO("***** executeSkipAction END!! *****\r\n");

	// 通常のライントレースに復帰
}

/**
 * アクションを更新
 * @brief アクションを更新
 * @param (int currentAction) 現在のアクション
 * @param (int nextAction)    次のアクション
 * @return なし
 * @detail 現在実行中のアクション、次のアクションを比較し、差分がある場合のみ実行する。
 */
void updateAction(int currentAction, int nextAction) {
	// Next Move Stateを更新
    mBeforeMoveState = currentAction;
	
	// 
    if (currentAction != nextAction) {
        mCurrentAction = nextAction;
    }
}

void initEmergencyStop(void) {
    DDRD  = 0x70;
    PORTD = 0x11;
}

/**
 * LEDを設定
 * @brief LEDを設定
 * @param (引数名) 引数の説明
 * @param (引数名) 引数の説明
 * @return 戻り値の説明
 * @sa 参照すべき関数を書けばリンクが貼れる
 * @detail 詳細な説明
 */
void setLED(void) {
#ifdef _LED_ON_
	//マイコンのレジスタ(DDRC)の設定
    DDRC  = 0x7F;
	//マイコンの出力ポート(PORTC)の設定
    PORTC = 0x7F;
#endif // _LED_ON_
}

/**
 * LED点灯
 * @brief LED点灯
 * @param (int i) LEDの番号
 * @return なし
 */
void LED_on(int i) {
#ifdef _LED_ON_
    if (i < 1 || i > 6) return;
	unsigned char c = PORTC;
	//マイコンの出力ポート(PORTC)の設定
    PORTC = c^(1<<i);
#endif // _LED_ON_
}

/**
 * LED消灯
 * @brief LED消灯
 * @param (int i) LEDの番号
 * @return なし
 */
void LED_off(int i) {
#ifdef _LED_ON_
    if (i < 1 || i > 6) return;
	unsigned char c = PORTC;
	//マイコンの出力ポート(PORTC)の設定
    PORTC = ~(c^(1<<i));
#endif // _LED_ON_
}
