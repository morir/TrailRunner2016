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

#define DELAY_MAX_TIME      (100)//delay時間の最大値(ミリ秒)
#define STOP_JUDGE_MAX_LIMIT	(10)//停止判定の上限値

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

void executeFinalAction(void);

void initEmergencyStop(void);

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
	int judgeSpeed = 0;

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
				LED_on(1);
				stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);

				//旋回実行
				LeftTurnMove();
				while(1) {
					sensorPattern = getSensorPattern();
					//旋回動作を抜けるための条件を判定
					if (sensorPattern == BIT_010000 || sensorPattern == BIT_010001) {
				LED_on(2);
						//中央のセンサーが黒なら停止を実行
						stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);

						//逆旋回を実行：センサーを中央に戻すため
						RightTurnMove();
						while(1) {
							//逆旋回動作を抜けるための条件を判定
							sensorPattern = getSensorPattern();
							if (sensorPattern == BIT_001000 || sensorPattern == BIT_001001) {
				LED_on(3);
								stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
								currentTraceAction = TRACE_STRAIGHT;
								break;
							}
						}
						break;
					} else if (sensorPattern == BIT_111110 || sensorPattern == BIT_111111) {
						currentTraceAction = TRACE_STRAIGHT;
						break;
					}
				}
			}
			else if (currentTraceAction == TRACE_R_TURN)
			{
				LED_on(5);
				stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);

				//旋回実行
				RightTurnMove();
				while(1) {
					sensorPattern = getSensorPattern();
					//旋回動作を抜けるための条件を判定
					if (sensorPattern == BIT_000100 || sensorPattern == BIT_000101) {
						LED_on(4);
						//中央のセンサーが黒なら停止を実行
						stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);

						//逆旋回を実行：センサーを中央に戻すため
						LeftTurnMove();
						while(1) {
							//逆旋回動作を抜けるための条件を判定
							sensorPattern = getSensorPattern();
							if (sensorPattern == BIT_001000 || sensorPattern == BIT_001001) {
								LED_on(3);
								stopMoveLessThanVal(STOP_JUDGE_MAX_LIMIT);
								currentTraceAction = TRACE_STRAIGHT;
								break;
							}
						}
						break;
					} else if (sensorPattern == BIT_111110 || sensorPattern == BIT_111111) {
						currentTraceAction = TRACE_STRAIGHT;
						break;
					}
				}

			}

			//左旋回中復帰時の動作
/* 復帰動作は一旦コメントアウト。動作検証後、変更か削除します。
			if (previousTraceAction == TRACE_L_TURN && currentTraceAction == TRACE_L_TURN_END) {
				//RightTurnMove();//逆回転
				StopMove();
				_delay_ms(1000);	// 100ms 逆回転を入力（強さと時間は調整必要）
				currentTraceAction = TRACE_R_ROUND_MIDDLE;
			}
			
			//右旋回中復帰時の動作
			if (previousTraceAction == TRACE_R_TURN && currentTraceAction == TRACE_R_TURN_END) {
				//LeftTurnMove();//逆回転
				StopMove();
				_delay_ms(1000);	// 100ms 逆回転を入力（強さと時間は調整必要）
				currentTraceAction = TRACE_L_ROUND_MIDDLE;
			}
*/
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
* センサー値から次の行動パターンを決定
* @brief センサー値から次の行動パターンを決定
* @return メインプログラムのステータス
* @detail センサー値から次の行動パターンを決定し、戻り値にメインプログラムのステータスを返す。
*/
int decideMoveAction(void) {
    int ret_state = 0;//メインプログラムのステータス
//	int rightVal = 0;
//	int leftVal = 0;
	
    getSensors();//現在のセンサー値を取得。
    int currentAction = mCurrentAction;//前回の判定を次の動作判定に使用する
	
    int nextAction;

	//前回の判定に応じて分岐する
    switch (currentAction) {
    case MOVE_SELECTION_TYPE_START:
    case MOVE_SELECTION_TYPE_STOP:
        Execute(currentAction);
        if (currentAction == MOVE_SELECTION_TYPE_START) {//このif文無くして、中の処理をcase MOVE_SELECTION_TYPE_STARTにすれば良いのでは？
        	//次は、直進
            nextAction = MOVE_SELECTION_TYPE_STRAIGHT;
        } else {
            nextAction = MOVE_SELECTION_TYPE_START;
            PID_reset_diff_integral();
        }
    	//次の動作判定を、更新(変更)する。
        updateAction(currentAction, nextAction);
        break;

	//中央４個のセンサーのどれか１つ以上黒ならここを使ってPID制御を行う
    case MOVE_SELECTION_TYPE_STRAIGHT:
    case MOVE_SELECTION_TYPE_RIGHTSIFT_1:
    case MOVE_SELECTION_TYPE_RIGHTSIFT_2:
    case MOVE_SELECTION_TYPE_LEFTSIFT_1:
    case MOVE_SELECTION_TYPE_LEFTSIFT_2:
        //PID_ctlr_Update(0, PID_ctlr, &rightVal, &leftVal);//PID制御の制御値を更新
        //setParamMoveAction(rightVal, leftVal);//モーターの駆動指令
		StraightMove();

        nextAction = getAction();//現在のセンサー値を使って、次の動作を決定

    	//次の動作判定を、更新(変更)する。
        updateAction(currentAction, nextAction);
        break;

    //右旋回用の分岐
    //一度、ここに入ると中央のセンサーが黒になるまでループし続ける。
    case MOVE_SELECTION_TYPE_RIGHTTURN:
    case MOVE_SELECTION_TYPE_RIGHTTURN_3:
        LED_on(1);//LED点灯
        Execute(MOVE_SELECTION_TYPE_RIGHTTURN);//右の定常旋回実行
		if (currentAction == MOVE_SELECTION_TYPE_RIGHTTURN) {
			//右旋回が判定されたら、一定時間旋回を継続させる
			//前回の判定も考慮して、ウェイト時間を変更すべきかな・・・・
			_delay_ms(150);//150msのウェイト
		}
    	//ループ
        while(1) {
            nextAction = getAction();//現在のセンサー値で、次の動作を決める
			if (nextAction == MOVE_SELECTION_TYPE_STRAIGHT ||
                nextAction == MOVE_SELECTION_TYPE_RIGHTSIFT_1 ||
                nextAction == MOVE_SELECTION_TYPE_RIGHTSIFT_2 ||
                nextAction == MOVE_SELECTION_TYPE_LEFTSIFT_1 ||
                nextAction == MOVE_SELECTION_TYPE_LEFTSIFT_2)
			{
				//中央４つのセンサーのいずれかが黒なら、以降の処理
				PID_reset_diff_integral();
				Execute(MOVE_SELECTION_TYPE_LEFTTURN);//逆回転を実行して、旋回動作のブレーキ。
				if (currentAction == MOVE_SELECTION_TYPE_RIGHTTURN) {
					//旋回中から、ここに入ったらブレーキ時間を長い時間にする
					//直角旋回からの復帰時は、ここに入るはず。
					_delay_ms(100);
				} else {
					//旋回以外から、ここに入ったらブレーキ時間を短い時間にする
					//斜めのラインの旋回からの復帰時は、ここに入るはず。
					_delay_ms(50);
				}
				Execute(MOVE_SELECTION_TYPE_STRAIGHT);//直進実行
				updateAction(currentAction, nextAction);//PID制御を使った直進に遷移。
				break;
			}
			else
			{
			//旋回中に、中央のセンサーが白ならここ。
				LED_on(1);
				Execute(MOVE_SELECTION_TYPE_RIGHTTURN);//右旋回を再実行
				_delay_ms(10);//10ms旋回を継続
			}
        }
        break;
        
    //左旋回用の分岐
    //一度、ここに入ると中央のセンサーが黒になるまでループし続ける。
    //右旋回の処理と同じ考えで、回転方向が左旋回。
    case MOVE_SELECTION_TYPE_LEFTTURN:
    case MOVE_SELECTION_TYPE_LEFTTURN_3:
        LED_on(5);
        Execute(MOVE_SELECTION_TYPE_LEFTTURN);
		if (currentAction == MOVE_SELECTION_TYPE_LEFTTURN) {
			_delay_ms(150);
		}
        while(1) {
            nextAction = getAction();
			if (nextAction == MOVE_SELECTION_TYPE_STRAIGHT ||
                nextAction == MOVE_SELECTION_TYPE_LEFTSIFT_1 ||
                nextAction == MOVE_SELECTION_TYPE_LEFTSIFT_2 ||
                nextAction == MOVE_SELECTION_TYPE_RIGHTSIFT_1 ||
                nextAction == MOVE_SELECTION_TYPE_RIGHTSIFT_2)
			{
				PID_reset_diff_integral();
				Execute(MOVE_SELECTION_TYPE_RIGHTTURN);
				if (currentAction == MOVE_SELECTION_TYPE_LEFTTURN) {
					_delay_ms(100);
				} else {
					_delay_ms(50);
				}
				Execute(MOVE_SELECTION_TYPE_STRAIGHT);
				updateAction(currentAction, nextAction);
				break;
			}
			else
			{
				LED_on(5);
				Execute(MOVE_SELECTION_TYPE_LEFTTURN);
				_delay_ms(10);
			}

        }
        break;

    //全白ならここ。
    //2015年のゴール判定用。
	case MOVE_SELECTION_TYPE_STRAIGHT_2:
    	//LEDをすべて点灯
        LED_on(1);
        LED_on(2);
        LED_on(3);
        LED_on(4);
        LED_on(5);
        LED_on(6);
		Execute(currentAction);//前回の判定で、モーターを駆動
		nextAction = getAction();//現在のセンサー値で、次の動作を判断
		updateAction(currentAction, nextAction);//次の動作を決定
		break;

    default:
        updateAction(0, MOVE_SELECTION_TYPE_STOP);
        ret_state = 0;
        break;
    }

    return ret_state;
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

/**
* センサー値を参照しアクションを取得する。
* @brief センサー値を参照しアクションを取得する。
* @return 戻り値の説明
*/
int getAction(void) {
    int ret = 0;
    
	// センサー値を取得
	getSensors();

	// LEDを設定
	setLED();

	switch(IR_BitPattern) { 
	//直進関連
	case BIT_010100:
	case BIT_010101:
	case BIT_110110:
	case BIT_110111:
	case BIT_011100:
	case BIT_011101:
	case BIT_001000:
	case BIT_001001:
		ret = MOVE_SELECTION_TYPE_STRAIGHT;
		LOG_DEBUG("getAction() MOVE_SELECTION_TYPE_STRAIGHT\r\n");
		break;
	case BIT_000000:
	case BIT_000001:
		ret = MOVE_SELECTION_TYPE_STRAIGHT_2;
		LOG_DEBUG("getAction() MOVE_SELECTION_TYPE_STRAIGHT_2\r\n");
		break;
	//検索
	case BIT_111110:
	case BIT_111111:
        LED_on(2);
        LED_on(5);
		ret = MOVE_SELECTION_TYPE_SEARCH;
		LOG_DEBUG("getAction() MOVE_SELECTION_TYPE_SEARCH\r\n");
		break;
	//直進関連（PID制御）
	case BIT_001100:
	case BIT_001101:
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_1;
		PID_ctlr = 1;
		LOG_DEBUG("getAction() MOVE_SELECTION_TYPE_RIGHTSIFT_1 : PID_ctlr[%d]\r\n", PID_ctlr);
		break;
	case BIT_000100:
	case BIT_000101:
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_1;
		PID_ctlr = 3;
		LOG_DEBUG("getAction() MOVE_SELECTION_TYPE_RIGHTSIFT_1 : PID_ctlr[%d]\r\n", PID_ctlr);
		break;
	case BIT_000110:
	case BIT_000111:
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		PID_ctlr = 5;
		LOG_DEBUG("getAction() MOVE_SELECTION_TYPE_RIGHTSIFT_2 : PID_ctlr[%d]\r\n", PID_ctlr);
		break;
	case BIT_011000:
	case BIT_011001:
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_1;
		PID_ctlr = -1;
		LOG_DEBUG("getAction() MOVE_SELECTION_TYPE_LEFTSIFT_1 : PID_ctlr[%d]\r\n", PID_ctlr);
		break;	
	case BIT_010000:
	case BIT_010001:
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_1;
		PID_ctlr = -3;
		LOG_DEBUG("getAction() MOVE_SELECTION_TYPE_LEFTSIFT_1 : PID_ctlr[%d]\r\n", PID_ctlr);
		break;
	case BIT_110000:
	case BIT_110001:
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		PID_ctlr = -5;
		LOG_DEBUG("getAction() MOVE_SELECTION_TYPE_LEFTSIFT_2 : PID_ctlr[%d]\r\n", PID_ctlr);
		break;
	//右旋回関連
	case BIT_001110:
	case BIT_001111:
		ret = MOVE_SELECTION_TYPE_RIGHTTURN;
		LOG_DEBUG("getAction() MOVE_SELECTION_TYPE_RIGHTTURN\r\n");
		break;
	case BIT_000010:
	case BIT_000011:
		ret = MOVE_SELECTION_TYPE_RIGHTTURN_3;
		PID_ctlr = 9;
		LOG_DEBUG("getAction() MOVE_SELECTION_TYPE_RIGHTTURN_3\r\n");
		break;
	//左旋回関連	
	case BIT_111000:
	case BIT_111001:
		ret = MOVE_SELECTION_TYPE_LEFTTURN;
		LOG_DEBUG("getAction() MOVE_SELECTION_TYPE_LEFTTURN\r\n");
		break;
	case BIT_100000:
	case BIT_100001:
		ret = MOVE_SELECTION_TYPE_LEFTTURN_3;
		PID_ctlr = -9;
		LOG_DEBUG("getAction() MOVE_SELECTION_TYPE_LEFTTURN_3\r\n");
		break;
	//defaultは検索に設定
	default:
		ret = MOVE_SELECTION_TYPE_SEARCH;
		LOG_DEBUG("getAction() case is default MOVE_SELECTION_TYPE_SEARCH\r\n");
		break;
	}
	
    return ret;
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
