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
// Trace status
#define TRACE_UNKNOWN		2000	// 判定不能(前回の動作を継続)
#define TRACE_STRAIGHT		2001	// 直進
#define TRACE_LEFTMOVE		2002	// 左前進
#define TRACE_RIGHTMOVE		2003	// 右前進
#define TRACE_LEFTTURN		2004	// 左旋回
#define TRACE_RIGHTTURN		2005	// 右旋回
#define TRACE_FINALACTION	2006	// ゴール動作

// Line Sensor
#define LINE_STATE_BLACK    0		// センサー値でラインが黒判定
#define LINE_STATE_WHITE    1		// センサー値でラインが白判定

#define _LED_ON_

// ------------------ Method Definition ------------------
void executeTraceProcess(void);
int getActionBySensors(void);
void decideTraceStatus(int action);

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

// Current trace status
int currentStatus = TRACE_STRAIGHT;

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

int mMoveCount = 0;


// PID Param
float pGain = 200;   //Proportional Gain
float iGain =  0.2;  //Integral Gain
float dGain =  120;  //Differential Gain
int delay = 10;
int32_t eInteg = 0;  //Integral accumulator
int32_t ePrev  =0;   //Previous Error

int PID_ctlr = 0;	//!< PID制御用変数。中心のセンサからの距離を入力することで、直進時のブレを抑制する制御を行う。

// ------------------ Method ------------------

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
	int actionBySensors = 0;
	
	while (1) {		  	
		// 現在のセンサー値に対応した動作を取得する。
    	actionBySensors = getActionBySensors();
		// 次の動作が最終動作であればループを抜ける。
		if (actionBySensors == TRACE_FINALACTION) {
			break;
		}
		
		// 前回動作を踏まえて次の動作を決定する。
		decideTraceStatus(actionBySensors);
		Execute(currentStatus);
	}
}

/**
* センサー値を参照しアクションを取得する。
* @brief センサー値を参照しアクションを取得する。
* @return 戻り値の説明
*/
int getActionBySensors(void) {
    int ptn = 0;
	int ret = 0;
	
	// LEDを設定
	setLED();
	
	// センサー値を取得
	getSensors();
	
	// 判定条件数を減らすためゴール判定用センサ値はフィルタリングしておく
	ptn = ((IR_BitPattern >> 1) << 1);
	
	// 直進関連
	if ((ptn == BIT_010100) ||
		(ptn == BIT_110110) ||
		(ptn == BIT_011100) ||
		(ptn == BIT_001000)) {
		ret = TRACE_STRAIGHT;
		LOG_DEBUG("getActionBySensors() TRACE_STRAIGHT\r\n");
	}
	// 右前進
	else if ((ptn == BIT_001100) ||
			 (ptn == BIT_000100)) {
		ret = TRACE_RIGHTMOVE;
		LOG_DEBUG("getActionBySensors() TRACE_RIGHTMOVE\r\n");
	}
	// 左前進
	else if ((ptn == BIT_011000) ||
			 (ptn == BIT_010000)) {
		ret = TRACE_LEFTMOVE;
		LOG_DEBUG("getActionBySensors() TRACE_LEFTMOVE\r\n");
	}
	// 右旋回関連
	else if ((ptn == BIT_001110) ||
			 (ptn == BIT_000110) ||
			 (ptn == BIT_000010)) {	 
		ret = TRACE_RIGHTTURN;
		LOG_DEBUG("getActionBySensors() TRACE_RIGHTTURN\r\n");
	}
	//左旋回関連
	else if ((ptn == BIT_111000) ||
			 (ptn == BIT_110000) ||
			 (ptn == BIT_100000)) {
		ret = TRACE_LEFTTURN;
		LOG_DEBUG("getActionBySensors() TRACE_LEFTTURN\r\n");
	}
	// その他
	else {
		LED_on(2);
		LED_on(5);
		ret = TRACE_UNKNOWN;
		LOG_DEBUG("getActionBySensors() TRACE_UNKNOWN\r\n");
	}

 	// ゴール判定（ゴール用センサを連続3回検知）
	if ((IR_BitPattern & BIT_GOAL_JUDGE_ON) == BIT_GOAL_JUDGE_ON) {
		goalCounter++;
		if (goalCounter >= 3) {
			ret = TRACE_FINALACTION;
		}
	} else {
		goalCounter = 0;
	}
	
	return ret;
}

void decideTraceStatus(int actionBySensor) {
	// センサの対応動作が TRACE_UNKNOWN なら前回の動作を継続する
	if (actionBySensor == TRACE_UNKNOWN) {
		return;
	}
	
	// TODO: 以下の状態遷移別動作処理を実装
	// STS_TRACE_STRAIGHT
	// STS_TRACE_LEFTMOVE
	// STS_TRACE_RIGHTMOVE
	// STS_TRACE_LEFTTURN
	// STS_TRACE_RIGHTTURN
}

/**
* センサー値から次の行動パターンを決定
* @brief センサー値から次の行動パターンを決定
* @return メインプログラムのステータス
* @detail センサー値から次の行動パターンを決定し、戻り値にメインプログラムのステータスを返す。
*/
int decideMoveAction(void) {
    int ret_state = 0;//メインプログラムのステータス
	
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
	if ( IR[GOAL_JUDGE]		>= COMPARE_VALUE )	IR_BitPattern |= BIT_GOAL_JUDGE_ON;
	if ( IR[RIGHT_OUTSIDE]	>= COMPARE_VALUE )	IR_BitPattern |= BIT_RIGHT_OUTSIDE_ON;
	if ( IR[RIGHT_INSIDE]	>= COMPARE_VALUE )	IR_BitPattern |= BIT_RIGHT_INSIDE_ON;
	if ( IR[CENTER]			>= COMPARE_VALUE )	IR_BitPattern |= BIT_CENTER_ON;
	if ( IR[LEFT_INSIDE]	>= COMPARE_VALUE )	IR_BitPattern |= BIT_LEFT_INSIDE_ON;
	if ( IR[LEFT_OUTSIDE]	>= COMPARE_VALUE )	IR_BitPattern |= BIT_LEFT_OUTSIDE_ON;

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
	case BIT_010101:
	case BIT_110111:
	case BIT_011100:
	case BIT_001001:
		ret = MOVE_SELECTION_TYPE_STRAIGHT;
		LOG_DEBUG("getAction() MOVE_SELECTION_TYPE_STRAIGHT\r\n");
		break;
	case BIT_000000:
		ret = MOVE_SELECTION_TYPE_STRAIGHT_2;
		LOG_DEBUG("getAction() MOVE_SELECTION_TYPE_STRAIGHT_2\r\n");
		break;
	//検索
	case BIT_111110:
        LED_on(2);
        LED_on(5);
		ret = MOVE_SELECTION_TYPE_SEARCH;
		LOG_DEBUG("getAction() MOVE_SELECTION_TYPE_SEARCH\r\n");
		break;
	//直進関連（PID制御）
	case BIT_001101:
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_1;
		PID_ctlr = 1;
		LOG_DEBUG("getAction() MOVE_SELECTION_TYPE_RIGHTSIFT_1 : PID_ctlr[%d]\r\n", PID_ctlr);
		break;
	case BIT_000100:
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_1;
		PID_ctlr = 3;
		LOG_DEBUG("getAction() MOVE_SELECTION_TYPE_RIGHTSIFT_1 : PID_ctlr[%d]\r\n", PID_ctlr);
		break;
	case BIT_000110:
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		PID_ctlr = 5;
		LOG_DEBUG("getAction() MOVE_SELECTION_TYPE_RIGHTSIFT_2 : PID_ctlr[%d]\r\n", PID_ctlr);
		break;
	case BIT_011001:
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_1;
		PID_ctlr = -1;
		LOG_DEBUG("getAction() MOVE_SELECTION_TYPE_LEFTSIFT_1 : PID_ctlr[%d]\r\n", PID_ctlr);
		break;	
	case BIT_010000:
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_1;
		PID_ctlr = -3;
		LOG_DEBUG("getAction() MOVE_SELECTION_TYPE_LEFTSIFT_1 : PID_ctlr[%d]\r\n", PID_ctlr);
		break;
	case BIT_110000:
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		PID_ctlr = -5;
		LOG_DEBUG("getAction() MOVE_SELECTION_TYPE_LEFTSIFT_2 : PID_ctlr[%d]\r\n", PID_ctlr);
		break;
	//右旋回関連
	case BIT_001110:
	case BIT_001100:
	case BIT_000111:
	case BIT_000011:
		ret = MOVE_SELECTION_TYPE_RIGHTTURN;
		LOG_DEBUG("getAction() MOVE_SELECTION_TYPE_RIGHTTURN\r\n");
		break;
	case BIT_000010:
		ret = MOVE_SELECTION_TYPE_RIGHTTURN_3;
		PID_ctlr = 9;
		LOG_DEBUG("getAction() MOVE_SELECTION_TYPE_RIGHTTURN_3\r\n");
		break;
	//左旋回関連	
	case BIT_111000:
	case BIT_011000:
	case BIT_110001:
	case BIT_100001:
		ret = MOVE_SELECTION_TYPE_LEFTTURN;
		LOG_DEBUG("getAction() MOVE_SELECTION_TYPE_LEFTTURN\r\n");
		break;
	case BIT_100000:
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
	MotorControl( COVER_MOTOR, 300 );
	_delay_ms(1500);
	MotorControl( COVER_MOTOR, 0 );
	_delay_ms(100);
    MotorControl( RIGHT_MOTOR, 1623 );
    MotorControl( LEFT_MOTOR, 600 );
	_delay_ms(500);
	Execute(MOVE_SELECTION_TYPE_STOP);
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
