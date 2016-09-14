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
// State
//メイン関数のステータス
#define STATE_INIT           2000//ボードの初期化
#define STATE_STOP           2001//ロボットの停止
#define STATE_START          2002//ロボット始動
#define STATE_MOVE           2003//ロボット動作中

// Line Sensor
#define LINE_STATE_BLACK    0//センサー値でラインが黒判定
#define LINE_STATE_WHITE    1//センサー値でラインが黒判定



// START SW address
#define SW_START 0x01   // Emergency Stop 未検証ですがトレイルのロボはこの設定不要だと思う

//#define MAX_STOP_COUNT 20
//ゴールカウントがこの値を超えるとゴールだと判定
//2015年は全白だったときの回数をカウントしてゴール判定した
#define MAX_STOP_COUNT 80

#define _LED_ON_

// ------------------ Method Definition ------------------
void split( char * s1 );
void initMoveAction(void);
int decideMoveAction(void);
int getAction(void);

void getSensors(void);
int getState(void);
void setState(int state);

void updateAction(int currentAction, int nextAction);

void executeFinalAction(void);

void initEmergencyStop(void);

void setLED(void);
void LED_on(int i);
void LED_off(int i);

// ------------------ Global Variables Definition ------------------

// Serial Message Buf
int serCmd[SERIAL_BUFFER_SIZE] = {0};

// State
int mState = STATE_INIT;

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
    char * readData = NULL; 
    int isFinish = 0;
    
    while(1){
        //シリアル接続モード？
        memset( &serCmd[0], 0x00, sizeof(int) * SERIAL_BUFFER_SIZE );
        if( checkSerialRead() > 0 ) {
            readData = getReadBuffer();
            if( readData != NULL ){
                LOG_INFO( "readData=%s\n", &readData[0] );
                split( &readData[0] );
                switch( serCmd[0] ) {
                case 1:
                    break;
                case 999:
                    LOG_INFO("finish\n");
                    isFinish = 1;
                    break;
                }
                if( isFinish > 0 ) {
                    break;
                }
                memset( readData, 0x00, SERIAL_BUFFER_SIZE );
            }
            _delay_ms(500);
        } else {
            //ロボ動作開始
            int state = getState();//初期値：ACTION_STATE_INIT
            //緊急停止がきたら止まる？
            if (~PIND & SW_START) {
                // Emergency stopped
                state = STATE_STOP;
            }
            
			//ゴール判定用のカウント for 2015
			if(mCount >= MAX_STOP_COUNT) {
				executeFinalAction();
				break;
			}
				
            switch (state) {
            //最初はここ。
            case STATE_INIT:
            	initMoveAction();
                setState(STATE_STOP);
                break;
            //初期化の終わりもここ
            case STATE_STOP:
                // motor stop
                Execute(MOVE_SELECTION_TYPE_STOP);
                setState(STATE_START);
                break;
                
            case STATE_START:
                setState(STATE_MOVE);
                break;
            //実質ここから開始？
            case STATE_MOVE:
                setState(decideMoveAction());
                break;
            
            default:
                setState(STATE_INIT);
                break;
            }
        }
    }
}

/**
* 文字列を分割
* @brief 文字列を分割
* @param (char * s1) 分割する文字列
* @return なし
*/
void split( char * s1 ) {
    char s2[] = " ,";
    char *tok;
    int cnt = 0;

    tok = strtok( s1, s2 );
    while( tok != NULL ){
        serCmd[cnt++] = atoi(tok);
        tok = strtok( NULL, s2 );  /* 2回目以降 */
    }
}

/**
* 電源ON後の初期アクション
* @brief 電源ON後の初期アクション
* @return なし
* @detail 電源ON後の初期アクションを設定する。
*/
void initMoveAction(void) {
    //Execute(MOVE_SELECTION_TYPE_LEFTTURN);//左回転・・・電源ONで180度回転させる設定。逆走する場合、これを復帰
    //_delay_ms(1000);    // 1s・・・電源ONで180度回転させる設定。逆走する場合、これを復帰
    Execute(MOVE_SELECTION_TYPE_STOP);//次の動作：停止
}

/**
* センサー値から次の行動パターンを決定
* @brief センサー値から次の行動パターンを決定
* @return メインプログラムのステータス
* @detail センサー値から次の行動パターンを決定し、戻り値にメインプログラムのステータスを返す。
*/
int decideMoveAction(void) {
    int ret_state = STATE_MOVE;//メインプログラムのステータス
	int rightVal = 0;
	int leftVal = 0;
	
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
        PID_ctlr_Update(0, PID_ctlr, &rightVal, &leftVal);//PID制御の制御値を更新
        setParamMoveAction(rightVal, leftVal);//モーターの駆動指令

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
        ret_state = STATE_STOP;
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
* メイン関数分岐用のステータスを取得
* @brief メイン関数分岐用のステータスを取得
* @return メイン関数分岐用のステータス
*/
int getState(void) {
    return mState;
}

/**
 * メイン関数分岐用のステータスを設定
 * @brief メイン関数分岐用のステータスを取得
 * @param (int state) メイン関数分岐用のステータス
 * @return なし
 */
void setState(int state) {
    mState = state;
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
	case BIT_111111:
        LED_on(2);
        LED_on(5);
		ret = MOVE_SELECTION_TYPE_SEARCH;
		LOG_DEBUG("BIT_111111\r\n");		
		break;
	case BIT_101111:
		ret = MOVE_SELECTION_TYPE_SEARCH;
		LOG_DEBUG("BIT_101111\r\n");
		break;
	case BIT_110111:
		ret = MOVE_SELECTION_TYPE_SEARCH;
		LOG_DEBUG("BIT_110111\r\n");
		break;
	case BIT_100111:
		ret = MOVE_SELECTION_TYPE_SEARCH;
		LOG_DEBUG("BIT_100111\r\n");
		break;
	case BIT_111011:
		ret = MOVE_SELECTION_TYPE_SEARCH;
		LOG_DEBUG("BIT_111011\r\n");
		break;
	case BIT_101011:
		ret = MOVE_SELECTION_TYPE_SEARCH;
		LOG_DEBUG("BIT_101011\r\n");
		break;
	case BIT_110011:
		ret = MOVE_SELECTION_TYPE_S_MOVE_1;
		LOG_DEBUG("BIT_110011\r\n");
		break;
	case BIT_100011:
		ret = MOVE_SELECTION_TYPE_S_MOVE_1;
		LOG_DEBUG("BIT_100011\r\n");
		break;
	case BIT_111101:
		ret = MOVE_SELECTION_TYPE_SEARCH;
		LOG_DEBUG("BIT_111101\r\n");
		break;
	case BIT_101101:
		ret = MOVE_SELECTION_TYPE_SEARCH;
		LOG_DEBUG("BIT_101101\r\n");
		break;
	case BIT_110101:
		ret = MOVE_SELECTION_TYPE_SEARCH;
		LOG_DEBUG("BIT_110101\r\n");
		break;
	case BIT_100101:
		ret = MOVE_SELECTION_TYPE_SEARCH;
		LOG_DEBUG("BIT_100101\r\n");
		break;
	case BIT_111001:
		ret = MOVE_SELECTION_TYPE_SEARCH;
		LOG_DEBUG("BIT_111001\r\n");
		break;
	case BIT_101001:
		ret = MOVE_SELECTION_TYPE_SEARCH;
		LOG_DEBUG("BIT_101001\r\n");
		break;
	case BIT_110001:
		ret = MOVE_SELECTION_TYPE_S_MOVE_1;
		LOG_DEBUG("BIT_110001\r\n");
		break;
	case BIT_100001:
		ret = MOVE_SELECTION_TYPE_S_MOVE_1;
		LOG_DEBUG("BIT_100001\r\n");
		break;
	case BIT_011111:
		ret = MOVE_SELECTION_TYPE_RIGHTTURN;
		LOG_DEBUG("BIT_011111\r\n");
		break;
	case BIT_001111:
		ret = MOVE_SELECTION_TYPE_RIGHTTURN;
		LOG_DEBUG("BIT_001111\r\n");
		break;
	case BIT_010111:
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		PID_ctlr = 9;
		LOG_DEBUG("BIT_010111\r\n");
		break;
	case BIT_000111:
		ret = MOVE_SELECTION_TYPE_RIGHTTURN;
		LOG_DEBUG("BIT_000111\r\n");
		break;
	case BIT_011011:
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		PID_ctlr = 9;
		LOG_DEBUG("xBIT_011011xxx\r\n");
		break;
	case BIT_001011:
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		PID_ctlr = 9;
		LOG_DEBUG("BIT_001011\r\n");
		break;
	case BIT_010011:
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		PID_ctlr = 9;
		LOG_DEBUG("BIT_010011\r\n");
		break;
	case BIT_000011:
		ret = MOVE_SELECTION_TYPE_RIGHTTURN_3;
		PID_ctlr = 9;
		LOG_DEBUG("BIT_000011\r\n");
		break;
	case BIT_011101:
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		PID_ctlr = 9;
		LOG_DEBUG("BIT_011101\r\n");
		break;
	case BIT_001101:
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		PID_ctlr = 9;
		LOG_DEBUG("BIT_001101\r\n");
		break;
	case BIT_010101:
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		PID_ctlr = 9;
		LOG_DEBUG("BIT_010101\r\n");
		break;
	case BIT_000101:
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		PID_ctlr = 9;
		LOG_DEBUG("BIT_000101\r\n");
		break;
	case BIT_011001:
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		PID_ctlr = 9;
		LOG_DEBUG("BIT_011001\r\n");
		break;
	case BIT_001001:
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		PID_ctlr = 9;
		LOG_DEBUG("BIT_001001\r\n");
		break;
	case BIT_010001:
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		PID_ctlr = 9;
		LOG_DEBUG("BIT_010001\r\n");
		break;
	case BIT_000001:
		ret = MOVE_SELECTION_TYPE_RIGHTTURN_3;
		PID_ctlr = 9;
		LOG_DEBUG("BIT_000001\r\n");
		break;
	case BIT_111110:
		ret = MOVE_SELECTION_TYPE_LEFTTURN;
		LOG_DEBUG("BIT_111110\r\n");
		break;
	case BIT_101110:
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		PID_ctlr = -9;
		LOG_DEBUG("BIT_101110\r\n");
		break;
	case BIT_110110:
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		PID_ctlr = -9;
		LOG_DEBUG("BIT_110110\r\n");
		break;
	case BIT_100110:
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		PID_ctlr = -9;
		LOG_DEBUG("BIT_100110\r\n");
		break;
	case BIT_111010:
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		PID_ctlr = -9;
		LOG_DEBUG("BIT_111010\r\n");
		break;
	case BIT_101010:
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		PID_ctlr = -9;
		LOG_DEBUG("BIT_101010\r\n");
		break;
	case BIT_110010:
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		PID_ctlr = -9;
		LOG_DEBUG("BIT_110010\r\n");
		break;
	case BIT_100010:
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		PID_ctlr = -9;
		LOG_DEBUG("BIT_100010\r\n");
		break;
	case BIT_111100:
		ret = MOVE_SELECTION_TYPE_LEFTTURN;
		LOG_DEBUG("BIT_111100\r\n");
		break;
	case BIT_101100:
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		PID_ctlr = -9;
		LOG_DEBUG("BIT_101100\r\n");
		break;
	case BIT_110100:
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		PID_ctlr = -9;
		LOG_DEBUG("BIT_110100\r\n");
		break;
	case BIT_100100:
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		PID_ctlr = -9;
		LOG_DEBUG("BIT_100100\r\n");
		break;
	case BIT_111000:
		ret = MOVE_SELECTION_TYPE_LEFTTURN;
		LOG_DEBUG("BIT_111000\r\n");
		break;
	case BIT_101000:
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		PID_ctlr = -9;
		LOG_DEBUG("BIT_101000\r\n");
		break;
	case BIT_110000:
		ret = MOVE_SELECTION_TYPE_LEFTTURN_3;
		PID_ctlr = -9;
		LOG_DEBUG("BIT_110000\r\n");
		break;
	case BIT_100000:
		ret = MOVE_SELECTION_TYPE_LEFTTURN_3;
		PID_ctlr = -9;
		LOG_DEBUG("BIT_100000\r\n");
		break;
	case BIT_011110:
		ret = MOVE_SELECTION_TYPE_STRAIGHT;
		LOG_DEBUG("BIT_011110\r\n");
		break;
	case BIT_001110:
		ret = MOVE_SELECTION_TYPE_S_MOVE_4;
		LOG_DEBUG("BIT_001110\r\n");
		break;
	case BIT_010110:
		ret = MOVE_SELECTION_TYPE_S_MOVE_4;
		LOG_DEBUG("BIT_010110\r\n");
		break;
	case BIT_000110:
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_1;
		PID_ctlr = 3;//2
		LOG_DEBUG("BIT_000110\r\n");
		break;
	case BIT_011010:
		ret = MOVE_SELECTION_TYPE_S_MOVE_4;
		LOG_DEBUG("BIT_011010\r\n");
		break;
	case BIT_001010:
		ret = MOVE_SELECTION_TYPE_S_MOVE_4;
		LOG_DEBUG("BIT_001010\r\n");
		break;
	case BIT_010010:
		ret = MOVE_SELECTION_TYPE_S_MOVE_4;
		LOG_DEBUG("BIT_010010\r\n");
		break;
	case BIT_000010:
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		PID_ctlr = 5;//3
		LOG_DEBUG("BIT_000010\r\n");
		break;
	case BIT_011100:
		ret = MOVE_SELECTION_TYPE_S_MOVE_4;
		LOG_DEBUG("BIT_011100\r\n");
		break;
	case BIT_001100:
		ret = MOVE_SELECTION_TYPE_STRAIGHT;
		LOG_DEBUG("BIT_001100\r\n");
		break;
	case BIT_010100:
		ret = MOVE_SELECTION_TYPE_S_MOVE_4;
		LOG_DEBUG("BIT_010100\r\n");
		break;
	case BIT_000100:
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_1;
		PID_ctlr = 1;
		LOG_DEBUG("BIT_000100\r\n");
		break;
	case BIT_011000:
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_1;
		PID_ctlr = -3;//-2
		LOG_DEBUG("BIT_011000\r\n");
		break;
	case BIT_001000:
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_1;
		PID_ctlr = -1;
		LOG_DEBUG("BIT_001000\r\n");
		break;
	case BIT_010000:
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		PID_ctlr = -5;//-3
		LOG_DEBUG("BIT_010000\r\n");
		break;
	case BIT_000000:
		ret = MOVE_SELECTION_TYPE_STRAIGHT_2;
		LOG_DEBUG("BIT_000000\r\n");
		break;
	default:
		LOG_DEBUG("default\r\n");
		break;
	}
	
	LOG_DEBUG("getAction() ret=%5d PID_ctlr=%5d\r\n",ret ,PID_ctlr );
	
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
