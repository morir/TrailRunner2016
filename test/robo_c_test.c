#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
//#include <util/delay.h>
#include <windows.h>

// ------------------ Defined ------------------
// Line Sensor
#define LINE_STATE_BLACK    0//センサー値でラインが白判定
#define LINE_STATE_WHITE    1//センサー値でラインが黒判定

#define _LED_ON_

// Move Type
#define MOVE_SELECTION_TYPE_START       1000
#define MOVE_SELECTION_TYPE_STOP        1001
#define MOVE_SELECTION_TYPE_STRAIGHT    1002
#define MOVE_SELECTION_TYPE_RIGHTSIFT_1 1003
#define MOVE_SELECTION_TYPE_RIGHTSIFT_2 1004
#define MOVE_SELECTION_TYPE_LEFTSIFT_1  1005
#define MOVE_SELECTION_TYPE_LEFTSIFT_2  1006
#define MOVE_SELECTION_TYPE_BACK        1007
#define MOVE_SELECTION_TYPE_RIGHTTURN   1008
#define MOVE_SELECTION_TYPE_LEFTTURN    1009
#define MOVE_SELECTION_TYPE_RIGHTTURN_2 1010
#define MOVE_SELECTION_TYPE_LEFTTURN_2  1011
#define MOVE_SELECTION_TYPE_SEARCH      1012
#define MOVE_SELECTION_TYPE_STRAIGHT_2	1013
#define MOVE_SELECTION_TYPE_SLOWBACK    1014
#define MOVE_SELECTION_TYPE_RIGHTTURN_3 1015
#define MOVE_SELECTION_TYPE_LEFTTURN_3  1016

// Trace pattern
#define TRACE_STRAIGHT		0	// 直進
#define TRACE_L_STRAIGHT	1	// 左前進
#define TRACE_L_ROUND		2	// 左カーブ
#define TRACE_R_STRAIGHT	3	// 右前進
#define TRACE_R_ROUND		4	// 右カーブ
#define TRACE_L_TURN		5	// 左旋回
#define TRACE_R_TURN		6	// 右旋回
#define TRACE_FINALACTION	7	// ゴール動作

#define SERIAL_BUFFER_SIZE 32

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

// ゴールセンサの検知最大数
#define GOAL_DETECTED_MAX_COUNT 10

/** ログ出力を設定(定義値が有効な場合、ログを出力する) */
#define LOG_FATAL_ON	//<! 致命的なエラーログの出力設定	：常時ON
#define LOG_ERROR_ON	//<! エラーログの出力設定			：常時ON
#define LOG_WARN_ON		//<! 警告の出力設定					：常時ON
#define LOG_INFO_ON		//<! 情報ログの出力設定				：常時ON
#define LOG_DEBUG_ON	//<! デバッグログの出力設定			：常時OFF

#if defined(LOG_FATAL_ON)
#define LOG_FATAL(...)	{printf("[FATAL] "); printf("[%s]", __func__); printf(__VA_ARGS__);}	//!< 致命的なエラーログ出力
#else
#define LOG_FATAL(...)	//!< 致命的なエラーログ出力
#endif

#if defined(LOG_ERROR_ON)
#define LOG_ERROR(...)	{printf("[ERROR] "); printf("[%s]", __func__); printf(__VA_ARGS__);}	//!< エラーログ出力
#else
#define LOG_ERROR(...)	//!< エラーログ出力
#endif

#if defined(LOG_WARN_ON)
#define LOG_WARN(cdbg, ...)	{printf("[WARN ] "); printf("[%s]", __func__); printf(__VA_ARGS__);}	//!< 警告ログ出力
#else
#define LOG_WARN(...)	//!< 警告ログ出力
#endif

#if defined(LOG_INFO_ON)
#define LOG_INFO(...)	{printf("[INFO ] "); printf("[%s]", __func__); printf(__VA_ARGS__);}	//!< 情報ログ出力
#else
#define LOG_INFO(...)	//!< 情報ログ出力
#endif

#if defined(LOG_DEBUG_ON)
#define LOG_DEBUG(...)  {printf("[DEBUG] "); printf("[%s]", __func__); printf(__VA_ARGS__);}	//!< デバッグログ出力
#else
#define LOG_DEBUG(...)	//!< デバッグログ出力
#endif

#define RIGHT_MOTOR         30      // Right Motor address
#define LEFT_MOTOR          31      // Left Motor address
#define DELAY_MSEC          1       // Delay time
#define OVER_RUN_TIME       500     // Over run time


// ------------------ Method Definition ------------------
void MotorInit(void);
void MotorControl(int id, int power);
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
void Move(int leftSpeed, int rightSpeed);
void LeftStraightMove(void);
void RightStraightMove(void);
void LeftRoundMove(void);
void RightRoundMove(void);
void LeftTurnMove(void);
void RightTurnMove(void);
void PrintErrorCode(void);
void PrintCommStatus(int CommStatus);

void AdjustSpeed(int targetSpeedL, int targetSpeedR);
int GetCurrentSpeed(int id);

void ReadIRSensors(unsigned int * sensors);
unsigned int ReadIRSensor(unsigned int ch);

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

// PID Param
float pGain = 200;   //Proportional Gain
float iGain =  0.2;  //Integral Gain
float dGain =  120;  //Differential Gain
int delay = 10;

int PID_ctlr = 0;	//!< PID制御用変数。中心のセンサからの距離を入力することで、直進時のブレを抑制する制御を行う。


// IRの状態(BITパターン)
int IR_BitPattern = 0;

// Speed settings
int BaseSpeed = 400;
float LowRate = 0.75;
float HighRate = 0.5;


int mMoveCount = 0;

static int irIndex = 0;

// ------------------ Trace action table ------------------
int traceActionTable[32][7] = {
//						前回動作	
// Sensor Pattern		TRACE_STRAIGHT		TRACE_L_STRAIGHT	TRACE_L_ROUND		TRACE_R_STRAIGHT	TRACE_R_ROUND		TRACE_L_TURN		TRACE_R_TURN
//----------------------------------------------------------------------------------------------------------------------------------------------------------------
/* 00:BIT_00000x */		TRACE_STRAIGHT,		TRACE_L_STRAIGHT,	TRACE_L_ROUND,		TRACE_R_STRAIGHT,	TRACE_R_ROUND,		TRACE_L_TURN,		TRACE_R_TURN,
/* 01:BIT_00001x */		TRACE_R_ROUND,		TRACE_R_ROUND,		TRACE_R_ROUND,		TRACE_R_ROUND,		TRACE_R_ROUND,		TRACE_R_ROUND,		TRACE_R_ROUND,
/* 02:BIT_00010x */		TRACE_R_STRAIGHT,	TRACE_R_STRAIGHT,	TRACE_R_STRAIGHT,	TRACE_R_STRAIGHT,	TRACE_R_STRAIGHT,	TRACE_R_STRAIGHT,	TRACE_R_STRAIGHT,
/* 03:BIT_00011x */		TRACE_R_TURN,		TRACE_R_TURN,		TRACE_R_TURN,		TRACE_R_TURN,		TRACE_R_TURN,		TRACE_R_TURN,		TRACE_R_TURN,
/* 04:BIT_00100x */		TRACE_STRAIGHT,		TRACE_STRAIGHT,		TRACE_STRAIGHT,		TRACE_STRAIGHT,		TRACE_STRAIGHT,		TRACE_STRAIGHT,		TRACE_STRAIGHT,
/* 05:BIT_00101x */		TRACE_STRAIGHT,		TRACE_L_STRAIGHT,	TRACE_L_ROUND,		TRACE_R_STRAIGHT,	TRACE_R_ROUND,		TRACE_L_TURN,		TRACE_R_TURN,
/* 06:BIT_00110x */		TRACE_STRAIGHT,		TRACE_L_STRAIGHT,	TRACE_L_ROUND,		TRACE_R_STRAIGHT,	TRACE_R_ROUND,		TRACE_L_TURN,		TRACE_R_TURN,
/* 07:BIT_00111x */		TRACE_R_TURN,		TRACE_R_TURN,		TRACE_R_TURN,		TRACE_R_TURN,		TRACE_R_TURN,		TRACE_R_TURN,		TRACE_R_TURN,
/* 08:BIT_01000x */		TRACE_L_STRAIGHT,	TRACE_L_STRAIGHT,	TRACE_L_STRAIGHT,	TRACE_L_STRAIGHT,	TRACE_L_STRAIGHT,	TRACE_L_STRAIGHT,	TRACE_L_STRAIGHT,
/* 09:BIT_01001x */		TRACE_STRAIGHT,		TRACE_L_STRAIGHT,	TRACE_L_ROUND,		TRACE_R_STRAIGHT,	TRACE_R_ROUND,		TRACE_L_TURN,		TRACE_R_TURN,
/* 10:BIT_01010x */		TRACE_STRAIGHT,		TRACE_L_STRAIGHT,	TRACE_L_ROUND,		TRACE_R_STRAIGHT,	TRACE_R_ROUND,		TRACE_L_TURN,		TRACE_R_TURN,
/* 11:BIT_01011x */		TRACE_STRAIGHT,		TRACE_L_STRAIGHT,	TRACE_L_ROUND,		TRACE_R_STRAIGHT,	TRACE_R_ROUND,		TRACE_L_TURN,		TRACE_R_TURN,
/* 12:BIT_01100x */		TRACE_STRAIGHT,		TRACE_L_STRAIGHT,	TRACE_L_ROUND,		TRACE_R_STRAIGHT,	TRACE_R_ROUND,		TRACE_L_TURN,		TRACE_R_TURN,
/* 13:BIT_01101x */		TRACE_STRAIGHT,		TRACE_L_STRAIGHT,	TRACE_L_ROUND,		TRACE_R_STRAIGHT,	TRACE_R_ROUND,		TRACE_L_TURN,		TRACE_R_TURN,
/* 14:BIT_01110x */		TRACE_STRAIGHT,		TRACE_L_STRAIGHT,	TRACE_L_ROUND,		TRACE_R_STRAIGHT,	TRACE_R_ROUND,		TRACE_L_TURN,		TRACE_R_TURN,
/* 15:BIT_01111x */		TRACE_R_TURN,		TRACE_R_TURN,		TRACE_R_TURN,		TRACE_R_TURN,		TRACE_R_TURN,		TRACE_R_TURN,		TRACE_R_TURN,
/* 16:BIT_10000x */		TRACE_L_ROUND,		TRACE_L_ROUND,		TRACE_L_ROUND,		TRACE_L_ROUND,		TRACE_L_ROUND,		TRACE_L_ROUND,		TRACE_L_ROUND,
/* 17:BIT_10001x */		TRACE_STRAIGHT,		TRACE_L_STRAIGHT,	TRACE_L_ROUND,		TRACE_R_STRAIGHT,	TRACE_R_ROUND,		TRACE_L_TURN,		TRACE_R_TURN,
/* 18:BIT_10010x */		TRACE_STRAIGHT,		TRACE_L_STRAIGHT,	TRACE_L_ROUND,		TRACE_R_STRAIGHT,	TRACE_R_ROUND,		TRACE_L_TURN,		TRACE_R_TURN,
/* 19:BIT_10011x */		TRACE_STRAIGHT,		TRACE_L_STRAIGHT,	TRACE_L_ROUND,		TRACE_R_STRAIGHT,	TRACE_R_ROUND,		TRACE_L_TURN,		TRACE_R_TURN,
/* 20:BIT_10100x */		TRACE_STRAIGHT,		TRACE_L_STRAIGHT,	TRACE_L_ROUND,		TRACE_R_STRAIGHT,	TRACE_R_ROUND,		TRACE_L_TURN,		TRACE_R_TURN,
/* 21:BIT_10101x */		TRACE_STRAIGHT,		TRACE_L_STRAIGHT,	TRACE_L_ROUND,		TRACE_R_STRAIGHT,	TRACE_R_ROUND,		TRACE_L_TURN,		TRACE_R_TURN,
/* 22:BIT_10110x */		TRACE_STRAIGHT,		TRACE_L_STRAIGHT,	TRACE_L_ROUND,		TRACE_R_STRAIGHT,	TRACE_R_ROUND,		TRACE_L_TURN,		TRACE_R_TURN,
/* 23:BIT_10111x */		TRACE_STRAIGHT,		TRACE_L_STRAIGHT,	TRACE_L_ROUND,		TRACE_R_STRAIGHT,	TRACE_R_ROUND,		TRACE_L_TURN,		TRACE_R_TURN,
/* 24:BIT_11000x */		TRACE_L_TURN,		TRACE_L_TURN,		TRACE_L_TURN,		TRACE_L_TURN,		TRACE_L_TURN,		TRACE_L_TURN,		TRACE_L_TURN,
/* 25:BIT_11001x */		TRACE_STRAIGHT,		TRACE_L_STRAIGHT,	TRACE_L_ROUND,		TRACE_R_STRAIGHT,	TRACE_R_ROUND,		TRACE_L_TURN,		TRACE_R_TURN,
/* 26:BIT_11010x */		TRACE_STRAIGHT,		TRACE_L_STRAIGHT,	TRACE_L_ROUND,		TRACE_R_STRAIGHT,	TRACE_R_ROUND,		TRACE_L_TURN,		TRACE_R_TURN,
/* 27:BIT_11011x */		TRACE_STRAIGHT,		TRACE_L_STRAIGHT,	TRACE_L_ROUND,		TRACE_R_STRAIGHT,	TRACE_R_ROUND,		TRACE_L_TURN,		TRACE_R_TURN,
/* 28:BIT_11100x */		TRACE_L_TURN,		TRACE_L_TURN,		TRACE_L_TURN,		TRACE_L_TURN,		TRACE_L_TURN,		TRACE_L_TURN,		TRACE_L_TURN,
/* 29:BIT_11101x */		TRACE_STRAIGHT,		TRACE_L_STRAIGHT,	TRACE_L_ROUND,		TRACE_R_STRAIGHT,	TRACE_R_ROUND,		TRACE_L_TURN,		TRACE_R_TURN,
/* 30:BIT_11110x */		TRACE_L_TURN,		TRACE_L_TURN,		TRACE_L_TURN,		TRACE_L_TURN,		TRACE_L_TURN,		TRACE_L_TURN,		TRACE_L_TURN,
/* 31:BIT_11111x */		TRACE_STRAIGHT,		TRACE_STRAIGHT,		TRACE_STRAIGHT,		TRACE_STRAIGHT,		TRACE_STRAIGHT,		TRACE_STRAIGHT,		TRACE_STRAIGHT
};

// ------------------ Method Definition ------------------
void executeTraceProcess(void);
int getSensorPattern(void);

int decideMoveAction(void);
int getAction(void);

void getSensors(void);

void updateAction(int currentAction, int nextAction);

void executeFinalAction(void);

void initEmergencyStop(void);

//void setLED(void);
void LED_on(int i);
void LED_off(int i);
void wait(int sec);

/**
* エントリーポイント
* @brief エントリーポイント
* @return 0：メイン処理の継続
* @return 1：メイン処理の終了
*/
int main(void) {
    
	// ロボ動作開始

    // ショートカットモードを作る場合はここに入れる。
    
	// トレース動作開始
	executeTraceProcess();

    // ゴール判定後の動作実質ここから開始？
	executeFinalAction();
}

char *getActionChar(int actionType) {
	static char retChar[20];
	memset(retChar, 0, 20);
	if (actionType == TRACE_STRAIGHT) {
		strcpy(retChar, "TRACE_STRAIGHT\0");
	} else if (actionType == TRACE_L_STRAIGHT) {
		strcpy(retChar, "TRACE_L_STRAIGHT\0");
	} else if (actionType == TRACE_L_ROUND) {
		strcpy(retChar, "TRACE_L_ROUND\0");
	} else if (actionType == TRACE_R_STRAIGHT) {
		strcpy(retChar, "TRACE_R_STRAIGHT\0");
	} else if (actionType == TRACE_R_ROUND) {
		strcpy(retChar, "TRACE_R_ROUND\0");
	} else if (actionType == TRACE_L_TURN) {
		strcpy(retChar, "TRACE_L_TURN\0");
	} else if (actionType == TRACE_FINALACTION) {
		strcpy(retChar, "TRACE_FINALACTION\0");
	}

//	LOG_INFO("retChar[%s]\n", retChar);
	return (char*)retChar;
}

/**
* ライントレース動作
* @brief ライントレース動作
* @return なし
* @detail ゴール判定条件を満たすまでライントレース動作を行う。
*/
void executeTraceProcess(void) {
	static int previousTraceAction = TRACE_STRAIGHT;
	int currentTraceAction = TRACE_STRAIGHT;
	int sensorPattern = BIT_000000;
	
	while (1) {

		if (irIndex > 1000) {
			return;
		}
		// センサ値のビットパターンを取得する。
		sensorPattern = getSensorPattern();

/* 暫定コメントアウト		
		// センサ値のパターンが最終動作であればループを抜ける。
		if (sensorPattern == TRACE_FINALACTION) {
			break;
		}
*/

		// 前回の動作とセンサ値のパターンの組み合わせから今回の動作を決定する。
		currentTraceAction = traceActionTable[(sensorPattern / 2)][previousTraceAction];
		LOG_INFO("(sensorPattern / 2) %3d\n", (sensorPattern / 2));
		char *previousTraceActionChar = getActionChar(previousTraceAction);
		char *currentTraceActionChar = getActionChar(previousTraceAction);
		LOG_INFO("previousTraceAction [%20s]: sensorPattern [%3d]: currentTraceAction: [%20s]\n",
			previousTraceActionChar, sensorPattern, currentTraceActionChar);
		if(currentTraceAction != previousTraceAction)
		{
			Execute(currentTraceAction);
		}
		
//		delay_time(1000);	// 10ms 間隔を空ける
		
		// 今回の動作を前回の動作に退避する。
		previousTraceAction = currentTraceAction;

		irIndex++;
	}
}

/**
* センサー値のBitパターンを取得する。
* @brief センサー値を参照し、対応するアクションを取得する。
* @return 戻り値の説明
*/
int getSensorPattern(void) {
    LOG_INFO("\n======start===========\n");

	int ptn = 0;
	
	// LEDを設定
//	setLED();
	
	// センサー値を取得
	getSensors();
	
	// 判定条件数を減らすためゴール判定用センサ値をフィルタリングする。
	ptn = ((IR_BitPattern >> 1) << 1);

/* 暫定コメントアウト	
 	// ゴール判定（ゴール用センサを連続で規定数回検知した場合に設定）
	if ((IR_BitPattern & BIT_GOAL_JUDGE_ON) == BIT_GOAL_JUDGE_ON) {
		goalCounter++;
		if (goalCounter >= GOAL_DETECTED_MAX_COUNT) {
			ptn = TRACE_FINALACTION;
		}
	} else {
		goalCounter = 0;
	}
*/

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
	if ( IR[GOAL_JUDGE]		>= COMPARE_VALUE )	IR_BitPattern |= BIT_GOAL_JUDGE_ON;
	if ( IR[RIGHT_OUTSIDE]	>= COMPARE_VALUE )	IR_BitPattern |= BIT_RIGHT_OUTSIDE_ON;
	if ( IR[RIGHT_INSIDE]	>= COMPARE_VALUE )	IR_BitPattern |= BIT_RIGHT_INSIDE_ON;
	if ( IR[CENTER]			>= COMPARE_VALUE )	IR_BitPattern |= BIT_CENTER_ON;
	if ( IR[LEFT_INSIDE]	>= COMPARE_VALUE )	IR_BitPattern |= BIT_LEFT_INSIDE_ON;
	if ( IR[LEFT_OUTSIDE]	>= COMPARE_VALUE )	IR_BitPattern |= BIT_LEFT_OUTSIDE_ON;

    LOG_INFO("sensor %3d: %3d: %3d: %3d: %3d: %3d \n",
	       IR[LEFT_OUTSIDE], IR[LEFT_INSIDE], IR[CENTER], IR[RIGHT_INSIDE], IR[RIGHT_OUTSIDE], IR[GOAL_JUDGE]);
	LOG_DEBUG("IR[R %1d%1d%1d%1d%1d L] GOAL[%1d]\n",
				((IR[LEFT_OUTSIDE]	>= COMPARE_VALUE)?  1 : 0),
				((IR[LEFT_INSIDE]	>= COMPARE_VALUE)?  1 : 0),
				((IR[CENTER]		>= COMPARE_VALUE)?  1 : 0),
				((IR[RIGHT_INSIDE]	>= COMPARE_VALUE)?  1 : 0),
				((IR[RIGHT_OUTSIDE]	>= COMPARE_VALUE)?  1 : 0),
				((IR[GOAL_JUDGE]	>= COMPARE_VALUE)?  1 : 0));
}

void executeFinalAction(void) {
	LOG_INFO("executeFinalAction!! in\n");

	LOG_INFO("executeFinalAction!! out\n");
}

void ReadIRSensors(unsigned int * sensors) {
    unsigned int i = 0;
	
    for (i = ADC_PORT_1; i <= ADC_PORT_6; i++) {
        sensors[i] = ReadIRSensor(i);
    }
    return;
}

unsigned int ReadIRSensor(unsigned int ch) {
	return rand()%400;
	
}

void MotorControl(int id, int power) {
	LOG_INFO("id[%d] power[%d]\n",id ,power);

}

void Execute(int type) {
	//type = 0;//DBG
    switch (type) {
        case MOVE_SELECTION_TYPE_START:
        case MOVE_SELECTION_TYPE_STRAIGHT:
		case TRACE_STRAIGHT:
            LOG_INFO("Straight\n");
            StraightMove();
            break;
        case MOVE_SELECTION_TYPE_STOP:
            LOG_INFO("Stop Move\n");
			StopMove();
            break;
        case MOVE_SELECTION_TYPE_RIGHTSIFT_1:
            LOG_INFO("Right Shift 1\n");
            StraightMoveRightShift();
            break;
        case MOVE_SELECTION_TYPE_LEFTSIFT_1:
            LOG_INFO("Left Shift 1\n");
            StraightMoveLeftShift();
            break;
        case MOVE_SELECTION_TYPE_RIGHTSIFT_2:
            LOG_INFO("Right Shift 2\n");
            StraightMoveRightShift2();
            break;
        case MOVE_SELECTION_TYPE_LEFTSIFT_2:
            LOG_INFO("Left Shift 2\n");
            StraightMoveLeftShift2();
            break;
        case MOVE_SELECTION_TYPE_RIGHTTURN:
            LOG_INFO("Right Turn\n");
            TurnLowMoveRight();
            break;
        case MOVE_SELECTION_TYPE_LEFTTURN:
            LOG_INFO("Left Turn\n");
            TurnLowMoveLeft();
            break;
        case MOVE_SELECTION_TYPE_RIGHTTURN_2:
            LOG_INFO("Right Turn 2\n");
			TurnMoveRight();
            break;
        case MOVE_SELECTION_TYPE_LEFTTURN_2:
            LOG_INFO("Left Turn 2\n");
			TurnMoveLeft();
            break;
        case MOVE_SELECTION_TYPE_BACK:
            LOG_INFO("Back Move\n");
            StopMove();
            delay_time(10);
            BackMove();
            delay_time(1000);
            break;
		case MOVE_SELECTION_TYPE_STRAIGHT_2:
            LOG_INFO("Straight Low Move\n");
            StraightLowMove();
            break;
			
		case TRACE_L_STRAIGHT:
            LOG_INFO("TRACE_L_STRAIGHT\n");
            LeftStraightMove();
            break;
		case TRACE_R_STRAIGHT:
			LOG_INFO("TRACE_R_STRAIGHT\n");
			RightStraightMove();
			break;
		case TRACE_L_ROUND:
			LOG_INFO("TRACE_L_ROUND\n");
			LeftRoundMove();
			break;
		case TRACE_R_ROUND:
            LOG_INFO("TRACE_R_ROUND\n");
            RightRoundMove();
            break;
		case TRACE_L_TURN:
            LOG_INFO("TRACE_L_TURN\n");
            LeftTurnMove();
            break;	
		case TRACE_R_TURN:
            LOG_INFO("TRACE_R_TURN\n");
            RightTurnMove();
            break;
        default:
            break;
	}
}

void setParamMoveAction(int right, int left) {
	int correctionVal = 0;
    MotorControl( RIGHT_MOTOR, right );
    MotorControl( LEFT_MOTOR, left + correctionVal );
    LOG_DEBUG("RIGHT_MORTOR:%d  LEFT_MORTOR:%d \n",right, left);
}

void StopMove(void) {
    MotorControl( RIGHT_MOTOR, 1024 );
    MotorControl( LEFT_MOTOR, 0 );
}

void StraightLowMove(void) {
	MotorControl( RIGHT_MOTOR, 1098 ); //300 P_CCW_SPEED_NOMAL 1632
	MotorControl( LEFT_MOTOR, 75 ); //300 P_CW_SPEED_NOMAL 609
}

void StraightMoveRightShift(void) {
    MotorControl( RIGHT_MOTOR, 1473 );//280 P_CCW_SPEED_TURN    1532
    MotorControl( LEFT_MOTOR, 500 );//300 P_CW_SPEED_NOMAL    609
}

void StraightMoveLeftShift(void) {
    MotorControl( RIGHT_MOTOR, 1523 );//300 P_CCW_SPEED_NOMAL   1632
    MotorControl( LEFT_MOTOR, 450 );//280 P_CW_SPEED_TURN     509
}

void StraightMoveRightShift2(void) {
    MotorControl( RIGHT_MOTOR, 1423 );//250 P_CCW_SPEED_TURN_2 1432
    MotorControl( LEFT_MOTOR, 500 );//300 P_CW_SPEED_NOMAL    609
}

void StraightMoveLeftShift2(void) {
    MotorControl( RIGHT_MOTOR, 1523 ); //300 P_CCW_SPEED_NOMAL 1632
    MotorControl( LEFT_MOTOR, 400 ); //250 P_CW_SPEED_TURN_2 409
}

void TurnMoveRight(void) {
    MotorControl( RIGHT_MOTOR, 400 );//250 P_CW_SPEED_TURN 509
    MotorControl( LEFT_MOTOR, 400 );//250 P_CW_SPEED_TURN 509
}

void TurnMoveLeft(void) {
    MotorControl( RIGHT_MOTOR, 1424 );//250 P_CCW_SPEED_TURN 1532
    MotorControl( LEFT_MOTOR, 1424 );//250 P_CCW_SPEED_TURN 1532
}

void TurnLowMoveRight(void) {
    MotorControl( RIGHT_MOTOR, 300 );//200 P_CW_SPEED_TURN_2   409
    MotorControl( LEFT_MOTOR, 300 );//200 P_CW_SPEED_TURN_2 409
}

void TurnLowMoveLeft(void) {
    MotorControl( RIGHT_MOTOR, 1324 );//200 P_CCW_SPEED_TURN_2 1432
    MotorControl( LEFT_MOTOR, 1324 );//200 P_CCW_SPEED_TURN_2 1432
}

void BackMove(void) {
    MotorControl( RIGHT_MOTOR, 250 ); //250 P_CW_SPEED_TURN 509
    MotorControl( LEFT_MOTOR, 1273 ); //R250 P_CCW_SPEED_TURN 1532
}

void Move(int leftSpeed, int rightSpeed)
{
	LOG_INFO("left = %3d, Right = %3d\n", leftSpeed, rightSpeed);
    MotorControl(LEFT_MOTOR, leftSpeed);
    MotorControl(RIGHT_MOTOR, rightSpeed);
}

void StraightMove(void) {
	int leftSpeed = BaseSpeed;
	int rightSpeed = (1024 + BaseSpeed);

	Move(leftSpeed, rightSpeed);
}

void LeftStraightMove(void) {
	int leftSpeed = (int)((float)BaseSpeed * LowRate);
	int rightSpeed = (1024 + BaseSpeed);

	Move(leftSpeed, rightSpeed);
}

void RightStraightMove(void) {
	int leftSpeed = BaseSpeed;
	int rightSpeed = (1024 + (int)((float)BaseSpeed * LowRate));

	Move(leftSpeed, rightSpeed);
}

void LeftRoundMove(void) {
	int leftSpeed = (int)((float)BaseSpeed * HighRate);
	int rightSpeed = (1024 + BaseSpeed);

	Move(leftSpeed, rightSpeed);
}

void RightRoundMove(void) {
	int leftSpeed = BaseSpeed;
	int rightSpeed = (1024 + (int)((float)BaseSpeed * HighRate));

	Move(leftSpeed, rightSpeed);
}

void LeftTurnMove(void) {
	int leftSpeed = (1024 + BaseSpeed);
	int rightSpeed = (1024 + BaseSpeed);

	Move(leftSpeed, rightSpeed);
}

void RightTurnMove(void) {
	int leftSpeed = BaseSpeed;
	int rightSpeed = BaseSpeed;

	Move(leftSpeed, rightSpeed);
}

delay_time(int sec) {
	Sleep(sec);
}

