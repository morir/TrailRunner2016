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

#include "pid.h"

// ------------------ Defined ------------------
// State
#define STATE_INIT           2000
#define STATE_STOP           2001
#define STATE_START          2002
#define STATE_MOVE           2003
#define STATE_FIXED_MOVE     2004
#define STATE_TEST_MODE      3000

// Line Sensor
#define LINE_STATE_BLACK    0
#define LINE_STATE_WHITE    1

// PID Param
#define K_P     0.70     // P param 1.00
#define K_I     0.50    // I param 5.00
#define K_D     0.50  // D param 0.0003
//#define pid_base 500   // base speed P_CW_SPEED_NOMAL 500
//#define pid_base 440   // base speed P_CW_SPEED_NOMAL 500
#define pid_base 500   // base speed P_CW_SPEED_NOMAL 500
#define pid_lim 30     // control value 30
#define DELTA_T 0.002   // delta T 0.002
#define offSet_val 250 //450;//300

//// Search Maximuim count
//#define SEARCH_MAX_COUNT 200

// START SW address
#define SW_START 0x01   // Emergency Stop

//#define MAX_STOP_COUNT 20
#define MAX_STOP_COUNT 80

// Debug Log ON/OFF
#define DBG 1

#define _LED_ON_

// ------------------ Method Definition ------------------
void split( char * s1 );
void initMoveAction(void);
int decideMoveAction(void);
int getAction(void);

int isSensor_000000(int*);
int isSensor_000001(int*);
int isSensor_000010(int*);
int isSensor_000011(int*);
int isSensor_000100(int*);
int isSensor_000101(int*);
int isSensor_000110(int*);
int isSensor_000111(int*);
int isSensor_001000(int*);
int isSensor_001001(int*);
int isSensor_001010(int*);
int isSensor_001011(int*);
int isSensor_001100(int*);
int isSensor_001101(int*);
int isSensor_001110(int*);
int isSensor_001111(int*);
int isSensor_010000(int*);
int isSensor_010001(int*);
int isSensor_010010(int*);
int isSensor_010011(int*);
int isSensor_010100(int*);
int isSensor_010101(int*);
int isSensor_010110(int*);
int isSensor_010111(int*);
int isSensor_011000(int*);
int isSensor_011001(int*);
int isSensor_011010(int*);
int isSensor_011011(int*);
int isSensor_011100(int*);
int isSensor_011101(int*);
int isSensor_011110(int*);
int isSensor_011111(int*);
int isSensor_100000(int*);
int isSensor_100001(int*);
int isSensor_100010(int*);
int isSensor_100011(int*);
int isSensor_100100(int*);
int isSensor_100101(int*);
int isSensor_100110(int*);
int isSensor_100111(int*);
int isSensor_101000(int*);
int isSensor_101001(int*);
int isSensor_101010(int*);
int isSensor_101011(int*);
int isSensor_101100(int*);
int isSensor_101101(int*);
int isSensor_101110(int*);
int isSensor_101111(int*);
int isSensor_110000(int*);
int isSensor_110001(int*);
int isSensor_110010(int*);
int isSensor_110011(int*);
int isSensor_110100(int*);
int isSensor_110101(int*);
int isSensor_110110(int*);
int isSensor_110111(int*);
int isSensor_111000(int*);
int isSensor_111001(int*);
int isSensor_111010(int*);
int isSensor_111011(int*);
int isSensor_111100(int*);
int isSensor_111101(int*);
int isSensor_111110(int*);
int isSensor_111111(int*);

void getSensors(void);
int getState(void);
void setState(int state);

void updateAction(int pre_state, int change_state);

void executeFinalAction(void);

void FixedMotionACtion(void);
void nextMoveAction(int action);

void initEmergencyStop(void);

void setLED(void);
void LED_on(int i);
void LED_off(int i);

int PID_2(int target_val, int sencer_val);

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

int mMoveCount = 0;

//int mSearchCount = 0;

// PID Param
float pGain = 200;   //Proportional Gain
float iGain =  0.2;  //Integral Gain
float dGain =  120;  //Differential Gain
int delay = 10;
int32_t eInteg = 0;  //Integral accumulator
int32_t ePrev  =0;   //Previous Error

int diff[2]    = {0,0};
int ret_val[2] = {0,0};
float integral = 0.0;
int target_senser = 0;

// ------------------ Method ------------------

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
                printf( "readData=%s\n", &readData[0] );
                split( &readData[0] );
                switch( serCmd[0] ) {
                case 1:
                    //MotorControl( 2, serCmd[1] );
                    //setParamMoveAction(serCmd[1], serCmd[2]);
                    break;
                case 999:
                    printf( "finish\n");
                    isFinish = 1;
                    break;
                }
                if( isFinish > 0 ) {
                    //MotorControl( 0, 0 );
                    //setMoveAction(MOVE_SELECTION_TYPE_STOP);
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
                
            case STATE_FIXED_MOVE:
                FixedMotionACtion();
                break;
                
            case STATE_TEST_MODE:
                Execute(MOVE_SELECTION_TYPE_S_MOVE_10);
                break;
            
            default:
                setState(STATE_INIT);
                break;
            }
            //_delay_ms(DELAY_MSEC);
//            _delay_us(50);
//            _delay_us(100);
//            _delay_ms(500);//DBG
        }
    }
}

void split( char * s1 ) {
    char s2[] = " ,";
    char *tok;
    int cnt = 0;

    tok = strtok( s1, s2 );
    while( tok != NULL ){
//      printf( "%s?n", tok );
        serCmd[cnt++] = atoi(tok);
        tok = strtok( NULL, s2 );  /* 2回目以降 */
    }
}

//最初の動作
void initMoveAction(void) {
//    Execute(MOVE_SELECTION_TYPE_LEFTTURN);//左回転
//    _delay_ms(1000);    // 1s
    Execute(MOVE_SELECTION_TYPE_STOP);//次の動作：停止
}

int decideMoveAction(void) {
    int ret_state = STATE_MOVE;
	
    getSensors();
    int currentAction = mCurrentAction;
	
    int nextAction;
//    int search_count = 0;

//	pre_move = MOVE_SELECTION_TYPE_STOP;//DBG
    switch (currentAction) {
    case MOVE_SELECTION_TYPE_START:
    case MOVE_SELECTION_TYPE_STOP:
        Execute(currentAction);
        if (currentAction == MOVE_SELECTION_TYPE_START) {
            nextAction = MOVE_SELECTION_TYPE_STRAIGHT;
        } else {
            nextAction = MOVE_SELECTION_TYPE_START;
            diff[0] = 0;
            diff[1] = 0;
            integral = 0;
        }
        updateAction(currentAction, nextAction);
        break;

    case MOVE_SELECTION_TYPE_STRAIGHT:
    case MOVE_SELECTION_TYPE_RIGHTSIFT_1:
    case MOVE_SELECTION_TYPE_RIGHTSIFT_2:
    case MOVE_SELECTION_TYPE_LEFTSIFT_1:
    case MOVE_SELECTION_TYPE_LEFTSIFT_2:
        PID_2(0, target_senser);
        setParamMoveAction(ret_val[0], ret_val[1]);

        getSensors();
        nextAction = getAction();

        updateAction(currentAction, nextAction);
        break;

    case MOVE_SELECTION_TYPE_BACK:
        //LED_on(1);
        //LED_on(2);
        //LED_on(3);
        //LED_on(4);
        //LED_on(5);
        //LED_on(6);
        Execute(currentAction);
        getSensors();
        nextAction = getAction();
        
        updateAction(currentAction, nextAction);
        break;
        
    case MOVE_SELECTION_TYPE_RIGHTTURN:
    case MOVE_SELECTION_TYPE_RIGHTTURN_3:
        LED_on(1);
//        Execute(currentAction);
        Execute(MOVE_SELECTION_TYPE_RIGHTTURN);
		if (currentAction == MOVE_SELECTION_TYPE_RIGHTTURN) {
			_delay_ms(150);
		}
//        search_count = 0;
        while(1) {
            getSensors();
            nextAction = getAction();
			if (nextAction == MOVE_SELECTION_TYPE_STRAIGHT ||
                nextAction == MOVE_SELECTION_TYPE_RIGHTSIFT_1 ||
                nextAction == MOVE_SELECTION_TYPE_RIGHTSIFT_2 ||
                nextAction == MOVE_SELECTION_TYPE_LEFTSIFT_1 ||
                nextAction == MOVE_SELECTION_TYPE_LEFTSIFT_2)
			{
				diff[0] = 0;
                diff[1] = 0;
                integral = 0;
				Execute(MOVE_SELECTION_TYPE_LEFTTURN);
				if (currentAction == MOVE_SELECTION_TYPE_RIGHTTURN) {
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
				LED_on(1);
				Execute(MOVE_SELECTION_TYPE_RIGHTTURN);
				_delay_ms(10);
			}
        }
//        LED_on(1);
        break;
        
    case MOVE_SELECTION_TYPE_LEFTTURN:
    case MOVE_SELECTION_TYPE_LEFTTURN_3:
        LED_on(5);
//        Execute(currentAction);
        Execute(MOVE_SELECTION_TYPE_LEFTTURN);
		if (currentAction == MOVE_SELECTION_TYPE_LEFTTURN) {
			_delay_ms(150);
		}
//        search_count = 0;
        while(1) {
            getSensors();
            nextAction = getAction();
			if (nextAction == MOVE_SELECTION_TYPE_STRAIGHT ||
                nextAction == MOVE_SELECTION_TYPE_LEFTSIFT_1 ||
                nextAction == MOVE_SELECTION_TYPE_LEFTSIFT_2 ||
                nextAction == MOVE_SELECTION_TYPE_RIGHTSIFT_1 ||
                nextAction == MOVE_SELECTION_TYPE_RIGHTSIFT_2)
			{
				diff[0] = 0;
				diff[1] = 0;
				integral = 0;
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
//        LED_on(2);
        break;

    case MOVE_SELECTION_TYPE_SEARCH:
        printf("Search move\r\n");
        LED_on(3);
        LED_on(6);
		//Execute(currentAction);
		getSensors();
        nextAction = getAction();
        if (nextAction != MOVE_SELECTION_TYPE_SEARCH ) {
            // next state
        } else {
            
            switch (mBeforeMoveState) {
                case MOVE_SELECTION_TYPE_STRAIGHT:
                    nextAction = MOVE_SELECTION_TYPE_BACK;
                    break;
                case MOVE_SELECTION_TYPE_RIGHTSIFT_1:
                case MOVE_SELECTION_TYPE_RIGHTSIFT_2:
                    nextAction = MOVE_SELECTION_TYPE_RIGHTTURN;
                    break;
                case MOVE_SELECTION_TYPE_LEFTSIFT_1:
                case MOVE_SELECTION_TYPE_LEFTSIFT_2:
                    nextAction = MOVE_SELECTION_TYPE_LEFTTURN;
                    break;
                case MOVE_SELECTION_TYPE_RIGHTTURN:
                    nextAction = MOVE_SELECTION_TYPE_LEFTTURN;
                    break;
                case MOVE_SELECTION_TYPE_LEFTTURN:
                    nextAction = MOVE_SELECTION_TYPE_RIGHTTURN;
                    break;
                case MOVE_SELECTION_TYPE_SEARCH:
                    nextAction = MOVE_SELECTION_TYPE_SEARCH;
                    break;
                default:
                    nextAction = MOVE_SELECTION_TYPE_LEFTTURN;
                    break;
            }
            
        }
        if (nextAction == MOVE_SELECTION_TYPE_SEARCH) {
            LED_on(3);
            LED_on(6);
        }
        updateAction(currentAction, nextAction);
        break;

	case MOVE_SELECTION_TYPE_STRAIGHT_2:
//		LED_on(1);
//		LED_on(4);
		//LED_on(3);
		//LED_on(6);
        LED_on(1);
        LED_on(2);
        LED_on(3);
        LED_on(4);
        LED_on(5);
        LED_on(6);
		Execute(currentAction);
		getSensors();
		nextAction = getAction();
		updateAction(currentAction, nextAction);
		break;

    case MOVE_SELECTION_TYPE_S_MOVE_1:
        // 1 1 0 0 1 1
        // 1 0 0 0 1 1
        // 1 1 0 0 0 1
        // 1 0 0 0 0 1
        printf("Special Move 1\r\n");
        nextAction = getAction();
        updateAction(currentAction, nextAction);
        break;
    case MOVE_SELECTION_TYPE_S_MOVE_2:
        // 0 0 0 0 0 1
//        LED_on(4);
        printf("Special Move 2\r\n");
        nextAction = getAction();
        updateAction(currentAction, nextAction);
        break;
    case MOVE_SELECTION_TYPE_S_MOVE_3:
        // 1 0 0 0 0 0
//        LED_on(5);
        printf("Special Move 3\r\n");
        nextAction = getAction();
        updateAction(currentAction, nextAction);
        break;
    case MOVE_SELECTION_TYPE_S_MOVE_4:
        // 0 1 1 1 1 0
        // 0 0 1 1 1 0
        // 0 1 0 1 1 0
        // 0 1 1 0 1 0
        // 0 0 1 0 1 0
        // 0 1 0 0 1 0
        // 0 1 1 1 0 0
        // 0 1 0 1 0 0
//        LED_on(6);
        printf( "Special Move 4\r\n" );
        nextAction = getAction();
        updateAction(currentAction, nextAction);
        break;
    default:
        updateAction(0, MOVE_SELECTION_TYPE_STOP);
        ret_state = STATE_STOP;
        break;
    }

    return ret_state;
}

void getSensors(void) {
    ReadIRSensors(IR);

    printf("sensor %3d: %3d: %3d: %3d: %3d: %3d \r\n", IR[4], IR[1], IR[5], IR[2], IR[6], IR[3]);
}

int getState(void) {
    return mState;
}

void setState(int state) {
    mState = state;
}

void updateAction(int pre_state, int change_state) {
    mBeforeMoveState = pre_state;
    if (pre_state != change_state) {
        mCurrentAction = change_state;
    }
}

int getAction(void) {
    int ret = 0;
    
    int i_state[ADC_PORT_6 + 1];
    
    int dbgFlag = 1;//DBG
    for (int i = ADC_PORT_1; i <= ADC_PORT_6; i++) {
        if ( IR[i] >= COMPARE_VALUE ) {
            i_state[i] = LINE_STATE_BLACK;
        } else {
            i_state[i] = LINE_STATE_WHITE;
        }
    }

	setLED();
    // Right 3
    // Center Right_2 6
    // Center Right_1 2
    // Center Left_1  5
    // Center Left_2  1
    // Left  4

	if (isSensor_111111(i_state) == TRUE) {
		// 1 1 1 1 1 1
        LED_on(2);
        LED_on(5);
		ret = MOVE_SELECTION_TYPE_SEARCH;
		if (dbgFlag) printf("1 1 1 1 1 1 \r\n");
	}
	else if (isSensor_101111(i_state) == TRUE) {
		// 1 0 1 1 1 1
//		ret = MOVE_SELECTION_TYPE_RIGHTTURN;
		ret = MOVE_SELECTION_TYPE_SEARCH;
		if (dbgFlag) printf("1 0 1 1 1 1 \r\n");
	}
	else if (isSensor_110111(i_state) == TRUE) {
		// 1 1 0 1 1 1
		ret = MOVE_SELECTION_TYPE_SEARCH;
		if (dbgFlag) printf("1 1 0 1 1 1 \r\n");
	}
	else if (isSensor_100111(i_state) == TRUE) {
		// 1 0 0 1 1 1
//		ret = MOVE_SELECTION_TYPE_RIGHTTURN;
		ret = MOVE_SELECTION_TYPE_SEARCH;
		if (dbgFlag) printf("1 0 0 1 1 1 \r\n");
	}
	else if (isSensor_111011(i_state) == TRUE) {
		// 1 1 1 0 1 1
		ret = MOVE_SELECTION_TYPE_SEARCH;
		if (dbgFlag) printf("1 1 1 0 1 1 \r\n");
	}
	else if (isSensor_101011(i_state) == TRUE) {
		// 1 0 1 0 1 1
		ret = MOVE_SELECTION_TYPE_SEARCH;
		if (dbgFlag) printf("1 0 1 0 1 1 \r\n");
	}
	else if (isSensor_110011(i_state) == TRUE) {
		// 1 1 0 0 1 1
		ret = MOVE_SELECTION_TYPE_S_MOVE_1;
		if (dbgFlag) printf("1 1 0 0 1 1 \r\n");
	}
	else if (isSensor_100011(i_state) == TRUE) {
		// 1 0 0 0 1 1
		ret = MOVE_SELECTION_TYPE_S_MOVE_1;
		if (dbgFlag) printf("1 0 0 0 1 1 \r\n");
	}
	else if (isSensor_111101(i_state) == TRUE) {
		// 1 1 1 1 0 1
//		ret = MOVE_SELECTION_TYPE_LEFTTURN;
		ret = MOVE_SELECTION_TYPE_SEARCH;
		if (dbgFlag) printf("1 1 1 1 0 1 \r\n");
	}
	else if (isSensor_101101(i_state) == TRUE) {
		// 1 0 1 1 0 1
		ret = MOVE_SELECTION_TYPE_SEARCH;
		if (dbgFlag) printf("1 0 1 1 0 1 \r\n");
	}
	else if (isSensor_110101(i_state) == TRUE) {
		// 1 1 0 1 0 1
		ret = MOVE_SELECTION_TYPE_SEARCH;
		if (dbgFlag) printf("1 1 0 1 0 1 \r\n");
	}
	else if (isSensor_100101(i_state) == TRUE) {
		// 1 0 0 1 0 1
		ret = MOVE_SELECTION_TYPE_SEARCH;
		if (dbgFlag) printf("1 0 0 1 0 1 \r\n");
	}
	else if (isSensor_111001(i_state) == TRUE) {
		// 1 1 1 0 0 1
//		ret = MOVE_SELECTION_TYPE_LEFTTURN;
		ret = MOVE_SELECTION_TYPE_SEARCH;
		if (dbgFlag) printf("1 1 1 0 0 1 \r\n");
	}
	else if (isSensor_101001(i_state) == TRUE) {
		// 1 0 1 0 0 1
		ret = MOVE_SELECTION_TYPE_SEARCH;
		if (dbgFlag) printf("1 0 1 0 0 1 \r\n");
	}
	else if (isSensor_110001(i_state) == TRUE) {
		// 1 1 0 0 0 1
		ret = MOVE_SELECTION_TYPE_S_MOVE_1;
		if (dbgFlag) printf("1 1 0 0 0 1 \r\n");
	}
	else if (isSensor_100001(i_state) == TRUE) {
		// 1 0 0 0 0 1
		ret = MOVE_SELECTION_TYPE_S_MOVE_1;
		if (dbgFlag) printf("1 0 0 0 0 1 \r\n");
	}
	else if (isSensor_011111(i_state) == TRUE) {
		// 0 1 1 1 1 1
		ret = MOVE_SELECTION_TYPE_RIGHTTURN;
		if (dbgFlag) printf("0 1 1 1 1 1 \r\n");
	}
	else if (isSensor_001111(i_state) == TRUE) {
		// 0 0 1 1 1 1
		ret = MOVE_SELECTION_TYPE_RIGHTTURN;
		if (dbgFlag) printf("0 0 1 1 1 1 \r\n");
	}
	else if (isSensor_010111(i_state) == TRUE) {
		// 0 1 0 1 1 1
//		ret = MOVE_SELECTION_TYPE_RIGHTTURN;
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		target_senser = 9;
		if (dbgFlag) printf("0 1 0 1 1 1 \r\n");
	}
	else if (isSensor_000111(i_state) == TRUE) {
		// 0 0 0 1 1 1
		ret = MOVE_SELECTION_TYPE_RIGHTTURN;
		if (dbgFlag) printf("0 0 0 1 1 1 \r\n");
	}
	else if (isSensor_011011(i_state) == TRUE) {
		// 0 1 1 0 1 1
//		ret = MOVE_SELECTION_TYPE_RIGHTTURN;
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		target_senser = 9;
		if (dbgFlag) printf("0 1 1 0 1 1 \r\n");
	}
	else if (isSensor_001011(i_state) == TRUE) {
		// 0 0 1 0 1 1
//		ret = MOVE_SELECTION_TYPE_RIGHTTURN;
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		target_senser = 9;
		if (dbgFlag) printf("0 0 1 0 1 1 \r\n");
	}
	else if (isSensor_010011(i_state) == TRUE) {
		// 0 1 0 0 1 1
//		ret = MOVE_SELECTION_TYPE_RIGHTTURN;
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		target_senser = 9;
		if (dbgFlag) printf("0 1 0 0 1 1 \r\n");
	}
	else if (isSensor_000011(i_state) == TRUE) {
		// 0 0 0 0 1 1
		ret = MOVE_SELECTION_TYPE_RIGHTTURN_3;
//		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		target_senser = 9;
		if (dbgFlag) printf("0 0 0 0 1 1 \r\n");
	}
	else if (isSensor_011101(i_state) == TRUE) {
		// 0 1 1 1 0 1
//		ret = MOVE_SELECTION_TYPE_RIGHTTURN;
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		target_senser = 9;
		if (dbgFlag) printf("0 1 1 1 0 1 \r\n");
	}
	else if (isSensor_001101(i_state) == TRUE) {
		// 0 0 1 1 0 1
//		ret = MOVE_SELECTION_TYPE_RIGHTTURN;
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		target_senser = 9;
		if (dbgFlag) printf("0 0 1 1 0 1 \r\n");
	}
	else if (isSensor_010101(i_state) == TRUE) {
		// 0 1 0 1 0 1
//		ret = MOVE_SELECTION_TYPE_RIGHTTURN;
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		target_senser = 9;
		if (dbgFlag) printf("0 1 0 1 0 1 \r\n");
	}
	else if (isSensor_000101(i_state) == TRUE) {
		// 0 0 0 1 0 1
//		ret = MOVE_SELECTION_TYPE_RIGHTTURN;
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		target_senser = 9;
		if (dbgFlag) printf("0 0 0 1 0 1 \r\n");
	}
	else if (isSensor_011001(i_state) == TRUE) {
		// 0 1 1 0 0 1
//		ret = MOVE_SELECTION_TYPE_RIGHTTURN;
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		target_senser = 9;
		if (dbgFlag) printf("0 1 1 0 0 1 \r\n");
	}
	else if (isSensor_001001(i_state) == TRUE) {
		// 0 0 1 0 0 1
//		ret = MOVE_SELECTION_TYPE_RIGHTTURN;
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		target_senser = 9;
		if (dbgFlag) printf("0 0 1 0 0 1 \r\n");
	}
	else if (isSensor_010001(i_state) == TRUE) {
		// 0 1 0 0 0 1
//		ret = MOVE_SELECTION_TYPE_RIGHTTURN;
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		target_senser = 9;
		if (dbgFlag) printf("0 1 0 0 0 1 \r\n");
	}
	else if (isSensor_000001(i_state) == TRUE) {
		// 0 0 0 0 0 1
		ret = MOVE_SELECTION_TYPE_RIGHTTURN_3;
//		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		target_senser = 9;
		if (dbgFlag) printf("0 0 0 0 0 1 \r\n");
	}
	else if (isSensor_111110(i_state) == TRUE) {
		// 1 1 1 1 1 0
		ret = MOVE_SELECTION_TYPE_LEFTTURN;
		if (dbgFlag) printf("1 1 1 1 1 0 \r\n");
	}
	else if (isSensor_101110(i_state) == TRUE) {
		// 1 0 1 1 1 0
//		ret = MOVE_SELECTION_TYPE_LEFTTURN;
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		target_senser = -9;
		if (dbgFlag) printf("1 0 1 1 1 0 \r\n");
	}
	else if (isSensor_110110(i_state) == TRUE) {
		// 1 1 0 1 1 0
//		ret = MOVE_SELECTION_TYPE_LEFTTURN;
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		target_senser = -9;
		if (dbgFlag) printf("1 1 0 1 1 0 \r\n");
	}
	else if (isSensor_100110(i_state) == TRUE) {
		// 1 0 0 1 1 0
//		ret = MOVE_SELECTION_TYPE_LEFTTURN;
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		target_senser = -9;
		if (dbgFlag) printf("1 0 0 1 1 0 \r\n");
	}
	else if (isSensor_111010(i_state) == TRUE) {
		// 1 1 1 0 1 0
//		ret = MOVE_SELECTION_TYPE_LEFTTURN;
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		target_senser = -9;
		if (dbgFlag) printf("1 1 1 0 1 0 \r\n");
	}
	else if (isSensor_101010(i_state) == TRUE) {
		// 1 0 1 0 1 0
//		ret = MOVE_SELECTION_TYPE_LEFTTURN;
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		target_senser = -9;
		if (dbgFlag) printf("1 0 1 0 1 0 \r\n");
	}
	else if (isSensor_110010(i_state) == TRUE) {
		// 1 1 0 0 1 0
//		ret = MOVE_SELECTION_TYPE_LEFTTURN;
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		target_senser = -9;
		if (dbgFlag) printf("1 1 0 0 1 0 \r\n");
	}
	else if (isSensor_100010(i_state) == TRUE) {
		// 1 0 0 0 1 0
//		ret = MOVE_SELECTION_TYPE_LEFTTURN;
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		target_senser = -9;
		if (dbgFlag) printf("1 0 0 0 1 0 \r\n");
	}
	else if (isSensor_111100(i_state) == TRUE) {
		// 1 1 1 1 0 0
		ret = MOVE_SELECTION_TYPE_LEFTTURN;
		if (dbgFlag) printf("1 1 1 1 0 0 \r\n");
	}
	else if (isSensor_101100(i_state) == TRUE) {
		// 1 0 1 1 0 0
//		ret = MOVE_SELECTION_TYPE_LEFTTURN;
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		target_senser = -9;
		if (dbgFlag) printf("1 0 1 1 0 0 \r\n");
	}
	else if (isSensor_110100(i_state) == TRUE) {
		// 1 1 0 1 0 0
//		ret = MOVE_SELECTION_TYPE_LEFTTURN;
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		target_senser = -9;
		if (dbgFlag) printf("1 1 0 1 0 0 \r\n");
	}
	else if (isSensor_100100(i_state) == TRUE) {
		// 1 0 0 1 0 0
//		ret = MOVE_SELECTION_TYPE_LEFTTURN;
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		target_senser = -9;
		if (dbgFlag) printf("1 0 0 1 0 0 \r\n");
	}
	else if (isSensor_111000(i_state) == TRUE) {
		// 1 1 1 0 0 0
		ret = MOVE_SELECTION_TYPE_LEFTTURN;
		if (dbgFlag) printf("1 1 1 0 0 0 \r\n");
	}
	else if (isSensor_101000(i_state) == TRUE) {
		// 1 0 1 0 0 0
//		ret = MOVE_SELECTION_TYPE_LEFTTURN;
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		target_senser = -9;
		if (dbgFlag) printf("1 0 1 0 0 0 \r\n");
	}
	else if (isSensor_110000(i_state) == TRUE) {
		// 1 1 0 0 0 0
		ret = MOVE_SELECTION_TYPE_LEFTTURN_3;
//		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		target_senser = -9;
		if (dbgFlag) printf("1 1 0 0 0 0 \r\n");
	}
	else if (isSensor_100000(i_state) == TRUE) {
		// 1 0 0 0 0 0
		ret = MOVE_SELECTION_TYPE_LEFTTURN_3;
//		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		target_senser = -9;
		if (dbgFlag) printf("1 0 0 0 0 0 \r\n");
	}
	else if (isSensor_011110(i_state) == TRUE) {
		// 0 1 1 1 1 0
		ret = MOVE_SELECTION_TYPE_STRAIGHT;
		if (dbgFlag) printf("0 1 1 1 1 0 \r\n");
	}
	else if (isSensor_001110(i_state) == TRUE) {
		// 0 0 1 1 1 0
		ret = MOVE_SELECTION_TYPE_S_MOVE_4;
		if (dbgFlag) printf("0 0 1 1 1 0 \r\n");
	}
	else if (isSensor_010110(i_state) == TRUE) {
		// 0 1 0 1 1 0
		ret = MOVE_SELECTION_TYPE_S_MOVE_4;
		if (dbgFlag) printf("0 1 0 1 1 0 \r\n");
	}
	else if (isSensor_000110(i_state) == TRUE) {
		// 0 0 0 1 1 0
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_1;
		target_senser = 3;//2
		if (dbgFlag) printf("0 0 0 1 1 0 \r\n");
	}
	else if (isSensor_011010(i_state) == TRUE) {
		// 0 1 1 0 1 0
		ret = MOVE_SELECTION_TYPE_S_MOVE_4;
		if (dbgFlag) printf("0 1 1 0 1 0 \r\n");
	}
	else if (isSensor_001010(i_state) == TRUE) {
		// 0 0 1 0 1 0
		ret = MOVE_SELECTION_TYPE_S_MOVE_4;
		if (dbgFlag) printf("0 0 1 0 1 0 \r\n");
	}
	else if (isSensor_010010(i_state) == TRUE) {
		// 0 1 0 0 1 0
		ret = MOVE_SELECTION_TYPE_S_MOVE_4;
		if (dbgFlag) printf("0 1 0 0 1 0 \r\n");
	}
	else if (isSensor_000010(i_state) == TRUE) {
		// 0 0 0 0 1 0
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_2;
		target_senser = 5;//3
		if (dbgFlag) printf("0 0 0 0 1 0 \r\n");
	}
	else if (isSensor_011100(i_state) == TRUE) {
		// 0 1 1 1 0 0
		ret = MOVE_SELECTION_TYPE_S_MOVE_4;
		if (dbgFlag) printf("0 1 1 1 0 0 \r\n");
	}
	else if (isSensor_001100(i_state) == TRUE) {
		// 0 0 1 1 0 0
		ret = MOVE_SELECTION_TYPE_STRAIGHT;
		if (dbgFlag) printf("0 0 1 1 0 0 \r\n");
	}
	else if (isSensor_010100(i_state) == TRUE) {
		// 0 1 0 1 0 0
		ret = MOVE_SELECTION_TYPE_S_MOVE_4;
		if (dbgFlag) printf("0 1 0 1 0 0 \r\n");
	}
	else if (isSensor_000100(i_state) == TRUE) {
		// 0 0 0 1 0 0
		ret = MOVE_SELECTION_TYPE_RIGHTSIFT_1;
		target_senser = 1;
		if (dbgFlag) printf("0 0 0 1 0 0 \r\n");
	}
	else if (isSensor_011000(i_state) == TRUE) {
		// 0 1 1 0 0 0
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_1;
		target_senser = -3;//-2
		if (dbgFlag) printf("0 1 1 0 0 0 \r\n");
	}
	else if (isSensor_001000(i_state) == TRUE) {
		// 0 0 1 0 0 0
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_1;
		target_senser = -1;
		if (dbgFlag) printf("0 0 1 0 0 0 \r\n");
	}
	else if (isSensor_010000(i_state) == TRUE) {
		// 0 1 0 0 0 0
		ret = MOVE_SELECTION_TYPE_LEFTSIFT_2;
		target_senser = -5;//-3
		if (dbgFlag) printf("0 1 0 0 0 0 \r\n");
	}
	else if (isSensor_000000(i_state) == TRUE) {
		// 0 0 0 0 0 0
//        LED_on(1);
//        LED_on(4);
		ret = MOVE_SELECTION_TYPE_STRAIGHT_2;
		if(dbgFlag) printf("0 0 0 0 0 0 \r\n");
	}
    return ret;
}

int isSensor_000000(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_000001(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_000010(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_000011(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_000100(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_000101(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_000110(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_000111(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_001000(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_001001(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_001010(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_001011(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_001100(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_001101(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_001110(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_001111(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_010000(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_010001(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_010010(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_010011(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_010100(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_010101(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_010110(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_010111(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_011000(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_011001(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_011010(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_011011(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_011100(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_011101(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_011110(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_011111(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_100000(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_100001(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_100010(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_100011(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_100100(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_100101(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_100110(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_100111(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_101000(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_101001(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_101010(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_101011(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_101100(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_101101(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_101110(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_101111(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_110000(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_110001(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_110010(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_110011(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_110100(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_110101(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_110110(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_110111(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_111000(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_111001(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_111010(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_111011(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_111100(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_111101(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_WHITE) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

int isSensor_111110(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_WHITE);
}

int isSensor_111111(int *sensor)
{
	return
		(sensor[ADC_PORT_4] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_1] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_5] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_2] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_6] == LINE_STATE_BLACK) &&
		(sensor[ADC_PORT_3] == LINE_STATE_BLACK);
}

void executeFinalAction(void)
{
	printf("executeFinalAction!!\r\n");

	//_delay_ms(50);
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

void FixedMotionACtion(void) {
    int next_state;
    int timer = 0;
    
    const int count = mMoveCount;
    if (count > MAX_COUNT) {
        return;
    }
    
    switch (count) {
        case 0:
        case 6:
        case 8:
        case 12:
        case 14:
            // Straight to Left Turn
            if (DBG) printf("Straight Move %d\r\n", count);
            //StraightMove();
            Execute(MOVE_SELECTION_TYPE_STRAIGHT);
            nextMoveAction(MOVE_SELECTION_TYPE_LEFTTURN);
            break;
        
        case 1:
        case 7:
        case 9:
        case 13:
        case 15:
            // Left Turn to Straight
            if (DBG) printf("Left Turn %d\r\n", count);
            //TurnMoveLeft();
            Execute(MOVE_SELECTION_TYPE_LEFTTURN);
            nextMoveAction(MOVE_SELECTION_TYPE_STRAIGHT);
            break;
        
        case 2:
        case 4:
        case 10:
        case 18:
            // Straight to Right Turn
            if (DBG) printf("Straight Move %d\r\n", count);
            //StraightMove();
            Execute(MOVE_SELECTION_TYPE_STRAIGHT);
            nextMoveAction(MOVE_SELECTION_TYPE_RIGHTTURN);
            break;
        
        case 3:
        case 5:
        case 11:
        case 17:
        case 19:
            // Right Turn to Straight
            if (DBG) printf("Right Turn %d\r\n", count);
            //TurnMoveRight();
            Execute(MOVE_SELECTION_TYPE_RIGHTTURN);
            nextMoveAction(MOVE_SELECTION_TYPE_STRAIGHT);
            break;
        
        case 16:
            // Long Straight to Right Turn
            if (DBG) printf("Straight Move %d\r\n", count);
            //StraightMove();
            Execute(MOVE_SELECTION_TYPE_STRAIGHT);
            while(1) {
                _delay_ms(10);
                getSensors();
                next_state = getAction();
                if ( next_state ==  MOVE_SELECTION_TYPE_RIGHTTURN ) {
                    mMoveCount++;
                    break;
                }
                timer++;
                if (timer > 50000) {
                    // 100s next
                    mMoveCount++;
                    break;
                }
            }
            break;
        
        case 20:
        case 22:
        case 24:
        case 26:
            // Straight to Bifurcate
            if (DBG) printf("Straight Move %d\r\n", count);
            //StraightMove();
            Execute(MOVE_SELECTION_TYPE_STRAIGHT);
            while(1) {
                _delay_ms(10);
                getSensors();
                next_state = getAction();
                if ( next_state ==  MOVE_SELECTION_TYPE_RIGHTTURN ||
                     next_state ==  MOVE_SELECTION_TYPE_LEFTTURN ) {
                    mMoveCount++;
                    break;
                }
                timer++;
                if (timer > 5000) {
                    // 10s next
                    mMoveCount++;
                    break;
                }
            }
            break;
        
        case 21:
        case 23:
        case 25:
        case 27:
            // Bifurcate to Straight
            next_state = getAction();
            if ( next_state ==  MOVE_SELECTION_TYPE_RIGHTTURN ) {
                if (DBG) printf("Right Turn %d\r\n", count);
                //TurnMoveRight();
                Execute(MOVE_SELECTION_TYPE_RIGHTTURN);
            } else if ( next_state ==  MOVE_SELECTION_TYPE_LEFTTURN ) {
                if (DBG) printf("Left Turn %d\r\n", count);
                //TurnMoveLeft();
                Execute(MOVE_SELECTION_TYPE_LEFTTURN);
            }
            nextMoveAction(MOVE_SELECTION_TYPE_STRAIGHT);
            break;

        case 28:
            // Last Spurt
            if (DBG) printf("Straight Move %d\r\n", count);
            //StraightMove();
            Execute(MOVE_SELECTION_TYPE_STRAIGHT);
            nextMoveAction(MOVE_SELECTION_TYPE_STOP);
            break;

        case 29:
        default:
            if (DBG) printf("STOP !! %d\r\n", count);
            Execute(MOVE_SELECTION_TYPE_STOP);
            break;
    }
}

void nextMoveAction(int action) {
    int next_state;
    int timeout = 0;
    while(1) {
        _delay_ms(10);
        getSensors();
        next_state = getAction();
        if ( next_state ==  action ) {
            mMoveCount++;
            break;
        }
        timeout++;
        if (timeout > 2000) {//5000
            // 1s next
            mMoveCount++;
            break;
        }
    }
}

struct PID_DATA pidData;

void PID_init(void);
void PID_init(void) {
    pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR , K_D * SCALING_FACTOR , &pidData); 
}

float PID(float cur_value, float req_value) {
    float pid;
    float error;
    
    error = req_value - cur_value;
    pid = (pGain * error)  + (iGain * eInteg) + (dGain * (error - ePrev));

    eInteg += error;        // integral is simply a summation over time
    ePrev = error;          // save previous for derivative

    return pid;
}

int PID_2(int target_val, int sencer_val) {
    float p,i,d;
    
    diff[0] = diff[1];
    diff[1] = sencer_val - target_val;
    
    integral += ((diff[0] + diff[1]) / 2.0 * DELTA_T);
    
    p = K_P * diff[1];
    i = K_I * integral;
    d = K_D * ((diff[1] - diff[0]) / DELTA_T);
    
    int MV = (int)(pid_lim * (p + i + d));
    
    printf( "MV = %d\r\n", MV );
    
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
    
    ret_val[0] = right_val + 1023;
    ret_val[1] = left_val;
    
    return 0;
}

void DelayMs(uint8_t ms) {
    uint8_t i;
    for( i = 0; i < ms; i++ ) {
        _delay_ms(1);
    }
}

void MainLog(char * msg) {
    if (DBG) {
        printf("Main : %s \r\n", msg);
    }
}

void initEmergencyStop(void) {
    DDRD  = 0x70;
    PORTD = 0x11;
}

void setLED(void) {
#ifdef _LED_ON_
    DDRC  = 0x7F;
    PORTC = 0x7F;
#endif // _LED_ON_
}

void LED_on(int i) {
#ifdef _LED_ON_
    if (i < 1 || i > 6) return;
	unsigned char c = PORTC;
    PORTC = c^(1<<i);
#endif // _LED_ON_
}

void LED_off(int i) {
#ifdef _LED_ON_
    if (i < 1 || i > 6) return;
	unsigned char c = PORTC;
    PORTC = ~(c^(1<<i));
#endif // _LED_ON_
}
