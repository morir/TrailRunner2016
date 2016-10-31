/*
 * MotorManager.c
 *
 * Created: 2014/10/29 14:46:23
 *  Author: Administrator
 */ 
#include "MotorManager.h"
#include "math.h"

#include "DebugLog.h"

#define DBG 1
//#define _MOTOR_OFF_

// Speed settings
int BaseSpeed = 200;
float LowRate = 0.90;
float HighRate = 0.80;

void MotorInit(void) {
    dxl_initialize( 0, DEFAULT_BAUDNUM ); // Not using device index
    //Wheel Mode
//    dxl_write_word( RIGHT_MOTOR, P_CW_ANGLE_LIMIT_L, 0 );
//    dxl_write_word( RIGHT_MOTOR, P_CW_ANGLE_LIMIT_H, 0 );
//    dxl_write_word( RIGHT_MOTOR, P_CCW_ANGLE_LIMIT_L, 0 );
//    dxl_write_word( RIGHT_MOTOR, P_CCW_ANGLE_LIMIT_H, 0 );
//    dxl_write_word( LEFT_MOTOR, P_CW_ANGLE_LIMIT_L, 0 );
//    dxl_write_word( LEFT_MOTOR, P_CW_ANGLE_LIMIT_H, 0 );
//    dxl_write_word( LEFT_MOTOR, P_CCW_ANGLE_LIMIT_L, 0 );
//    dxl_write_word( LEFT_MOTOR, P_CCW_ANGLE_LIMIT_H, 0 );
    //Set Torque
    dxl_write_word( RIGHT_MOTOR, 24, 1 );
    dxl_write_word( LEFT_MOTOR,  24, 1 );
    //Set EEP Lock
    dxl_write_word( RIGHT_MOTOR, P_EEP_LOCK, 1 );
    dxl_write_word( LEFT_MOTOR, P_EEP_LOCK, 1 );
    // Set goal speed
    dxl_write_word( BROADCAST_ID, P_GOAL_SPEED_L, 0 );
	// AX-S1 赤外線を低感度モードに設定
	
    _delay_ms(500);//1000
}

void MotorControl(int id, int power) {
#ifndef _MOTOR_OFF_
    int CommStatus = COMM_RXSUCCESS;
    dxl_write_word( id, P_GOAL_SPEED_L, power );
    CommStatus = dxl_get_result();
    if( CommStatus == COMM_RXSUCCESS )
        PrintErrorCode();
    else
        PrintCommStatus(CommStatus);
#endif // _MOTOR_OFF_
}

/************************************************************************/
// 関節用モーター実行関数
// id モーターID
// speed 速度
// position 位置（0～1023、512はセンター）
/************************************************************************/
void MotorControlJoint(int id, int speed, int position) {
	#ifndef _MOTOR_OFF_
	int CommStatus = COMM_RXSUCCESS;
	dxl_write_word( id, P_GOAL_POSITION_L, position );
	dxl_write_word( id, P_GOAL_SPEED_L, speed );
	CommStatus = dxl_get_result();
	if( CommStatus == COMM_RXSUCCESS )
	PrintErrorCode();
	else
	PrintCommStatus(CommStatus);
	#endif // _MOTOR_OFF_
}

void Execute(int type) {
	//type = 0;//DBG
    switch (type) {
        case MOVE_SELECTION_TYPE_START:
        case MOVE_SELECTION_TYPE_STRAIGHT:
		case TRACE_STRAIGHT:
            LOG_INFO("Straight\r\n");
            StraightMove();
            break;
        case MOVE_SELECTION_TYPE_STOP:
            LOG_INFO("Stop Move\r\n");
			StopMove();
            break;
        case MOVE_SELECTION_TYPE_RIGHTSIFT_1:
            LOG_INFO("Right Shift 1\r\n");
            StraightMoveRightShift();
            break;
        case MOVE_SELECTION_TYPE_LEFTSIFT_1:
            LOG_INFO("Left Shift 1\r\n");
            StraightMoveLeftShift();
            break;
        case MOVE_SELECTION_TYPE_RIGHTSIFT_2:
            LOG_INFO("Right Shift 2\r\n");
            StraightMoveRightShift2();
            break;
        case MOVE_SELECTION_TYPE_LEFTSIFT_2:
            LOG_INFO("Left Shift 2\r\n");
            StraightMoveLeftShift2();
            break;
        case MOVE_SELECTION_TYPE_RIGHTTURN:
            LOG_INFO("Right Turn\r\n");
            TurnLowMoveRight();
            break;
        case MOVE_SELECTION_TYPE_LEFTTURN:
            LOG_INFO("Left Turn\r\n");
            TurnLowMoveLeft();
            break;
        case MOVE_SELECTION_TYPE_RIGHTTURN_2:
            LOG_INFO("Right Turn 2\r\n");
			TurnMoveRight();
            break;
        case MOVE_SELECTION_TYPE_LEFTTURN_2:
            LOG_INFO("Left Turn 2\r\n");
			TurnMoveLeft();
            break;
        case MOVE_SELECTION_TYPE_BACK:
            LOG_INFO("Back Move\r\n");
            StopMove();
            _delay_ms(10);
            BackMove();
            _delay_ms(1000);
            break;
		case MOVE_SELECTION_TYPE_STRAIGHT_2:
            LOG_INFO("Straight Low Move\r\n");
            StraightLowMove();
            break;
			
		case TRACE_L_STRAIGHT:
            LOG_INFO("Left Straight\r\n");
            LeftStraightMove();
            break;
		case TRACE_R_STRAIGHT:
			LOG_INFO("Right Straight\r\n");
			RightStraightMove();
			break;
		case TRACE_L_ROUND:
			LOG_INFO("Left Round\r\n");
			LeftRoundMove();
			break;
		case TRACE_R_ROUND:
            LOG_INFO("Right Round\r\n");
            RightRoundMove();
            break;
		case TRACE_L_TURN:
            LOG_INFO("Left Turn\r\n");
            LeftTurnMove();
            break;	
		case TRACE_R_TURN:
            LOG_INFO("Right Turn\r\n");
            RightTurnMove();
            break;
        default:
            LOG_INFO("Unknown type[%d]\r\n", type);
            break;
    }
	
	if(type == MOVE_SELECTION_TYPE_STRAIGHT_2){
		mCount++;
	} else {
		mCount = 0;
	}
}

void setParamMoveAction(int right, int left) {
	int correctionVal = 0;
    MotorControl( RIGHT_MOTOR, right );
    MotorControl( LEFT_MOTOR, left + correctionVal );
    LOG_DEBUG("RIGHT_MORTOR:%d  LEFT_MORTOR:%d \r\n",right, left);
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
	LOG_INFO("left = %3d, Right = %3d\r\n", leftSpeed, rightSpeed);
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
	int speed = (BaseSpeed /2);
	int leftSpeed = (1024 + speed * 0.8);
	int rightSpeed = (1024 + speed);

	Move(leftSpeed, rightSpeed);
}

void RightTurnMove(void) {
	int speed = (BaseSpeed /2);
	int leftSpeed = speed;
	int rightSpeed = speed * 0.8;

	Move(leftSpeed, rightSpeed);
}

void PrintErrorCode() {
    if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
    LOG_ERROR("Input voltage error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
    LOG_ERROR("Angle limit error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
    LOG_ERROR("Overheat error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
    LOG_ERROR("Out of range error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
    LOG_ERROR("Checksum error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
    LOG_ERROR("Overload error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
    LOG_ERROR("Instruction code error!\n");
}

// Print communication result
void PrintCommStatus(int CommStatus) {
    switch(CommStatus)  {
        case COMM_TXFAIL:
            LOG_INFO("COMM_TXFAIL: Failed transmit instruction packet!\n");
            break;

        case COMM_TXERROR:
            LOG_INFO("COMM_TXERROR: Incorrect instruction packet!\n");
            break;

        case COMM_RXFAIL:
            LOG_INFO("COMM_RXFAIL: Failed get status packet from device!\n");
            break;

        case COMM_RXWAITING:
            LOG_INFO("COMM_RXWAITING: Now receiving status packet!\n");
            break;

        case COMM_RXTIMEOUT:
            LOG_INFO("COMM_RXTIMEOUT: There is no status packet!\n");
            break;

        case COMM_RXCORRUPT:
            LOG_INFO("COMM_RXCORRUPT: Incorrect status packet!\n");
            break;

        default:
            LOG_INFO("This is unknown error code!\n");
            break;
    }
}

/**
 * 現在の速度(右モータ)取得。
 * @brief 現在の速度(右モータ)取得。
 * @return 現在の速度(右モータ)
 * @detail 上位バイト3bit、下位8bitから現在の速度(右モータ)を取得する。
 *         パケット通信失敗時、前回の速度を返す。
 *         前進:1024～2047(CCW)
 *         後進:0000～1023(CW)
 */
int GetCurrentSpeedR(void) {
	int readValueHigh = 0;	// 上位バイト
	int readValueLow = 0;	// 下位バイト
	static int speed = 0;	// 現在の速度
	
	// 上位バイト取得
	readValueHigh = dxl_read_byte(RIGHT_MOTOR, CTRL_TBL_ADDR_PRESENT_SPEED_H) & 0x07;
	if(dxl_get_result() != COMM_RXSUCCESS) {
		// パケット通信失敗時、前回値を返す。
		return speed;
	}
	// 下位バイト取得
	readValueLow  = dxl_read_byte(RIGHT_MOTOR, CTRL_TBL_ADDR_PRESENT_SPEED_L) & 0xFF;
	if(dxl_get_result() != COMM_RXSUCCESS) {
		// パケット通信失敗時、前回値を返す。
		return speed;
	}
	// 上位バイトと下位バイトから現在の速度を計算
	speed = ((readValueHigh << 8) + readValueLow);
	LOG_DEBUG("GetCurrentSpeedR() is %d\n", speed);

	return speed;
}

/**
 * 現在の速度(左モータ)取得。
 * @brief 現在の速度(左モータ)取得。
 * @return 現在の速度(左モータ)
 * @detail 上位バイト3bit、下位8bitから現在の速度(左モータ)を取得する。
 *         パケット通信失敗時、前回の速度を返す。
 *         前進:0000～1023(CW)
 *         後進:1024～2047(CCW)
 */
int GetCurrentSpeedL(void) {
	int readValueHigh = 0;	// 上位バイト
	int readValueLow = 0;	// 下位バイト
	static int speed = 0;	// 現在の速度
	
	// 上位バイト取得
	readValueHigh = dxl_read_byte(LEFT_MOTOR, CTRL_TBL_ADDR_PRESENT_SPEED_H) & 0x07;
	if(dxl_get_result() != COMM_RXSUCCESS) {
		// パケット通信失敗時、前回値を返す。
		return speed;
	}
	// 下位バイト取得
	readValueLow  = dxl_read_byte(LEFT_MOTOR, CTRL_TBL_ADDR_PRESENT_SPEED_L) & 0xFF;
	if(dxl_get_result() != COMM_RXSUCCESS) {
		// パケット通信失敗時、前回値を返す。
		return speed;
	}
	// 上位バイトと下位バイトから現在の速度を計算
	speed = ((readValueHigh << 8) + readValueLow);
	LOG_DEBUG("GetCurrentSpeedL() is %d\n", speed);

	return speed;
}

/**
 * AX-S1の赤外線センサ値を取得する。
 * @brief AX-S1の赤外線センサ値を取得する。
 * @param (int *out_fire_data_left) 左側の赤外線センサ値
 * @param (int *out_fire_data_center) 中央の赤外線センサ値
 * @param (int *out_fire_data_right) 右側の赤外線センサ値
 * @return なし
 */
void GetAXS1SensorFireData(int *out_fire_data_left, int *out_fire_data_center, int *out_fire_data_right)
{
	static int fire_data_left = 0;		// 左側の赤外線センサ値
	static int fire_data_center = 0;	// 中央の赤外線センサ値
	static int fire_data_right = 0;		// 右側の赤外線センサ値
	
	// 左側の赤外線センサ値
	fire_data_left = dxl_read_byte(CENTER_AXS1_SENSOR, AXS1_ADDR_IR_LEFT_FIRE_DATA);
	if(dxl_get_result() != COMM_RXSUCCESS) {
		// パケット通信失敗時、前回値を返す。
		return;
	}
	*out_fire_data_left = fire_data_left;
	
	// 中央の赤外線センサ値
	fire_data_center = dxl_read_byte(CENTER_AXS1_SENSOR, AXS1_ADDR_IR_CENTER_FIRE_DATA);
	if(dxl_get_result() != COMM_RXSUCCESS) {
		// パケット通信失敗時、前回値を返す。
		return;
	}
	*out_fire_data_center = fire_data_center;
	
	// 右側の赤外線センサ値
	fire_data_right = dxl_read_byte(CENTER_AXS1_SENSOR, AXS1_ADDR_IR_RIGHT_FIRE_DATA);
	if(dxl_get_result() != COMM_RXSUCCESS) {
		// パケット通信失敗時、前回値を返す。
		return;
	}
	*out_fire_data_right = fire_data_right;
	
	LOG_DEBUG("GetAXS1SensorFireData() [%4d, %4d, %4d]\n", fire_data_left, fire_data_center, fire_data_right);

	return;
}