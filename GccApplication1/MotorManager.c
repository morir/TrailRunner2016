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

void Execute(int type) {
	//type = 0;//DBG
    switch (type) {
        case MOVE_SELECTION_TYPE_START:
        case MOVE_SELECTION_TYPE_STRAIGHT:
		case TRACE_STRAIGHT:
            LOG_INFO("Straight Move\r\n");
            StraightMove();
            break;
        case MOVE_SELECTION_TYPE_STOP:
            LOG_INFO("Stop Move\r\n");
			StopMove();
            break;
        case MOVE_SELECTION_TYPE_RIGHTSIFT_1:
		case TRACE_RIGHTMOVE_1:
            LOG_INFO("Right Shift 1\r\n");
            StraightMoveRightShift();
            break;
        case MOVE_SELECTION_TYPE_LEFTSIFT_1:
		case TRACE_LEFTMOVE_1:
            LOG_INFO("Left Shift 1\r\n");
            StraightMoveLeftShift();
            break;
        case MOVE_SELECTION_TYPE_RIGHTSIFT_2:
		case TRACE_RIGHTMOVE_2:
            LOG_INFO("Right Shift 2\r\n");
            StraightMoveRightShift2();
            break;
        case MOVE_SELECTION_TYPE_LEFTSIFT_2:
		case TRACE_LEFTMOVE_2:
            LOG_INFO("Left Shift 2\r\n");
            StraightMoveLeftShift2();
            break;
        case MOVE_SELECTION_TYPE_RIGHTTURN:
		case TRACE_RIGHTTURN:
            LOG_INFO("Right Turn\r\n");
            TurnLowMoveRight();
            break;
        case MOVE_SELECTION_TYPE_LEFTTURN:
		case TRACE_LEFTTURN:
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
			
        default:
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

void StraightMove(void) {
    MotorControl( RIGHT_MOTOR, 1623 ); //300 P_CCW_SPEED_NOMAL 1632
    MotorControl( LEFT_MOTOR, 600 ); //300 P_CW_SPEED_NOMAL 609
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

void AdjustSpeed(int expectedR, int expectedL) {
	// 9/20 TODO:
	// 取得値が目標値より高い値は読み飛ばす。
	// 目標値の判断はセンサ値で。
	// 設定値はセンサ値に負荷分で遅くなってしまう速度分を加味する。

	// RIGHT_MOTOR
	//   Forward：0 - 1023  Backward：1024 - 2047
	int realR = GetCurrentSpeed(RIGHT_MOTOR);
	int diffR = (expectedR - realR);
	int adjustedR = (expectedR + diffR);
	
	// Range check
	if (expectedR < 1024) {
		if (adjustedR < 0) {
			adjustedR = 0;
		}
		else if (adjustedR > 1023) {
			adjustedR = 1023;
		}	
	}
	else {
		if (adjustedR < 1024) {
			adjustedR = 1024;
		}
		else if (adjustedR > 2047) {
			adjustedR = 2047;
		}
	}

	// LEFT_MOTOR
	//   Forward：1024 - 2047  Backward：0 - 1023
	int realL = GetCurrentSpeed(LEFT_MOTOR);
	int diffL = (expectedL - realL);
	int adjustedL = (expectedL + diffL);
	
	// Range check
	if (expectedR > 1023) {
		if (adjustedL < 1024) {
			adjustedL = 1024;
		}
		else if (adjustedL > 2047) {
			adjustedL = 2047;
		}
	}
	else {
		if (adjustedL < 0) {
			adjustedL = 0;
		}
		else if (adjustedL > 1023) {
			adjustedL = 1023;
		}
	}
	
	MotorControl(RIGHT_MOTOR, adjustedR);
	MotorControl(LEFT_MOTOR,  adjustedL);
}

int GetCurrentSpeed(int id) {
	int readValueHigh = 0;
	int readValueLow = 0;
	int speed = 0;
	
	readValueHigh = dxl_read_byte(id, 39) & 0x3;
	readValueLow  = dxl_read_byte(id, 38) & 0xF0;
	speed = ((readValueHigh << 8) + readValueLow);
	LOG_INFO("Current speed is %d\n", speed);
	return speed;
}
