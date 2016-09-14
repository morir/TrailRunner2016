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
            LOG_INFO("Straight Move\r\n");
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
	AdjustSpeed(1024, 0);
}

void StraightMove(void) {
	AdjustSpeed(1623, 600);
}

void StraightLowMove(void) {
	AdjustSpeed(1098, 75);
}

void StraightMoveRightShift(void) {
	AdjustSpeed(1473, 500);
}

void StraightMoveLeftShift(void) {
	AdjustSpeed(1523, 450);
}

void StraightMoveRightShift2(void) {
	AdjustSpeed(1423, 500);
}

void StraightMoveLeftShift2(void) {
	AdjustSpeed(1523, 400);
}

void TurnMoveRight(void) {
	AdjustSpeed(400, 400);
}

void TurnMoveLeft(void) {
	AdjustSpeed(1424, 1424);
}

void TurnLowMoveRight(void) {
	AdjustSpeed(300, 300);
}

void TurnLowMoveLeft(void) {
	AdjustSpeed(1324, 1324);
}

void BackMove(void) {
	AdjustSpeed(250, 1273);
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
	int speed = 0;
	
	speed = dxl_read_word(id, 0x26);
	LOG_INFO("Current speed is %d\n", speed);
	return speed;
}