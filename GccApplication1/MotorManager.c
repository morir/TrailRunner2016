/*
 * MotorManager.c
 *
 * Created: 2014/10/29 14:46:23
 *  Author: Administrator
 */ 
#include "MotorManager.h"
#include "math.h"

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
//  printf( "%d %d\n", id, power );
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
            printf("Straight Move\r\n");
            StraightMove();
            break;
        case MOVE_SELECTION_TYPE_STOP:
            printf("Stop Move\r\n");
			StopMove();
            break;
        case MOVE_SELECTION_TYPE_RIGHTSIFT_1:
            printf("Right Shift 1\r\n");
            StraightMoveRightShift();
            break;
        case MOVE_SELECTION_TYPE_LEFTSIFT_1:
            printf("Left Shift 1\r\n");
            StraightMoveLeftShift();
            break;
        case MOVE_SELECTION_TYPE_RIGHTSIFT_2:
            printf("Right Shift 2\r\n");
            StraightMoveRightShift2();
            break;
        case MOVE_SELECTION_TYPE_LEFTSIFT_2:
            printf("Left Shift 2\r\n");
            StraightMoveLeftShift2();
            break;
        case MOVE_SELECTION_TYPE_RIGHTTURN:
            printf("Right Turn\r\n");
            TurnLowMoveRight();
            break;
        case MOVE_SELECTION_TYPE_LEFTTURN:
            printf("Left Turn\r\n");
            TurnLowMoveLeft();
            break;
        case MOVE_SELECTION_TYPE_RIGHTTURN_2:
            printf("Right Turn 2\r\n");
			TurnMoveRight();
            break;
        case MOVE_SELECTION_TYPE_LEFTTURN_2:
            printf("Left Turn 2\r\n");
			TurnMoveLeft();
            break;
        case MOVE_SELECTION_TYPE_BACK:
            printf("Back Move\r\n");
            StopMove();
            _delay_ms(10);
            BackMove();
            _delay_ms(1000);
            break;
		case MOVE_SELECTION_TYPE_STRAIGHT_2:
            printf("Straight Low Move\r\n");
            StraightLowMove();
            break;
			
        case MOVE_SELECTION_TYPE_S_MOVE_10:
            TestMode();
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
    printf("RIGHT_MORTOR:%d  LEFT_MORTOR:%d \r\n",right, left);//DBG
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

void TestMode(void) {
    long move_time = 100000;
    long stop_time = 50000;
    StopMove();
    _delay_ms(stop_time);

    StraightMove();
    _delay_ms(move_time);

    StopMove();
    _delay_ms(stop_time);

    StraightMoveLeftShift();
    _delay_ms(move_time);

    StopMove();
    _delay_ms(stop_time);

    StraightMoveRightShift();
    _delay_ms(move_time);

    StopMove();
    _delay_ms(stop_time);

    StraightMoveLeftShift2();
    _delay_ms(move_time);

    StopMove();
    _delay_ms(stop_time);

    StraightMoveRightShift2();
    _delay_ms(move_time);

    StopMove();
    _delay_ms(stop_time);

    TurnMoveRight();
    _delay_ms(move_time);

    StopMove();
    _delay_ms(stop_time);

    TurnMoveLeft();
    _delay_ms(move_time);

    StopMove();
    _delay_ms(stop_time);

    BackMove();
    _delay_ms(move_time);
}

void PrintErrorCode() {
    if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
    printf("Input voltage error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
    printf("Angle limit error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
    printf("Overheat error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
    printf("Out of range error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
    printf("Checksum error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
    printf("Overload error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
    printf("Instruction code error!\n");
}

// Print communication result
void PrintCommStatus(int CommStatus) {
    switch(CommStatus)  {
        case COMM_TXFAIL:
            printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
            break;

        case COMM_TXERROR:
            printf("COMM_TXERROR: Incorrect instruction packet!\n");
            break;

        case COMM_RXFAIL:
            printf("COMM_RXFAIL: Failed get status packet from device!\n");
            break;

        case COMM_RXWAITING:
            printf("COMM_RXWAITING: Now receiving status packet!\n");
            break;

        case COMM_RXTIMEOUT:
            printf("COMM_RXTIMEOUT: There is no status packet!\n");
            break;

        case COMM_RXCORRUPT:
            printf("COMM_RXCORRUPT: Incorrect status packet!\n");
            break;

        default:
            printf("This is unknown error code!\n");
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
//	speed = (dxl_read_byte(id, 0X27) << 8);
//	speed += dxl_read_byte(id, 0X26);
//  printf("Current speed is %d\n", speed);
	
	speed = dxl_read_word(id, 0x26);
	printf("Current speed is %d\n", speed);
	return speed;
}