/*
 * motor_dxl.h
 *
 *  Created on: Aug 12, 2015
 *      Author: odroid
 */

#ifndef MOTOR_DXL_H_
#define MOTOR_DXL_H_

#define P_MODEL_NUMBER_L      		0
#define P_MODEL_NUMBER_H      		1
#define P_VERSION             		2
#define P_ID                  		3
#define P_BAUD_RATE           		4
#define P_RETURN_DELAY_TIME   		5
#define P_CW_ANGLE_LIMIT_L    		6
#define P_CW_ANGLE_LIMIT_H    		7
#define P_CCW_ANGLE_LIMIT_L   		8
#define P_CCW_ANGLE_LIMIT_H   		9
#define P_SYSTEM_DATA2        		10
#define P_LIMIT_TEMPERATURE   		11
#define P_DOWN_LIMIT_VOLTAGE  		12
#define P_UP_LIMIT_VOLTAGE    		13
#define P_MAX_TORQUE_L        		14
#define P_MAX_TORQUE_H        		15
#define P_RETURN_LEVEL        		16
#define P_ALARM_LED           		17
#define P_ALARM_SHUTDOWN      		18
#define P_OPERATING_MODE      		19
#define P_DOWN_CALIBRATION_L  		20
#define P_DOWN_CALIBRATION_H  		21
#define P_UP_CALIBRATION_L    		22
#define P_UP_CALIBRATION_H    		23

//------------------------------------------------------------------------------
//					===== Control Table Address =====
//					===== RAM AREA				=====
//------------------------------------------------------------------------------
#define P_TORQUE_ENABLE         	(24)
#define P_LED                 	   	(25)
#define P_CW_COMPLIANCE_MARGIN 	  	(26)
#define P_CCW_COMPLIANCE_MARGIN		(27)
#define P_CW_COMPLIANCE_SLOPE    	(28)
#define P_CCW_COMPLIANCE_SLOPE   	(29)
#define P_GOAL_POSITION_L        	(30)
#define P_GOAL_POSITION_H        	(31)
#define P_GOAL_SPEED_L           	(32)
#define P_GOAL_SPEED_H           	(33)
#define P_TORQUE_LIMIT_L         	(34)
#define P_TORQUE_LIMIT_H         	(35)
#define P_PRESENT_POSITION_L     	(36)
#define P_PRESENT_POSITION_H     	(37)
#define P_PRESENT_SPEED_L        	(38)
#define P_PRESENT_SPEED_H        	(39)
#define P_PRESENT_LOAD_L         	(40)
#define P_PRESENT_LOAD_H         	(41)
#define P_PRESENT_VOLTAGE        	(42)
#define P_PRESENT_TEMPERATURE    	(43)
#define P_REGISTERED_INSTRUCTION 	(44)
#define P_PAUSE_TIME             	(45)
#define P_MOVING                 	(46)
#define P_LOCK                   	(47)
#define P_PUNCH_L                	(48)
#define P_PUNCH_H                	(49)

//------------------------------------------------------------------------------
//					===== Instruction =====
//------------------------------------------------------------------------------
#define INST_PING           		0x01	//
#define INST_READ           		0x02	// Control Table
#define INST_WRITE          		0x03	// Control Table
#define INST_REG_WRITE     		 	0x04	// Control Table
#define INST_ACTION         		0x05	// Control Table
#define INST_RESET          		0x06	// Control Table
#define INST_DIGITAL_RESET  		0x07	// Reserved
#define INST_SYSTEM_READ    		0x0C	// Reserved
#define INST_SYSTEM_WRITE   		0x0D	// Reserved
#define INST_SYNC_WRITE     		0x83	// Control Table
#define INST_SYNC_REG_WRITE 		0x84	// Control Table

//------------------------------------------------------------------------------
//					===== User Define =====
//------------------------------------------------------------------------------
#define CLEAR_BUFFER RxBufferReadPointer = RxBufferWritePointer
#define DEFAULT_RETURN_PACKET_SIZE 	6
#define BROADCASTING_ID 			0xFE


#define SET_TxD0_FINISH   	sbi(UCSR0A,6)
#define RESET_TXD0_FINISH 	cbi(UCSR0A,6)
#define CHECK_TXD0_FINISH 	(UCSR0A & (1<<TXC0))

#define SET_TxD1_FINISH   	sbi(UCSR1A,6)
#define RESET_TXD1_FINISH 	cbi(UCSR1A,6)
#define CHECK_TXD1_FINISH 	(UCSR1A & (1<<TXC1))

#define RX_INTERRUPT 		0x01
#define TX_INTERRUPT 		0x02
#define OVERFLOW_INTERRUPT 	0x01
#define SERIAL_PORT0 		0
#define SERIAL_PORT1 		1

//------------------------------------------------------------------------------
//			 		===== TXD / RXD  =====
//------------------------------------------------------------------------------
#define TXD0_READY			(UCSR0A & (1<<UDRE0))
#define TXD0_DATA			(UDR0)
#define RXD0_READY			(UCSR0A & (1<<RXC0))
#define RXD0_DATA			(UDR0)

#define TXD1_READY			(UCSR1A & (1<<UDRE1))
#define TXD1_DATA			(UDR1)
#define RXD1_READY			(UCSR1A & (1<<RXC1))
#define RXD1_DATA			(UDR1)

//------------------------------------------------------------------------------
//         			===== Funtion Prototype =====
//------------------------------------------------------------------------------
//unsigned char TxPacket(unsigned char ID, unsigned char Instruction, unsigned char ParameterLength);



#endif /* MOTOR_DXL_H_ */
