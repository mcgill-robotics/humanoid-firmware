/*
 Code based on:
 Dynamixel.cpp - Ax-12+ Half Duplex USART Comunication
 Copyright (c) 2011 Savage Electronics.
 And Dynamixel Pro library for OpenCM-9.04 made by ROBOTIS, LTD.

 Modified to work only with Dynamixel XL-320 actuator.

 Modifications made by Luis G III for HelloSpoon robot.
 Webpage: http://hellospoonrobot.com
 Twitter: @HelloSpoon
 Youtube: http://youtube.com/user/hellospoonrobot

 This file can be used and be modified by anyone,
 don't forget to say thank you to OP!

 */

#ifndef XL320_H_
#define XL320_H_

#define MOTOR_CHOICE_XL320 1
#define MOTOR_CHOICE_XL430 2
#define MOTOR_CHOICE MOTOR_CHOICE_XL430

#define DYNAMIXAL_INSTR_PING 0x01
#define DYNAMIXAL_INSTR_READ 0x02
#define DYNAMIXAL_INSTR_WRITE 0x03
#define DYNAMIXAL_INSTR_REG_WRITE 0x04
#define DYNAMIXAL_INSTR_ACTION 0x05
#define DYNAMIXAL_INSTR_RESET 0x06
#define DYNAMIXAL_INSTR_SYNC_READ 0x82
#define DYNAMIXAL_INSTR_SYNC_WRITE 0x83

#define ADDRESS_SIZE 2

#if MOTOR_CHOICE == MOTOR_CHOICE_XL320
#define PAYLOAD_SIZE 2
/*EEPROM Area*/
#define XL_MODEL_NUMBER_L 0
#define XL_MODEL_NUMBER_H 1
#define XL_VERSION 2
#define XL_ID 3
#define XL_BAUD_RATE 4
#define XL_RETURN_DELAY_TIME 5
#define XL_CW_ANGLE_LIMIT_L 6
#define XL_CW_ANGLE_LIMIT_H 7
#define XL_CCW_ANGLE_LIMIT_L 8
#define XL_CCW_ANGLE_LIMIT_H 9
#define XL_CONTROL_MODE 11
#define XL_LIMIT_TEMPERATURE 12
#define XL_DOWN_LIMIT_VOLTAGE 13
#define XL_UP_LIMIT_VOLTAGE 14
#define XL_MAX_TORQUE_L 15
#define XL_MAX_TORQUE_H 16
#define XL_RETURN_LEVEL 17
#define XL_ALARM_SHUTDOWN 18
/*RAM Area*/
#define XL_TORQUE_ENABLE 24
#define XL_LED 25
#define XL_D_GAIN 27
#define XL_I_GAIN 28
#define XL_P_GAIN 29
#define XL_GOAL_POSITION_L 30
#define XL_GOAL_SPEED_L 32
#define XL_GOAL_TORQUE 35
#define XL_PRESENT_POSITION 37
#define XL_PRESENT_POSITION_SIZE 2
#define XL_PRESENT_SPEED 39
#define XL_PRESENT_LOAD 41
#define XL_PRESENT_VOLTAGE 45
#define XL_PRESENT_TEMPERATURE 46
#define XL_REGISTERED_INSTRUCTION 47
#define XL_MOVING 49
#define XL_HARDWARE_ERROR 50
#define XL_PUNCH 51
#elif MOTOR_CHOICE == MOTOR_CHOICE_XL430
#define PAYLOAD_SIZE 4
/* EEPROM Area */
#define XL_MODEL_NUMBER_L 0
#define XL_MODEL_NUMBER_H 1
#define XL_VERSION 6
#define XL_ID 7
#define XL_BAUD_RATE 8
#define XL_RETURN_DELAY_TIME 9
#define XL_CW_ANGLE_LIMIT_L 48	// Adjusted to a relevant register
#define XL_CW_ANGLE_LIMIT_H 49	// Adjusted to a relevant register
#define XL_CCW_ANGLE_LIMIT_L 50 // Adjusted to a relevant register
#define XL_CCW_ANGLE_LIMIT_H 51 // Adjusted to a relevant register
#define XL_CONTROL_MODE 11
#define XL_LIMIT_TEMPERATURE 31
#define XL_DOWN_LIMIT_VOLTAGE 34 // Min Voltage Limit in the new table
#define XL_UP_LIMIT_VOLTAGE 32	 // Max Voltage Limit in the new table
#define XL_MAX_TORQUE_L 36		 // Adjusted to a relevant register
#define XL_MAX_TORQUE_H 37		 // Adjusted to a relevant register
#define XL_RETURN_LEVEL 68		 // Status Return Level in the RAM Area
#define XL_ALARM_SHUTDOWN 63	 // Shutdown in the new table

/* RAM Area */
#define XL_GOAL_TORQUE -1
#define XL_TORQUE_ENABLE 64
#define XL_LED 65
#define XL_STATUS_RETURN_LEVEL 68
#define XL_REGISTERED_INSTRUCTION 69
#define XL_HARDWARE_ERROR 70
#define XL_VELOCITY_I_GAIN 76
#define XL_VELOCITY_P_GAIN 78
#define XL_POSITION_D_GAIN 80
#define XL_POSITION_I_GAIN 82
#define XL_POSITION_P_GAIN 84
#define XL_FEEDFORWARD_2ND_GAIN 88
#define XL_FEEDFORWARD_1ST_GAIN 90
#define XL_BUS_WATCHDOG 98
#define XL_GOAL_PWM 100
#define XL_GOAL_SPEED_L 104
#define XL_PROFILE_ACCELERATION 108
#define XL_PROFILE_VELOCITY 112
#define XL_GOAL_POSITION_L 116
#define XL_REALTIME_TICK 120
#define XL_MOVING 122
#define XL_MOVING_STATUS 123
#define XL_PRESENT_PWM 124
#define XL_PRESENT_LOAD 126
#define XL_PRESENT_SPEED 128
#define XL_PRESENT_POSITION 132
#define XL_PRESENT_POSITION_SIZE 4
#define XL_VELOCITY_TRAJECTORY 136
#define XL_POSITION_TRAJECTORY 140
#define XL_PRESENT_INPUT_VOLTAGE 144
#define XL_PRESENT_TEMPERATURE 146
#define XL_BACKUP_READY 147
#define XL_INDIRECT_ADDRESS_1 168
#define XL_INDIRECT_ADDRESS_2 170
#define XL_INDIRECT_ADDRESS_3 172
/* Continue for other indirect addresses and data if necessary */
#define XL_INDIRECT_DATA_1 224
#define XL_INDIRECT_DATA_2 225
#define XL_INDIRECT_DATA_3 226
/* Continue for other indirect data if necessary */
#define XL_INDIRECT_ADDRESS_29 578
#define XL_INDIRECT_ADDRESS_30 580
#define XL_INDIRECT_ADDRESS_31 582
#define XL_INDIRECT_ADDRESS_54 628
#define XL_INDIRECT_ADDRESS_55 630
#define XL_INDIRECT_ADDRESS_56 632
#define XL_INDIRECT_DATA_29 634
#define XL_INDIRECT_DATA_30 635
#define XL_INDIRECT_DATA_31 636
/* Continue for other indirect data if necessary */
#endif // MOTOR_CHOICE == MOTOR_CHOICE_XL320

#define Tx_MODE 1
#define Rx_MODE 0

#include <inttypes.h>
#include <Stream.h>

class XL320
{
private:
	unsigned char Direction_Pin;
	volatile char gbpParamEx[130 + 10];
	Stream *stream;

	void nDelay(uint32_t nTime);

public:
	XL320();
	virtual ~XL320();

	void begin(Stream &stream);

	void moveJoint(int id, int value);
	void setJointSpeed(int id, int value);
	void LED(int id, char led_color[]);
	void setJointTorque(int id, int value);

	void TorqueON(int id);
	void TorqueOFF(int id);
	int broadcastPing(Stream *debugStream, byte *IDbuf);

	void quickTest();
	void setID(int oldID, int newID);

	int getSpoonLoad();
	int getJointPosition(int id);
	int getJointPosition(int id, Stream *debugStream);
	int getJointSpeed(int id);
	int getJointLoad(int id);
	int getJointTemperature(int id);
	int isJointMoving(int id);

	int sendPacket(int id, int Address, int value);
	int readPacket(unsigned char *buffer, size_t size);

	int RXsendPacket(int id, int Address);
	int RXsendPacket(int id, int Address, int size);

	int flush();

	class Packet
	{
		bool freeData;

	public:
		unsigned char *data;
		size_t data_size;

		// wrap a received data stream in an Packet object for analysis
		Packet(unsigned char *data, size_t size);
		// build a packet into the pre-allocated data array
		// if data is null it will be malloc'ed and free'd on destruction.

		Packet(
			unsigned char *data,
			size_t size,
			unsigned char id,
			unsigned char instruction,
			int parameter_data_size,
			...);
		~Packet();
		unsigned char getId();
		int getLength();
		int getSize();
		int getParameterCount();
		unsigned char getInstruction();
		unsigned char getParameter(int n);
		bool isValid();

		void toStream(Stream &stream);
	};
};

#endif
