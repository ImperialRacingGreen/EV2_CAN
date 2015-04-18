#ifndef EV2_CAN
#define EV2_CAN

#include "due_can.h"

/**
* CANBus related constants
**/
#define CAN_BAUD_RATE CAN_BPS_500K
#define NDRIVE_RXID 0x210
#define NDRIVE_TXID 0x180

// Writing
#define SPEED_WRITE_ADD		0x31
#define MAX_SPEED_WRITE 	0x0CCD
#define TORQUE_WRITE_ADD	0x90
#define MAX_TORQUE_WRITE	0x7FF8

// Reading
#define DS_SERVO		0x3D		//Motor Controller
#define SPEED_READ_ADD	0x30
#define MAX_SPEED_READ	0x7FFF	
#define CORE_STATUS		0x40
#define KERN_STATUS		0x181 		// Bit 0 = Drive Enabled, Bit 7 = Position Control, Bit 8 = Speed Control

bool CAN_setup();
void printFrame(CAN_FRAME &frame);
void parseFrame(CAN_FRAME &frame);	// for logging
void createFrame(CAN_FRAME &frame, int RXID, int length, int REGID, int DATA_1, int DATA_2);

void abort_requests(int REGID);
void abort_all_requests();
bool status();
void emergency_stop();
void assert_or_abort(bool condition);


#define SPEED_REPETITION 100;
void createSpeedRequestFrame(CAN_FRAME &frame, int repetition); // repition in ms
void createCoreStatusRequestFrame(CAN_FRAME &frame);

/**
*	Pedal Reading
**/
// Pedal channels
#define PEDAL1_ADC_CHANNEL ADC_CHANNEL_7 // this is A0
#define PEDAL2_ADC_CHANNEL ADC_CHANNEL_6 // this is A1

void adc_setup(void);
void ADC_Handler(void);
int get_pedal_reading(const int raw_value, const int min_value, const int max_value);
int get_average_pedal_reading(const int reading_1, const int reading_2);
void assert_pedal_in_threshold(const int reading_1, const int reading_2, const int threshold);

/**
*	BMS Related Constants
**/
#define BMS_STATUS		0x622
#define PACK_VOLTAGE	0x623
#define PACK_CURRENT	0x624
#define PACK_SOC		0x626
#define PACK_TEMP		0x627

#define MAX_TEMP		100
#define MIN_TEMP		0

#endif