#include "EV2_CAN.h"

// Registers to store raw pedal readings during interrupt
volatile int pedal1_raw = -1;
volatile int pedal2_raw = -1;

bool CAN_setup() {
	if (CAN.init(CAN_BAUD_RATE) && CAN2.init(CAN_BAUD_RATE)) {}
    else
		return false;

    //By default there are 7 mailboxes for each device that are RX boxes
    //This sets each mailbox to have an open filter that will accept extended
    //or standard frames
    int filter;
    for (int filter = 0; filter < 7; filter++) {
        CAN.setRXFilter(filter, 0, 0, false);
        CAN2.setRXFilter(filter, 0, 0, false);
    }      

    return true;
}

void printFrame(CAN_FRAME &frame) {
	Serial.print("ID: 0x");
	Serial.print(frame.id, HEX);
    Serial.print(" Len: ");
    Serial.print(frame.length);
    Serial.print(" Data: 0x");
    for (int count = 0; count < frame.length; count++) {
        Serial.print(frame.data.bytes[count], HEX);
        Serial.print(" ");
    }
    Serial.print("\r\n");
}

void createFrame(CAN_FRAME &frame, int RXID, int length, int REGID, int DATA_1, int DATA_2) {
	frame.id = RXID;
	frame.length = length;
	frame.extended = 0;

	frame.data.bytes[0] = REGID;
	frame.data.bytes[1] = DATA_1;
	frame.data.bytes[2] = DATA_2;

	// frame.data.low = (REGID, data1 and data2 combined)
}

void parseFrame(CAN_FRAME &frame) {
    // Motor Controller
    if (frame.id == NDRIVE_TXID) {
        switch(frame.data[0]) {
            // MC Related
            case SPEED_READ_ADD:
                int speed = (frame.data[1] << 8) | frame.data[2];
                break;
            case CORE_STATUS:
                int status = (frame.data[1] << 8) | frame.data[2];
                if (status == KERN_STATUS) {
                    // DRIVE IS ENABLED
                    // POSITION CONTROL IS ENABLED
                    // SPEED CONTROL IS ENABLED
                } 
                break;
        }
    }
    else {
    	switch (frame.data[0]) {
            // BMS Related
    		case BMS_STATUS:
                if (frame.data[1] != 0) {
                    // ERROR
                }
    			break;
            case PACK_VOLTAGE:
                break;
            case PACK_CURRENT:
                break;
            case PACK_SOC:
                break;
            case PACK_TEMP:
                if (frame.data[1] > MAX_TEMP) {
                    // TOO HOT
                }
                break;
    	}
    }
}

void abort_requests(int REGID) {
	CAN_FRAME frame_abort;
	createFrame(frame_abort, NDRIVE_RXID, 3, DS_SERVO, REGID, 0xFF);
	CAN.sendFrame(frame_abort);
	delayMicroseconds(100);
}

bool status() {
	CAN_FRAME frame_status,incoming;
	createFrame(frame_status, NDRIVE_RXID, 3, DS_SERVO, CORE_STATUS, 0x00);
	CAN.sendFrame(frame_status);
	delayMicroseconds(100);

	int counter = 0;
	while(counter < 5000) {
		if (CAN.rx_avail()) {
            CAN.get_rx_buff(incoming);
            if (incoming.data.low == KERN_STATUS)
            	return true;
            else
            	return false;
        }
        else
        	counter++;
	}
	return false;
}

void adc_setup(void) {
    adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, 8);
    adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
    adc_set_resolution(ADC, ADC_12_BITS);

    // Enable ADC channels arrange by arduino pins from A0 to A9
    // adc_enable_channel(ADC, ADC_CHANNEL_7); // A0
    // adc_enable_channel(ADC, ADC_CHANNEL_6); // A1
    // adc_enable_channel(ADC, ADC_CHANNEL_5); // A2
    // adc_enable_channel(ADC, ADC_CHANNEL_4); // A3
    // adc_enable_channel(ADC, ADC_CHANNEL_3); // A4
    // adc_enable_channel(ADC, ADC_CHANNEL_2); // A5
    // adc_enable_channel(ADC, ADC_CHANNEL_1); // A6
    // adc_enable_channel(ADC, ADC_CHANNEL_0); // A7
    // adc_enable_channel(ADC, ADC_CHANNEL_10); // A8
    // adc_enable_channel(ADC, ADC_CHANNEL_11); // A9

    // Enable ADC channels for pedals
    adc_enable_channel(ADC, PEDAL1_ADC_CHANNEL);
    adc_enable_channel(ADC, PEDAL2_ADC_CHANNEL);

    // Enable ADC interrupt
    adc_enable_interrupt(ADC, ADC_IER_EOC7); //EOC9 so that interrupt triggered when analogue input channerl 9 has reached end of conversion
    
    // Trigger configuration
    adc_configure_trigger(ADC, ADC_TRIG_SW, 0);
    
    // Enable ADC interrupt
    NVIC_EnableIRQ(ADC_IRQn);
    
    //start ADC conversion, note that ADC conversion has to be restarted once conversion is finished
    adc_start(ADC);
}

void ADC_Handler(void)
{
    // Check the ADC conversion status
    if ((adc_get_status(ADC) & ADC_ISR_EOC7) == ADC_ISR_EOC7)
    {
        // // Get digital data value from ADC channels and can be used by application
        // CHANNEL_0_REG = adc_get_channel_value(ADC, ADC_CHANNEL_0);
        // CHANNEL_1_REG = adc_get_channel_value(ADC, ADC_CHANNEL_1);
        // CHANNEL_2_REG = adc_get_channel_value(ADC, ADC_CHANNEL_2);
        // CHANNEL_3_REG = adc_get_channel_value(ADC, ADC_CHANNEL_3);
        // CHANNEL_4_REG = adc_get_channel_value(ADC, ADC_CHANNEL_4);
        // CHANNEL_5_REG = adc_get_channel_value(ADC, ADC_CHANNEL_5);
        // CHANNEL_6_REG = adc_get_channel_value(ADC, ADC_CHANNEL_6);
        // CHANNEL_7_REG = adc_get_channel_value(ADC, ADC_CHANNEL_7);
        // CHANNEL_8_REG = adc_get_channel_value(ADC, ADC_CHANNEL_10); //notice that its channel 10
        // CHANNEL_9_REG = adc_get_channel_value(ADC, ADC_CHANNEL_11); //notice that its channel 11

        pedal1_raw = adc_get_channel_value(ADC, PEDAL1_ADC_CHANNEL);
        pedal2_raw = adc_get_channel_value(ADC, PEDAL2_ADC_CHANNEL);
    }
    adc_start(ADC);
}


/**
 * Emergency functions
 * -------------------
 */

/**
 * Stops the vehicle completely
 */
void emergency_stop()
{
    // TODO send shutdown commands, functions, etc.

    #ifdef SerialDebug
    SerialDebug.println("Aborted!");
    #endif

    while (true) {
        // Infinite loop
    }
}

/**
 * Emergency stop the vehicle if condition is invalid.
 * @param condition
 */
void assert_or_abort(bool condition)
{
    if ( ! condition) {
        emergency_stop();
    }
}

/**
 * Pedal reading functions
 * -----------------------
 */

/**
 * Processes pedal reading. Maps value to 16-bit range.
 * @param  raw_value Raw value
 * @param  min_value Minimum value
 * @param  max_value Maximum value
 * @return value     Processed value
 */
int get_pedal_reading(const int raw_value, const int min_value, const int max_value) {
    // Map to 16-bit range
    return constrain(map(raw_value, min_value, max_value, 0, 65536), 0, 65536);
}

/**
 * Returns average of two readings
 * @param  reading_1
 * @param  reading_2
 * @return
 */
int get_average_pedal_reading(const int reading_1, const int reading_2) {
    return (reading_1 + reading_2) / 2;
}

/**
 * Asserts pedal readings are in threshold
 * @param  reading_1
 * @param  reading_2
 * @param  threshold
 */
void assert_pedal_in_threshold(const int reading_1, const int reading_2, const int threshold)
{
    int difference = abs(reading_1 - reading_2);
    bool condition = difference < threshold;

    #ifdef SerialDebug
    if ( ! condition) {
        SerialDebug.println("Pedal reading discrepancy detected!");
        SerialDebug.print("Reading (1): ");
        SerialDebug.println(reading_1);
        SerialDebug.print("Reading (2): ");
        SerialDebug.println(reading_2);
        SerialDebug.print("Threshold: ");
        SerialDebug.println(threshold);
        SerialDebug.print("Difference: ");
        SerialDebug.println(difference);
    }
    #endif

    assert_or_abort(condition);
}