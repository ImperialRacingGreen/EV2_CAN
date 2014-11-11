#include "variant.h"
#include "due_can.h"

/**
 * Debugging flags
 */
#define SerialDebug Serial // Select Serial port for printing out debug messages
// #define DEBUG_PEDAL // Display pedal readings
#define DEBUG_PEDAL_AVERAGE // Display average pedal reading

/**
 * Constants
 */
#define CAN_BAUD_RATE CAN_BPS_500K
#define NDRIVE_RXID 0x210
#define NDRIVE_TXID 0x180

#define TEST1_CAN0_TX_PRIO             15
#define CAN_MSG_DUMMY_DATA             0x11BFFA4E

// CAN frame max data length
#define MAX_CAN_FRAME_DATA_LEN     8

// Pedal channels
#define PEDAL1_ADC_CHANNEL ADC_CHANNEL_7 // this is A0
#define PEDAL2_ADC_CHANNEL ADC_CHANNEL_6 // this is A1

/**
 * Calibrated values
 */
const int pedal1_min = 200;  // pedal1 min value in 12-bit range
const int pedal1_max = 1000; // pedal1 max value in 12-bit range
const int pedal2_min = 700;  // pedal2 min value in 12-bit range
const int pedal2_max = 1500; // pedal2 max value in 12-bit range

// abs(pedal1 - pedal2) in 16-bit range must not go over this
const int pedal_threshold = 10000;

// Registers to store raw pedal readings during interrupt
volatile int pedal1_raw = -1;
volatile int pedal2_raw = -1;

// volatile int CHANNEL_0_REG = 0;
// volatile int CHANNEL_1_REG = 0;
// volatile int CHANNEL_2_REG = 0;
// volatile int CHANNEL_3_REG = 0;
// volatile int CHANNEL_4_REG = 0;
// volatile int CHANNEL_5_REG = 0;
// volatile int CHANNEL_6_REG = 0;
// volatile int CHANNEL_7_REG = 0;
// volatile int CHANNEL_8_REG = 0;
// volatile int CHANNEL_9_REG = 0;

uint32_t sentFrames, receivedFrames;

//Leave this defined if you use the native port or comment it out if you use the programming port
//#define Serial SerialUSB

CAN_FRAME frame1, frame2, incoming;

CAN_FRAME frame_n_actual, frame_torque_cmd;

void adc_setup(void)
{
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

void populate_frames() {
    frame_n_actual.id = NDRIVE_RXID;
    frame_n_actual.length = 3;
    //frame_n_actual.data.low = 0x0064303d;
    //frame_n_actual.data.high = 0;
    frame_n_actual.data.bytes[0] = 0x3d;
    frame_n_actual.data.bytes[1] = 0x30;
    frame_n_actual.data.bytes[2] = 0x64;
    frame_n_actual.extended = 0;

    frame_torque_cmd.id = NDRIVE_RXID;
    frame_torque_cmd.length = 3;
    frame_torque_cmd.data.low = 0x0064903d;
    frame_torque_cmd.data.high = 0;
    frame_torque_cmd.extended = 0;
}

void abort_all_requests() {
    CAN_FRAME frame_abort, incoming;
    uint32_t counter = 0;
    
    frame_abort.id = NDRIVE_RXID;
    frame_abort.length = 3;
    frame_abort.data.bytes[0] = 0x3d;
    frame_abort.data.bytes[1] = 0x00; // REGID
    frame_abort.data.bytes[2] = 0xff;
    
    frame_abort.data.bytes[1] = 0x30;
    CAN.sendFrame(frame_abort);
    delayMicroseconds(100);

    frame_abort.data.bytes[1] = 0x90;
    CAN.sendFrame(frame_abort);
    delayMicroseconds(100);

    /*
    while (counter < 5000) {
        if (CAN.rx_avail()) {
            CAN.get_rx_buff(incoming);
            
            if (incoming.id == NDRIVE_TXID) {
                frame_abort.data.bytes[1] = incoming.data.bytes[0];
                CAN.sendFrame(frame_abort);
                delayMicroseconds(100);
            }
            
            counter = 0;
        } else {
            counter++;
        }
    }
    */
}

/*
bool has_received_data(uint8_t data_address) {
    if (CAN.rx_avail()) {
        CAN.get_rx_buff(incoming);

        if (incoming.id == NDRIVE_TXID && incoming.data[0] == data_address) {
            return true;
        }

        delayMicroseconds(100);
    }
    
    return false;
}
*/

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
int get_pedal_reading(const int raw_value, const int min_value, const int max_value)
{
    // Map to 16-bit range
    return constrain(map(raw_value, min_value, max_value, 0, 65536), 0, 65536);
}

/**
 * Returns average of two readings
 * @param  reading_1
 * @param  reading_2
 * @return
 */
int get_average_pedal_reading(const int reading_1, const int reading_2)
{
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

void setup() {

    // start serial port at 115200 bps:
    Serial.begin(115200);

    // adc setup
    adc_setup();

    // Verify CAN0 and CAN1 initialization, baudrate set by CAN_BAUD_RATE:
    if (CAN.init(CAN_BAUD_RATE) &&
        CAN2.init(CAN_BAUD_RATE)) {
    }
    else {
    Serial.println("CAN initialization (sync) ERROR");
    }

    //Both of these lines create a filter on the corresponding CAN device that allows
    //just the one ID we're interested in to get through.
    //The syntax is (mailbox #, ID, mask, extended)
    //You can also leave off the mailbox number: (ID, mask, extended)
    CAN.setRXFilter(0, NDRIVE_TXID, 0x1FFFFFFF, false);
    CAN2.setRXFilter(0, NDRIVE_TXID, 0x1FFFFFFF, false);

    populate_frames();

    abort_all_requests();
}

// Test rapid fire ping/pong of extended frames
static void test_1(void)
{
    CAN_FRAME inFrame;
    uint32_t counter = 0;
                
    // Send out the first frame
    CAN.sendFrame(frame_n_actual);
    sentFrames++;

    while (1==1) {
        if (CAN.rx_avail()) {
            CAN.get_rx_buff(incoming);
            CAN.sendFrame(frame_n_actual);
            delayMicroseconds(100);
            sentFrames++;
            receivedFrames++;
            counter++;
        }
        if (CAN2.rx_avail()) {
            CAN2.get_rx_buff(incoming);
            CAN2.sendFrame(frame_n_actual);
            delayMicroseconds(100);
            sentFrames++;
            receivedFrames++;
            counter++;
        }
        if (counter > 5000) {
            counter = 0;
            Serial.print("S: ");
            Serial.print(sentFrames);
            Serial.print(" R: ");
            Serial.println(receivedFrames);
        }
    }
}

CAN_FRAME create_throttle_frame(float value)
{
    CAN_FRAME frame;

    frame.id = NDRIVE_RXID;
    frame.length = 3;
    frame.data.bytes[0] = 0x3d;
    frame.data.bytes[1] = 0;
    frame.data.bytes[2] = 0;
    frame.extended = 0;

    return frame;
}

// can_example application entry point
void loop()
{
    CAN_FRAME incoming, test_frame, test_frame_2, test_frame_3;

    test_frame.id = NDRIVE_RXID;
    test_frame.length = 3;
    test_frame.data.bytes[0] = 0x3d;
    test_frame.data.bytes[1] = 0x30;
    test_frame.data.bytes[2] = 0x64;
    test_frame.extended = 0;

    test_frame_2.id = NDRIVE_RXID;
    test_frame_2.length = 3;
    test_frame_2.data.bytes[0] = 0x3d;
    test_frame_2.data.bytes[1] = 0x90;
    test_frame_2.data.bytes[2] = 0x64;
    test_frame_2.data.high = 0;
    test_frame_2.extended = 0;

    test_frame_3.id = NDRIVE_RXID;
    test_frame_3.length = 3;
    test_frame_3.data.bytes[0] = 0x90;
    test_frame_3.data.bytes[1] = 0xFC;
    test_frame_3.data.bytes[2] = 0x3F;

    //CAN.sendFrame(test_frame);
    //delayMicroseconds(100);
    //CAN.sendFrame(test_frame_2);
    //delayMicroseconds(100);

    while (1) {        
        // retrieves pedal input
        int reading_1 = get_pedal_reading(pedal1_raw, pedal1_min, pedal1_max);
        int reading_2 = get_pedal_reading(pedal2_raw, pedal2_min, pedal2_max);
        int average_reading = get_average_pedal_reading(reading_1, reading_2);
        assert_pedal_in_threshold(reading_1, reading_2, pedal_threshold);

        #ifdef DEBUG_PEDAL
        SerialDebug.print("Raw value (1): ");
        SerialDebug.println(pedal1_raw);
        SerialDebug.print("Raw value (2): ");
        SerialDebug.println(pedal2_raw);

        SerialDebug.print("Computed value (1): ");
        SerialDebug.println(reading_1);
        SerialDebug.print("Computed value (2): ");
        SerialDebug.println(reading_2);

        SerialDebug.print("Difference (1-2): ");
        SerialDebug.println(abs(reading_1 - reading_2));
        #endif

        #ifdef DEBUG_PEDAL_AVERAGE
        SerialDebug.print("Average (1,2): ");
        SerialDebug.println(average_reading);
        #endif

        //test_frame_3.data.bytes[1] = reading & 0xff;
        //test_frame_3.data.bytes[2] = (reading >> 8) & 0xff;

        // Serial.print(test_frame_3.data.bytes[1], HEX);
        // Serial.print(" ");
        // Serial.print(test_frame_3.data.bytes[2], HEX);
        // Serial.print(" ");
        // Serial.print(PEDAL2_REG, HEX);
        // Serial.print(" ");
        // Serial.println();

        // printFrame(test_frame_3);
        // delayMicroseconds(100);
        // CAN.sendFrame(test_frame_3);
        // delayMicroseconds(100);
        // delay(1000);

        if (CAN.rx_avail()) {
            CAN.get_rx_buff(incoming); 
            printFrame(incoming);
        }
        if (CAN2.rx_avail()) {
            CAN2.get_rx_buff(incoming); 
            printFrame(incoming);
        }
    }
}
