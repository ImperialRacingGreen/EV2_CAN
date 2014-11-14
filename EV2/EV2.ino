#include "variant.h"
#include "EV2_CAN.h"

/**
 * Debugging flags
 */
#define SerialDebug Serial // Select Serial port for printing out debug messages
// #define DEBUG_PEDAL // Display pedal readings
#define DEBUG_PEDAL_AVERAGE // Display average pedal reading

void testPingPong();

void setup() {
    // start serial port at 115200 bps:
    Serial.begin(115200);

    adc_setup();
    if (!CAN_setup())
        Serial.println("CAN initialization (sync) ERROR");

    Serial.println("SetupDone");

    // 1. abort_all_requests(); // reset CAN Bus comms
    // 2. check status
    // 3. setup repeated speed messages
    // 4. setup repeated bms_status messages
}

void loop()
{
    // 1. parse frame
    // 2. check if emergency

    //Testing
    testPingPong();

    Serial.println("end");
}

void testPingPong() {
    // TESTING
    Serial.println("test");


    CAN_FRAME outFrame,inFrame;
    createFrame(outFrame,0x11,3,0x22,0x33,0x44);
    printFrame(outFrame);

    uint32_t counter = 0;
    int sentFrames = 0;
    int receivedFrames = 0;

    // Send out the first frame
    CAN.sendFrame(outFrame);
    sentFrames++;

    Serial.println("sent");

    while (true) {
        if (CAN.rx_avail()) {
            CAN.get_rx_buff(inFrame);
            CAN.sendFrame(inFrame);
            delayMicroseconds(100);
            sentFrames++;
            receivedFrames++;
            counter++;
        }
        if (CAN2.rx_avail()) {
            CAN2.get_rx_buff(inFrame);
            CAN2.sendFrame(inFrame);
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