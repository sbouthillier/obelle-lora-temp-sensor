#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LoRaWan_APP.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "lora_payload.pb.h"


#define uS_TO_S_FACTOR 1000000      /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  15 * 60      /* Time ESP32 will go to sleep (in seconds) */

//----------------------------------------------------------------
// LoRa
//----------------------------------------------------------------
#define RF_FREQUENCY                                915000000 // Hz

#define TX_OUTPUT_POWER                             5        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 30 // Define the payload size here

static RadioEvents_t RadioEvents;

static uint8_t txpacket[BUFFER_SIZE];

//----------------------------------------------------------------
// Temperature sensor
//----------------------------------------------------------------

// GPIO where the DS18B20 is connected to
const int oneWireBus = 4;          

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);
DeviceAddress sensorAddr;

//----------------------------------------------------------------
// Type definitions
//----------------------------------------------------------------


//----------------------------------------------------------------
// Private data
//----------------------------------------------------------------
RTC_DATA_ATTR uint32_t packetCounter = 0;
static        bool     packet_transmitted;

//----------------------------------------------------------------
// Private functions
//----------------------------------------------------------------
static void vExtOn(void);
static void vExtOff(void);

void OnTxDone(void);
void OnTxTimeout(void);


void setup() {
    Serial.begin(115200);
    Mcu.begin();
    
    // Start the DS18B20 sensor
    vExtOn();
    sensors.begin();
    sensors.getAddress(sensorAddr, 0);
    vExtOff();
  
     // Start LoRa radio
    RadioEvents.TxDone    = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    
    Radio.Init( &RadioEvents );
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(MODEM_LORA,                       // Modem
                      TX_OUTPUT_POWER,                  // Power
                      0,                                // fdev
                      LORA_BANDWIDTH,                   // bandwidth
                      LORA_SPREADING_FACTOR,            // datarate
                      LORA_CODINGRATE,                  // coderate
                      LORA_PREAMBLE_LENGTH,             // preambleLen
                      LORA_FIX_LENGTH_PAYLOAD_ON,       // fixLen
                      true,                             // crcOn
                      0,                                // freqHopOn
                      0,                                // hopPeriod
                      LORA_IQ_INVERSION_ON,             // iqInverted
                      3000);                            // timeout

    Serial.println();

    pb_ostream_t ostream;
    LoraPayload payload = LoraPayload_init_zero;

    ++packetCounter;
    payload.id = packetCounter;

    // New temperature readings
    vExtOn();
    sensors.requestTemperaturesByAddress(sensorAddr); 
    // Temperature in Celsius degrees
    float temp = sensors.getTempC(sensorAddr);
    vExtOff();

    payload.temperature = temp;
    ostream = pb_ostream_from_buffer(txpacket, sizeof(txpacket));

    if(!pb_encode(&ostream, &LoraPayload_msg, &payload)) {
        Serial.printf("Encoding failed: %s\n", PB_GET_ERROR(&ostream));
    }
    else {
        packet_transmitted = false;

        // Send the same packet 3 times
        for(int i = 0; i < 3; i++) {
            Radio.Send(txpacket, ostream.bytes_written);
            packet_transmitted = false;

            while(packet_transmitted == false) {
                Radio.IrqProcess();
            }
            delay(100);
        }

        Serial.printf("Température %.2f", temp);
        Serial.println();
    }

    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.flush(); 
    Radio.Sleep( );
    esp_deep_sleep_start();
}

void loop() {
}

static void vExtOn(void) {
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, LOW);
}

static void vExtOff(void) {
    pinMode(Vext, INPUT);
}

void OnTxDone(void) {
    packet_transmitted = true;
	Serial.println("TX done......");
}

void OnTxTimeout(void) {
    packet_transmitted = true;
    Serial.println("TX Timeout......");
}
