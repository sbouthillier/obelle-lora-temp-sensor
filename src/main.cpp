/*
 * Copyright (c) 2024 EmbedGenius. All rights reserved.
 */

#include <Arduino.h>
#include <LoRaWan_APP.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <HardwareSerial.h>
#include <ArduinoLog.h>
#include <error_or.h>
#include <lora_payload.pb.h>

using error::ErrorOr;
using error::Error;

const auto  uS_TO_S_FACTOR  {1000000};      /* Conversion factor for micro seconds to seconds */
/* Sleeping time is 20 minutes (20 * 60 = 1200 seconds) */
const auto  TIME_TO_SLEEP   {20 * 60};      /* Time ESP32 will go to sleep (in seconds) */

//----------------------------------------------------------------
// LoRa
//----------------------------------------------------------------
constexpr auto  RF_FREQUENCY               {915000000};     // Hz

constexpr auto  TX_OUTPUT_POWER            {5};             // dBm

constexpr auto  LORA_BANDWIDTH             {0};             // [0: 125 kHz,
                                                            //  1: 250 kHz,
                                                            //  2: 500 kHz,
                                                            //  3: Reserved]
constexpr auto  LORA_SPREADING_FACTOR      {7};             // [SF7..SF12]
constexpr auto  LORA_CODINGRATE            {1};             // [1: 4/5,
                                                            //  2: 4/6,
                                                            //  3: 4/7,
                                                            //  4: 4/8]
constexpr auto  LORA_PREAMBLE_LENGTH       {8};             // Same for Tx and Rx
constexpr auto  LORA_SYMBOL_TIMEOUT        {0};             // Symbols
constexpr auto  LORA_FIX_LENGTH_PAYLOAD_ON {false};
constexpr auto  LORA_IQ_INVERSION_ON       {false};

constexpr auto  RX_TIMEOUT_VALUE           {1000};
constexpr auto  BUFFER_SIZE                {30};            // Define the payload size here

static RadioEvents_t RadioEvents;

//----------------------------------------------------------------
// Level sensor
//----------------------------------------------------------------
constexpr uint32_t serialSpeed {9600};
constexpr uint8_t  rxPin {5};
constexpr uint8_t  txPin {4};
HardwareSerial hwSerial(1);

//----------------------------------------------------------------
// Water level range
//----------------------------------------------------------------
constexpr uint32_t  distanceMin {150 - 10};
constexpr uint32_t  distanceMax {1660 - 10};

//----------------------------------------------------------------
// Type definitions
//----------------------------------------------------------------


//----------------------------------------------------------------
// Private data
//----------------------------------------------------------------
RTC_DATA_ATTR uint32_t packetCounter {0};
RTC_DATA_ATTR uint32_t err_sensor    {0};
RTC_DATA_ATTR uint32_t err_lora      {0};

static bool packet_transmitted {false};

//----------------------------------------------------------------
// Private functions
//----------------------------------------------------------------
static void vExtOn(void);
static void vExtOff(void);

static void OnTxDone(void);
static void OnTxTimeout(void);

static ErrorOr<uint16_t> ReadDistance(void);
static Error LoraSend(LoraPayload const *payload);


void setup() {
    Serial.begin(115200);
    hwSerial.begin(serialSpeed, SERIAL_8N1, rxPin, txPin);

    Log.begin(LOG_LEVEL_VERBOSE, &Serial);

    pinMode(txPin, OUTPUT);
    digitalWrite(txPin, HIGH);

    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);

    Mcu.begin();
    hwSerial.flush();

     // Start LoRa radio
    RadioEvents.TxDone    = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;

    Radio.Init(&RadioEvents);
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
                      false,                            // freqHopOn
                      0,                                // hopPeriod
                      LORA_IQ_INVERSION_ON,             // iqInverted
                      3000);                            // timeout

    pb_ostream_t ostream;
    LoraPayload payload = LoraPayload_init_zero;

    // while (true) {
        ++packetCounter;
        payload.id = packetCounter;
        payload.err_sensor = err_sensor;
        payload.err_lora   = err_lora;

        // New level readings
        vExtOn();
        digitalWrite(LED, HIGH);
        delay(50);
        digitalWrite(LED, LOW);
        ErrorOr<uint16_t> distance = ReadDistance();
        vExtOff();

        if (distance.Ok()) {
            auto dist = distance.ValueOrDie();
            payload.distance = dist;
            Log.infoln("Distance %dmm", dist);

            // Compute water level based on distace
            auto level = map(dist, distanceMin, distanceMax, 100, 0);
            payload.level = constrain(level, 0, 100);
            Log.infoln("Water Level %d%%", payload.level);

            Error err = LoraSend(&payload);

            if (!err.Ok()) {
                ++err_lora;
            }
        } else {
            ++err_sensor;
        }
#if defined(DEBUG_NO_SLEEP)
        delay(5000);
#else
        esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
        Serial.flush();
        Radio.Sleep();
        esp_deep_sleep_start();
#endif
    // }
}

void loop() {
    /*empty*/
}


static void vExtOn(void) {
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, LOW);
}


static void vExtOff(void) {
    pinMode(Vext, INPUT);
}


static void OnTxDone(void) {
    packet_transmitted = true;
    Log.noticeln("LoRa TX done......");
}


static void OnTxTimeout(void) {
    packet_transmitted = true;
    ++err_lora;
    Log.errorln("LoRa TX Timeout......");
}


/**
 * @brief Reads the distance measurement from a sensor via serial communication.
 * 
 * This function attempts to read the distance up to three times. It sends a signal to the sensor,
 * waits for a response, and reads the data. The function checks for a valid start byte, 
 * verifies the checksum, and returns the distance in millimeters if successful. 
 * In case of errors (timeout or checksum failure), it returns an appropriate error code.
 * 
 * @return ErrorOr<uint16_t> A structure containing either the distance in millimeters 
 *                            or an error code indicating the type of failure encountered.
 */
static ErrorOr<uint16_t> ReadDistance(void) {
    ErrorOr<uint16_t> ret;

    for (uint8_t retries = 0; retries < 3 && !ret.Ok(); retries++) {
        delay(200);

        hwSerial.flush(false);

        digitalWrite(txPin, LOW);
        delay(30);
        digitalWrite(txPin, HIGH);

        uint32_t time = millis();
        bool timeout = false;

        // Wait fot the answer
        while (hwSerial.available() == 0 && timeout == false) {
            if (millis() - time >= 100) {
                timeout = true;
            }
        }

        auto startByte = (byte)hwSerial.read();

        if (startByte != 255) {
            Log.errorln("Distance receive timeout");
            ret = Error::INTERNAL_ERROR;
            continue;
        } else {
            std::array<uint8_t, 9> uartBytes;

            hwSerial.readBytes(uartBytes.data(), 3);
            uint16_t distanceMm = (uint16_t)(uartBytes[0] << 8) + uartBytes[1];

            if (((startByte + uartBytes[0] + uartBytes[1]) & 0xFF) == uartBytes[2]) {
                ret = distanceMm;
            } else {
                Log.errorln("Distance receive checksum error");
                ret = Error::INTERNAL_ERROR;
            }
        }
    }

    return ret;
}

/**
 * @brief Sends a LoRa payload over the radio.
 * 
 * This function encodes the provided LoRaPayload structure into a binary format
 * and transmits it over the radio. The packet is sent three times to ensure
 * reliable transmission. If encoding fails, an error is logged and an internal
 * error code is returned. The function blocks until the packet is successfully 
 * transmitted for each attempt.
 * 
 * @param payload A pointer to the LoRaPayload structure to be sent.
 * 
 * @return Error An error code indicating the result of the operation. 
 *               Returns Error::OK on success, or an internal error code on failure.
 */
static Error LoraSend(LoraPayload const *payload) {
    Error err = Error::OK;
    std::array<uint8_t, BUFFER_SIZE> txpacket;
    txpacket.fill(0);

    pb_ostream_t ostream = pb_ostream_from_buffer(txpacket.data(), txpacket.size());

    if (!pb_encode(&ostream, &LoraPayload_msg, payload)) {
        Log.errorln("Encoding failed: %s\n", PB_GET_ERROR(&ostream));
        err = Error::INTERNAL_ERROR;
    } else {
        packet_transmitted = false;

        // Send the same packet 3 times
        for (int i = 0; i < 3; i++) {
            Radio.Send(txpacket.data(), (uint8_t)ostream.bytes_written);
            packet_transmitted = false;

            while (packet_transmitted == false) {
                Radio.IrqProcess();
            }
            delay(100);
        }
    }

    return err;
}
