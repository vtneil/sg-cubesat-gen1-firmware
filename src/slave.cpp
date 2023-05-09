#include <Arduino.h>

#include "vnet_lora.h"
#include "vnet_tools.h"

/* Pins Configuration */
#define PIN_LORA_M0     (6)
#define PIN_LORA_M1     (7)
#define PIN_BUZZER      (8)
#define GPS_TX_TO_RX    (2)
#define GPS_RX_TO_TX    (3)

//#define SWAP
//#define USE_GPS

/* Global Variables and Typedefs */
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
AltSoftSerial SerialLoRa;
#ifdef USE_GPS
NeoSWSerial SerialGPS(GPS_RX_TO_TX, GPS_TX_TO_RX);
#endif
#endif

#ifdef USE_GPS
TinyGPSPlus gps;

typedef struct {
    float lat;
    float lon;
    float alt;
} GPS_Data_t;

GPS_Data_t gps_data = {};
String msg_gps = "";
#endif

String msg_lora = "";

LoRa_E32<HardwareSerial> lora(&Serial2, 9600U, 6, 7);

/* Functions */

void setup() {
    /* Pin Modes */

    /* Init Serial and Buzzer Test */
    Serial.begin(115200);

    lora.begin_normal();

    delay(1000);
}

void loop() {
    /* LoRa Read and Write */
    while (Serial2.available()) {
        Serial.write(Serial2.read());
    }

    if (Serial.available()) {
        delay(10);
        lora.begin_cfg();
        lora.cmd_get_params();
        lora.print_params();

        uint8_t ch = 0;
        while (Serial.available()) {
            ch *= 10;
            ch += Serial.read() - '0';
        }

        if (ch == 255) {
            delay(100);
            lora.begin_normal();
            return;
        }

        lora.cmd_set_params(0, LoRaCFG::LORA_BAUD_9600, LoRaCFG::LORA_8N1,
                            LoRa_E32<>::LORA_RATE_2400, ch, LoRaCFG::LORA_TX_MAX,
                            false, true);
        lora.cmd_write_params();
        delay(100);
        lora.cmd_get_params();
        lora.print_params();
        lora.begin_normal();
        delay(100);
    }
}
