#include <Arduino.h>

#include "vnet_lora.h"
#include "NeoSWSerial.h"
#include "AltSoftSerial.h"
#include "TinyGPS++.h"

/* Pins Configuration */
#define PIN_LORA_M0     (6)
#define PIN_LORA_M1     (7)
#define PIN_BUZZER      (8)
#define GPS_TX_TO_RX    (2)
#define GPS_RX_TO_TX    (3)

//#define SWAP

/* Global Variables and Typedefs */
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
NeoSWSerial SerialGPS(GPS_RX_TO_TX, GPS_TX_TO_RX);
AltSoftSerial SerialLoRa;
#endif

TinyGPSPlus gps;

typedef struct {
    float lat;
    float lon;
    float alt;
} GPS_Data_t;

GPS_Data_t gps_data = {};
String msg_lora = "";
String msg_gps = "";

/* Functions */
template<typename T>
String build_string(T last) {
    return String(last);
}

template<typename T, typename... Args>
String build_string(T first, Args... args) {
    String s0 = "";
    s0 += String(first) + "," + build_string(args...);
    return s0;
}

void setup() {
    /* Pin Modes */
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_LORA_M0, OUTPUT);
    pinMode(PIN_LORA_M1, OUTPUT);

    /* LoRa M0, M1 Active Low Mode */
    digitalWrite(PIN_LORA_M0, 0);
    digitalWrite(PIN_LORA_M1, 0);

    /* Init Serial and Buzzer Test */
    digitalWrite(PIN_BUZZER, 1);
    Serial.begin(9600);
    SerialGPS.begin(9600);
    SerialLoRa.begin(9600);

    Serial.println("Starting...");

    delay(1000);
    digitalWrite(PIN_BUZZER, 0);
}

void loop() {
//    static bool found = false;
//    static unsigned long t_last = millis();
//    static unsigned long t_gps = millis();

    /* LoRa Read and Write */
    while (SerialLoRa.available()) {
        Serial.write(Serial.read());
//        msg_lora += (char) SerialLoRa.read();
    }

//    if (msg_lora.indexOf('\n') >= 0) {
//        found = true;
//        t_last = millis();
//        digitalWrite(PIN_BUZZER, 1);
//        Serial.print(msg_lora);
//        msg_lora = "";
//    }

    /* GPS Read and Write */
//    while (SerialGPS.available()) {
//        static char c = 0;
//        c = (char) SerialGPS.read();
//        gps.encode(c);
//    }
//
//    if (gps.location.isValid() || gps.satellites.isUpdated()) {
//        gps_data = {
//                (float) gps.location.lat(),
//                (float) gps.location.lng(),
//                (float) gps.altitude.meters()
//        };
//    }

//    if (millis() - t_gps > 1000) {
//        Serial.print("GS_GPS,");
//        Serial.print(gps_data.lat);
//        Serial.print(",");
//        Serial.print(gps_data.lon);
//        Serial.print(",");
//        Serial.print(gps_data.alt);
//        Serial.println();
//        t_gps = millis();
//    }

    /* Reset Buzzer */
//    if (found && millis() - t_last > 100) {
//        digitalWrite(PIN_BUZZER, 0);
//        found = false;
//        t_last = millis();
//    }
//    delay(1);
}
