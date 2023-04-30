#include <Arduino.h>
#include "Adafruit_BME280.h"
#include "vnet_tools.h"

Adafruit_BME280 bme;

void setup() {
    Serial.begin(115200U);
    randomSeed(1238414);
}

void loop() {
    static unsigned long counter = 0;
    Serial.println(build_string(
            ++counter,
            random(1000, 1010),
            random(30, 70),
            random(20, 40)));

    delay(500);
}
