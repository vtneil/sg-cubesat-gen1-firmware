/*
 * @file main.cpp
 *
 * Firmware version 0.1 for Project CubeSat SG (Main Board)
 *
 * This is the source code for CubeSat SG (Main Board)'s proprietary firmware.
 *
 * @author Vivatsathorn Thitasirivit
 * Contact: www.vt.in.th
 */

/* Includes */
#include <Arduino.h>
#include <avr/wdt.h>
#include "vnet_definitions.h"
#include "vnet_tools.h"
#include "vnet_states.h"
#include "vnet_sensors.h"

/* Programming Options */
#define DEVICE_NUMBER 0
//#define DEVICE_SETUP_MODE
#define DEVICE_DEBUG_MODE

/* Definitions */
#define PIN_LED          PIN_D10
#define PIN_BUZZER       PIN_D45
#define PIN_WDT_Activate PIN_D47
#define PIN_WDT_PWM      PIN_D46
#define PIN_ADC_BATT     PIN_A0
#define PIN_ADC_CAM      PIN_A1
#define PIN_ADC_SENSE    PIN_A2
#define PIN_ADC_C_OUT    PIN_A3
#define PIN_EXT_TEMP     PD4

#define SerialGPS1 Serial1
#define SerialLoRa Serial2

#define FLAG_SET_CNT 2

/* Macros */
#ifdef DEVICE_DEBUG_MODE
#define LOG(CMD) CMD
#else
#define LOG(CMD)
#endif

/* Typedefs */
typedef struct {
    String device_name;
    uint32_t counter;
    Sensors_Data_GPS_Position_t pos0;
    Sensors_Data_GPS_Position_t pos1;
    Sensors_Data_PHT_t pht;
    float ext_temperature;
    float battery_v;
} Data_t;

/* Functions Declarations */
extern inline void setup_wdt();

extern void init_peripherals();

extern void timer_increment();

extern inline void user_uart_rx();

extern void dfu_mode_handler();

extern void boot_handler();

extern void setup_handler();

extern void main_loop_handler();

/* Device Configurations and Parameters */
EEPROM_Config_t device_params = {
        .loop_interval = MILLIS_1S
};

/* Global Variables */
Peripherals_t sensors_list = {};
Sensors_Environmental *env_sensors;
Sensors_GPS *gps0;
Sensors_GPS *gps1;
Sensors_Environmental_DS18B20 *ext_sense;

Data_t data = {};
String packet;

volatile tim_cnt_t tim_cnt[FLAG_SET_CNT] = {{}};
volatile flag_t flag[FLAG_SET_CNT] = {{}};

uint8_t uart0_rx_buf[UART0_BUFFER_SIZE];
uint8_t uart0_rx_ptr = 0;

State *state = nullptr;
State boot_state = OSState(boot_handler, StateID_e::SYS_BOOT_UP);
State setup_state = OSState(setup_handler, StateID_e::SYS_SETUP);
State main_state = OSState(main_loop_handler, StateID_e::SYS_MAIN);
State dfu_state = OSState(dfu_mode_handler, StateID_e::SYS_DFU);

State *setup_states_list[] = {&boot_state, &setup_state};

void setup() {
    /* Run Setups: boot init and setup variables */
    for (State *&s: setup_states_list) {
        s->run();
    }
    state = &main_state;
}

void loop() {
    /* Begin Run current state */
    state->run();
    /* End Run current state */

    /* Begin Poll for Serial */
    user_uart_rx();
    /* End Poll for Serial */
}

/* Functions Definitions */

void user_uart_rx() {
    IF_FLAG_DO(flag[1][0], {
        if (Serial.available()) {
            /* Clear Serial Rx buffer and pointer */
            uart0_rx_ptr = 0;
            memset(uart0_rx_buf, 0, sizeof(uart0_rx_buf));

            /* Copy to buffer */
            while (Serial.available()) {
                if (uart0_rx_ptr >= sizeof(uart0_rx_buf)) {
                    memset(uart0_rx_buf, 0, sizeof(uart0_rx_buf));
                    break;
                }
                uart0_rx_buf[uart0_rx_ptr++] = (uint8_t) Serial.read();
            }

            /* Compare buffer, then change mode */
            if (CMP_STATE(uart0_rx_buf, STATE_DFU_LOOP)) {
                state = &dfu_state;
            } else if (CMP_STATE(uart0_rx_buf, STATE_MAIN_LOOP)) {
                state = &main_state;
            }
        }
    })
}

/* Begin Handler Functions Definitions */
void setup_handler() {
    /* Begin void setup */
    init_peripherals();
    /* End void setup */

    /* Begin init Hardware Auto-Reset */
    analogWrite(PIN_WDT_PWM, 1 << 7);
    digitalWrite(PIN_WDT_Activate, 0);
    delay(100);
    digitalWrite(PIN_WDT_Activate, 1);
    /* End init Hardware Auto-Reset */
}

void main_loop_handler() {
    /* Begin loop */
    /* MAIN: Every 1000 ms */
    IF_FLAG_DO(flag[0][0], {
        ++data.counter;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

        packet = build_string(free_memory(),
                              data.device_name,
                              data.counter,
                              String(data.pos0.latitude, 6),
                              String(data.pos0.longitude, 6),
                              data.pos0.altitude,
                              String(data.pos1.latitude, 6),
                              String(data.pos1.longitude, 6),
                              data.pos1.altitude,
                              data.pht.altitude,
                              data.pht.temperature,
                              data.pht.humidity,
                              data.pht.pressure,
                              data.ext_temperature,
                              data.battery_v
        );

        Serial.println(packet);

        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    })

    /* SENSE: Every 50 ms */
    IF_FLAG_DO(flag[0][1], {
        data.pht = env_sensors->read_PHT();
        data.pos0 = gps0->read_position();
        data.pos1 = gps1->read_position();
        data.ext_temperature = ext_sense->read_temperature();
        data.battery_v = analogRead(PIN_ADC_BATT);
    })
    /* End loop */
}

void boot_handler() {
    /* Begin Serial USB */
    Serial.begin(UART0_BAUD, UART0_PARITY);
    Serial.println("Boot init...");
    /* End Serial USB */

    /* Begin WDT */
    setup_wdt();
    /* End WDT */

    data.device_name = String("CGSAT") + String(DEVICE_NUMBER);
}

void dfu_mode_handler() {
    Serial.println("LoRa Setup Mode");
}

/* End Handler Functions Definitions */
void init_peripherals() {
    Serial.println("Peripheral init...");
    /* Begin Pins */
    DDRH |= (1 << PH2) | (1 << PH3);        // PH2, PH3 Output   for LoRa Mode
    PORTH &= ~((1 << PH2) | (1 << PH3));    // PH2, PH3 Set Low  for LoRa Mode

    DDRD &= ~(1 << PIN_EXT_TEMP);                      // PD4 for Temperature Sensor

    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_WDT_Activate, OUTPUT);
    pinMode(PIN_WDT_PWM, OUTPUT);
    /* End Pins */

    /* Begin Sensors */
    env_sensors = new Sensors_Environmental(&sensors_list);
    gps0 = new Sensors_GPS_SparkFun(&sensors_list);
    gps1 = new Sensors_GPS_Serial(&sensors_list, &SerialGPS1);
    ext_sense = new Sensors_Environmental_DS18B20(&sensors_list);
    /* End Sensors */

    /* Begin Interface */
    SerialLoRa.begin(9600);
    /* End Interface */
}

void timer_increment() {
    /* Begin User Timers and Flags */
    /* Main Loop Timer */
    INCREMENT_AND_COMPARE(tim_cnt[0][0], flag[0][0], device_params.loop_interval)
    INCREMENT_AND_COMPARE(tim_cnt[0][1], flag[0][1], MILLIS_50)
    /* End User Timers and Flags */

    /* Begin Reserved Timers and Flags */
    /* Serial Polling Timer at 100 ms */
    INCREMENT_AND_COMPARE(tim_cnt[1][0], flag[1][0], MILLIS_100)
    /* DFU Loop Timer at 500 ms */
    INCREMENT_AND_COMPARE(tim_cnt[1][6], flag[1][6], MILLIS_500)
    /* End Reserved Timers and Flags */
}

void setup_wdt() {
    /* Begin Setup millisecond Timer */
    cli();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    TCCR1B |= (1 << CS11) | (1 << CS10);    // Clock Select = 011 (Prescaler clk/64) 16 MHz / 64 = 250 kHz
    TCCR1B |= (1 << WGM12);                 // Operating Mode: CTC (Clear Timer on Compare match)
    OCR1A = 250;                            // Set Output Compare Register A value (250 kHz / 250 = 1 kHz or 1 ms)
    TIMSK1 |= (1 << OCIE1A);                // Set Timer to Output Compare Register A
    sei();
    /* End Setup millisecond Timer */

    /* Begin Enable Watchdog Timer for 4 seconds */
    wdt_enable(WDTO_4S);
    /* End Enable Watchdog Timer for 4 seconds */
}

/* Begin Interrupt Service Routine Definitions */
ISR(TIMER1_COMPA_vect) {
    timer_increment();
    /* Watchdog Timer Reset */
    if (++tim_cnt[1][7] == MILLIS_1S) {
        wdt_reset();
        tim_cnt[1][7] = 0;
    }
}
/* End Interrupt Service Routine Definitions */
