/*
 * @file main.cpp
 *
 * Firmware version 0.1 for Project CubeSat SG (Main Board)
 *
 * This is the source code for CubeSat SG (Main Board)'s proprietary firmware.
 *
 * @author Vivatsathorn Thitasirivit
 * Contact: www.vt.in.th
 *
 * Boot & States Sequence
 * 1. Boot State: Serial and Screen
 * 2. Setup State: Init peripherals and Auto-reset and WDT
 */

/* Includes */
#include <Arduino.h>
#include <avr/wdt.h>
#include "vnet_definitions.h"
#include "vnet_tools.h"
#include "vnet_states.h"

#define ONEWIRE_CUSTOM_PIN

#include "vnet_sensors.h"

/* Device Parameters */
#define DEVICE_NUMBER 0
#define LORA_CHANNEL 0
#define FILENAME ""

/* Programming Options */
//#define DEVICE_SETUP_MODE
#define DEVICE_DEBUG_MODE
//#define ENABLE_HW_RESET
#define OPTIMIZE_GPS_COMPILE

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
#if defined(DEVICE_DEBUG_MODE)
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
    Sensors_Data_PHT_t pht_ext;
    float ext_temperature;
    float battery_v;
} Data_t;

/* Functions Declarations */
extern inline void setup_wdt();

extern inline void init_peripherals();

extern inline void timer_increment();

extern inline void user_uart_rx();

extern void lora_cfg_handler();

extern void boot_handler();

extern void setup_handler();

extern void main_loop_handler();

extern void sd_read();

extern void display_data();

extern inline void sig_trap();

extern inline void reset_device();

/* Device Configurations and Parameters */
EEPROM_Config_t device_params = {
        .loop_interval = MILLIS_1S
};

/* Global Variables */
Peripherals_t sensors_list = {};
LoRa_E32 *lora;
Sensors_Environmental *env_sensors;
Sensors_Environmental *env_sensors_ext;
#if defined(OPTIMIZE_GPS_COMPILE)
Sensors_GPS_SparkFun *gps0;
Sensors_GPS_Serial *gps1;
#else
Sensors_GPS *gps0;
Sensors_GPS *gps1;
#endif
Sensors_Environmental_DS18B20 *ext_sense;
Storage_SD *sd;
Screen_OLED *screen;

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
State lora_cfg_state = OSState(lora_cfg_handler, StateID_e::SYS_DFU);
State sd_read_state = OSState(sd_read, StateID_e::SYS_DFU);
State sink_state = OSState(sig_trap);

void setup() {
    /* Run Setups: boot init and setup variables */
    boot_state.run();
    setup_state.run();
    state = &main_state;
}

void loop() {
    /* Begin Run current state */
    state->run();
    /* End Run current state */

    /* Begin Poll for Serial */
    if (state == &main_state)
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
            switch (BYTES_2(uart0_rx_buf[0], uart0_rx_buf[1])) {
                case State::MAIN_LOOP:
                    state = &main_state;
                    break;
                case State::DFU_LOOP:
                    state = &lora_cfg_state;
                    break;
                case State::SD_READ:
                    state = &sd_read_state;
                    break;
                case State::FORCE_RESET:
                    reset_device();
                    break;
                default:
                    break;
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
#if defined(ENABLE_HW_RESET)
    analogWrite(PIN_WDT_PWM, 1 << 7);
    digitalWrite(PIN_WDT_Activate, 0);
    delay(100);
    digitalWrite(PIN_WDT_Activate, 1);
#endif
    /* End init Hardware Auto-Reset */

    /* Begin WDT */
    setup_wdt();
    /* End WDT */
}

void main_loop_handler() {
    /* Begin loop */
    /* MAIN: Every 1000 ms */
    static uint32_t time_now = millis();
    IF_FLAG_DO(flag[0][0], {
        ++data.counter;
        packet = build_string(data.device_name,
                              data.counter,
                              millis() - time_now,
                              free_memory(),
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
                              data.pht_ext.altitude,
                              data.pht_ext.temperature,
                              data.pht_ext.humidity,
                              data.pht_ext.pressure,
                              data.ext_temperature,
                              data.battery_v);
        time_now = millis();

        /* Log and Write Data */
        LOG(Serial.println(packet));
        SerialLoRa.println(packet);
        sd->file.println(packet);
        sd->flush();

        /* Display Data on Screen */
        display_data();

        /* Toggle LED in each packet */
        digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    })

    /* SENSE: Every 50 ms */
    IF_FLAG_DO(flag[0][1], {
        data.pht = env_sensors->read_PHT();
        data.pht_ext = env_sensors_ext->read_PHT();
        data.pos0 = gps0->read_position();
        data.pos1 = gps1->read_position();
        data.battery_v = (float) (0.0051 * 4 * analogRead(PIN_ADC_BATT));
    })

    /* SENSE Dallas: Every 2000 ms */
    IF_FLAG_DO(flag[0][2], {
        data.ext_temperature = ext_sense->read_temperature();
    })
    /* End loop */
}

void boot_handler() {
    /* Begin Serial USB */
    Serial.begin(UART0_BAUD, UART0_PARITY);
    /* End Serial USB */

    /* Begin Screen */
    screen = new Screen_OLED();
    /* End Screen */

    /* Delay (Wait for Potential Uploading) */
    delay(MILLIS_3S);

    /* Clear Booting Screen */
    screen->clear();

    /* Set Device Name */
    data.device_name = String("CG") + String(DEVICE_NUMBER);
}

void lora_cfg_handler() {
    lora->begin_cfg();

    lora->cmd_get_params();

    Serial.println("==== LoRa Current Configuration  ====");

    lora->print_params();

    Serial.println("=== LoRa Begin Configuration Mode ===");

    lora->cmd_set_params(0,
                         LoRa_E32::LORA_BAUD_115200,
                         LoRa_E32::LORA_8N1,
                         LoRa_E32::LORA_RATE_9600,
                         15, LoRa_E32::LORA_TX_MAX,
                         false, true);

    lora->cmd_write_params();

    lora->print_params();

    Serial.println("==== LoRa End Configuration Mode ====");

    sig_trap();
}

/* End Handler Functions Definitions */

void init_peripherals() {
    /* Begin Pins */
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);

    pinMode(PIN_ADC_BATT, INPUT);
    /* End Pins */

    /* Begin LoRa */
    lora = new LoRa_E32(&SerialLoRa, 115200U, DDRH, DDRH, PORTH, PORTH, PH2, PH3);
    /* End LoRa */

    /* Begin SD */
    sd = new Storage_SD(FILENAME);
    /* End SD */

    /* Begin Sensors */
    env_sensors = new Sensors_Environmental(&sensors_list);
    env_sensors_ext = new Sensors_Environmental(&sensors_list, 0x77);
    gps0 = new Sensors_GPS_SparkFun(&sensors_list);
    gps1 = new Sensors_GPS_Serial(&sensors_list, &SerialGPS1);
    ext_sense = new Sensors_Environmental_DS18B20(&sensors_list);
    /* End Sensors */

    /* Begin Interface */
    lora->begin_normal(115200U);
    /* End Interface */

#if defined(ENABLE_HW_RESET)
    pinMode(PIN_WDT_Activate, OUTPUT);
    pinMode(PIN_WDT_PWM, OUTPUT);
#endif
}

void sd_read() {
    static String filename;
    static uint8_t mode = 0;

    filename = "";
    while (!Serial.available());
    delay(100);
    while (Serial.available()) filename += (char) Serial.read();
    filename.trim();

    if (filename == "quit" || filename == "exit") {
        LOG(Serial.println("Returning to Normal..."));
        sd->open();
        state = &main_state;
        return;
    } else if (filename == "delete") {
        LOG(Serial.println("Delete Mode: Listing Directory..."));
        mode = 1;
    } else if (filename == "delete-all") {
        LOG(Serial.println("Delete All Mode"));
        sd->delete_all();
        sd->close();
        return;
    } else {
        LOG(Serial.println("Listing Directory..."));
        mode = 0;
    }

    sd->close();
    sd->list_files();

    LOG(Serial.println("Input file name..."));

    while (!Serial.available());
    delay(100);
    filename = "";
    while (Serial.available()) filename += (char) Serial.read();
    filename.trim();

    LOG(Serial.println(filename));

    if (mode == 0)
        sd->read_content(filename);
    else if (mode == 1)
        sd->delete_file(filename);
}

void display_data() {
//            data.device_name,
//            String(data.pos0.latitude, 6),
//            String(data.pos0.longitude, 6),
//            data.pos0.altitude,
//
//            String(data.pos1.latitude, 6),
//            String(data.pos1.longitude, 6),
//            data.pos1.altitude,
//
//            data.pht.altitude,
//            data.pht.temperature,
//            data.pht.humidity,
//            data.pht.pressure,
//            data.pht_ext.altitude,
//            data.pht_ext.temperature,
//
//            data.pht_ext.humidity,
//            data.pht_ext.pressure,
//            data.ext_temperature,
//            data.battery_v

    if (!screen->valid()) return;
    screen->clear();
    screen->display->setCursor(0, 7);
    screen->display->println("Device Name: " + data.device_name);
    screen->display->println("Battery: " + String(data.battery_v) + " V");
    screen->display->println("------------------------------");
    screen->display->println("Pos 0: " + build_string(String(data.pos0.latitude, 6),
                                                      String(data.pos0.longitude, 6),
                                                      data.pos0.altitude));
    screen->display->println("Pos 1: " + build_string(String(data.pos1.latitude, 6),
                                                      String(data.pos1.longitude, 6),
                                                      data.pos1.altitude));
    screen->display->println("------------------------------");
    screen->display->println("PHT+A 0: " + build_string(data.pht.pressure,
                                                        data.pht.humidity,
                                                        data.pht.temperature,
                                                        data.pht.altitude));
    screen->display->println("PHT+A 1: " + build_string(data.pht_ext.pressure,
                                                        data.pht_ext.humidity,
                                                        data.pht_ext.temperature,
                                                        data.pht_ext.altitude));
    screen->display->println("------------------------------");
    screen->display->println("Ext Temp Probe: " + String(data.ext_temperature));
    screen->display->display();
}

void timer_increment() {
    /* Begin User Timers and Flags */
    /* Main Loop Timer */
    INCREMENT_AND_COMPARE(tim_cnt[0][0], flag[0][0], device_params.loop_interval)
    /* Sense Timer */
    INCREMENT_AND_COMPARE(tim_cnt[0][1], flag[0][1], MILLIS_50)
    /* Dallas Timer */
    INCREMENT_AND_COMPARE(tim_cnt[0][2], flag[0][2], MILLIS_1S)
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

void sig_trap() {
    delay(20);
    while (true);
}

void reset_device() {
    wdt_disable();
    wdt_enable(WDTO_15MS);
    PAUSE_INTERRUPT(sig_trap();)
}
