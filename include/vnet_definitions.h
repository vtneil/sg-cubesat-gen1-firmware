#ifndef VNET_DEFINITIONS_H
#define VNET_DEFINITIONS_H

#include <stdint.h>

#define USB_VENDOR_ID        0x239A
#define USB_PRODUCT_ID       0x8023
#define USB_MANUFACTURER_STR "Adafruit Industries"
#define USB_PRODUCT_STR      "CG CubeSat Main Board"
#define USB_SERIAL_NUM_STR   "SPACEAC0001"

#define UART0_BAUD        (115200)
#define UART0_PARITY      SERIAL_8N1
#define UART0_BUFFER_SIZE (64)
#define CONFIG_SIZE       (256)

#if defined(ARDUINO_TEENSY40)
#define EEPROM_CFG_START (1080-CONFIG_SIZE)
#elif defined(ARDUINO_TEENSY41)
#define EEPROM_CFG_START (4284-CONFIG_SIZE)
#elif defined(__AVR_ATmega2560__)
#define EEPROM_CFG_START (4096-CONFIG_SIZE)
#elif defined(__AVR_ATmega328P__)
#define EEPROM_CFG_START (1024-CONFIG_SIZE)
#else
#define EEPROM_CFG_START (0)
#endif

/** AVR ATmega2560 Pin Macros */
#define PIN_D0  (0)
#define PIN_D1  (1)
#define PIN_D2  (2)
#define PIN_D3  (3)
#define PIN_D4  (4)
#define PIN_D5  (5)
#define PIN_D6  (6)
#define PIN_D7  (7)
#define PIN_D8  (8)
#define PIN_D9  (9)
#define PIN_D10 (10)
#define PIN_D11 (11)
#define PIN_D12 (12)
#define PIN_D13 (13)
#define PIN_D14 (14)
#define PIN_D15 (15)
#define PIN_D16 (16)
#define PIN_D17 (17)
#define PIN_D18 (18)
#define PIN_D19 (19)
#define PIN_D20 (20)
#define PIN_D21 (21)
#define PIN_D22 (22)
#define PIN_D23 (23)
#define PIN_D24 (24)
#define PIN_D25 (25)
#define PIN_D26 (26)
#define PIN_D27 (27)
#define PIN_D28 (28)
#define PIN_D29 (29)
#define PIN_D30 (30)
#define PIN_D31 (31)
#define PIN_D32 (32)
#define PIN_D33 (33)
#define PIN_D34 (34)
#define PIN_D35 (35)
#define PIN_D36 (36)
#define PIN_D37 (37)
#define PIN_D38 (38)
#define PIN_D39 (39)
#define PIN_D40 (40)
#define PIN_D41 (41)
#define PIN_D42 (42)
#define PIN_D43 (43)
#define PIN_D44 (44)
#define PIN_D45 (45)
#define PIN_D46 (46)
#define PIN_D47 (47)
#define PIN_D48 (48)
#define PIN_D49 (49)
#define PIN_D50 (50)
#define PIN_D51 (51)
#define PIN_D52 (52)
#define PIN_D53 (53)
#define PIN_D54 (54)

/** Default UART Pin */
#define UART0_RXD PIN_D0
#define UART0_TXD PIN_D1

/** AVR ATmega2560 UART Pin Macros */
#define UART1_RXD PIN_D19
#define UART1_TXD PIN_D18
#define UART2_RXD PIN_D17
#define UART2_TXD PIN_D16
#define UART3_RXD PIN_D15
#define UART3_TXD PIN_D14

/** Millis Standardized Macros */
#define MILLIS_10   (10)
#define MILLIS_20   (20)
#define MILLIS_30   (30)
#define MILLIS_40   (40)
#define MILLIS_50   (50)
#define MILLIS_100  (100)
#define MILLIS_200  (200)
#define MILLIS_250  (250)
#define MILLIS_400  (400)
#define MILLIS_500  (500)
#define MILLIS_800  (800)
#define MILLIS_1S   (1000)
#define MILLIS_2S   (2000)
#define MILLIS_2_5S (2500)
#define MILLIS_3S   (3000)
#define MILLIS_4S   (4000)
#define MILLIS_5S   (5000)
#define MILLIS_8S   (8000)
#define MILLIS_10S  (10000)

/** Communication Frequency Channel Macros */
#define LORA_433_CH1  (410000000)
#define LORA_433_CH2  (411000000)
#define LORA_433_CH3  (412000000)
#define LORA_433_CH4  (413000000)
#define LORA_433_CH5  (414000000)
#define LORA_433_CH6  (415000000)
#define LORA_433_CH7  (416000000)
#define LORA_433_CH8  (417000000)
#define LORA_433_CH9  (418000000)
#define LORA_433_CH10 (419000000)
#define LORA_433_CH11 (420000000)
#define LORA_433_CH12 (421000000)
#define LORA_433_CH13 (422000000)
#define LORA_433_CH14 (423000000)
#define LORA_433_CH15 (424000000)
#define LORA_433_CH16 (425000000)
#define LORA_433_CH17 (426000000)
#define LORA_433_CH18 (427000000)
#define LORA_433_CH19 (428000000)
#define LORA_433_CH20 (429000000)
#define LORA_433_CH21 (430000000)
#define LORA_433_CH22 (431000000)
#define LORA_433_CH23 (432000000)
#define LORA_433_CH24 (433000000)
#define LORA_433_CH25 (434000000)
#define LORA_433_CH26 (435000000)
#define LORA_433_CH27 (436000000)
#define LORA_433_CH28 (437000000)
#define LORA_433_CH29 (438000000)
#define LORA_433_CH30 (439000000)
#define LORA_433_CH31 (440000000)
#define LORA_433_CH32 (441000000)

#define LORA_915_CH1  (900000000)
#define LORA_915_CH2  (901000000)
#define LORA_915_CH3  (902000000)
#define LORA_915_CH4  (903000000)
#define LORA_915_CH5  (904000000)
#define LORA_915_CH6  (905000000)
#define LORA_915_CH7  (906000000)
#define LORA_915_CH8  (907000000)
#define LORA_915_CH9  (908000000)
#define LORA_915_CH10 (909000000)
#define LORA_915_CH11 (910000000)
#define LORA_915_CH12 (911000000)
#define LORA_915_CH13 (912000000)
#define LORA_915_CH14 (913000000)
#define LORA_915_CH15 (914000000)
#define LORA_915_CH16 (915000000)
#define LORA_915_CH17 (916000000)
#define LORA_915_CH18 (917000000)
#define LORA_915_CH19 (918000000)
#define LORA_915_CH20 (919000000)
#define LORA_915_CH21 (920000000)
#define LORA_915_CH22 (921000000)
#define LORA_915_CH23 (922000000)
#define LORA_915_CH24 (923000000)
#define LORA_915_CH25 (924000000)
#define LORA_915_CH26 (925000000)
#define LORA_915_CH27 (926000000)
#define LORA_915_CH28 (927000000)
#define LORA_915_CH29 (928000000)
#define LORA_915_CH30 (929000000)
#define LORA_915_CH31 (930000000)
#define LORA_915_CH32 (931000000)

/** Sensors Macros and Addresses */
#define ADDR_BME280   (0x76)
#define ADDR_CCS811   (0x5A)
#define ADDR_BNO085   (0x4A)
#define ADDR_BNO055   (0x28)
#define ADDR_ICM20948 (0x68)

/** Other Macros */
#define WEAK __attribute__((weak))
#define PAUSE_INTERRUPT(BLOCK) cli(); BLOCK sei();
#define BYTES_2(_B1, _B0) ((_B1<<1)+_B0)
#define IF_FLAG_DO(FLAG, BLOCK) if(FLAG){BLOCK FLAG=false;}
#define INCREMENT_AND_COMPARE(TIM_CNT, FLAG, INTERVAL) if(++TIM_CNT==INTERVAL){FLAG=true;TIM_CNT=0;}
#define DO_LOOP(ITERATION, CMD) for(size_t _i=0;_i<ITERATION;++_i) CMD

#define SQUARE(X) (X*X)
#define MAP(X,IN_MIN,IN_MAX,OUT_MIN,OUT_MAX) ((X-IN_MIN)*(OUT_MAX-OUT_MIN)/(IN_MAX-IN_MIN)+OUT_MIN)
#define CLAMP CONSTRAIN
#define CONSTRAIN(X,X_MIN,X_MAX) (X<X_MIN?X_MIN:X>X_MAX?X_MAX:X)

typedef enum {
    SYS_DISABLED = 0,
    SYS_BOOT_UP,
    SYS_SETUP,
    SYS_MAIN,
    SYS_DFU,
    DEBUG
} State_ID;

typedef struct {
    bool env_bme280[2];
    bool env_ccs811;
    bool env_gy8511;
    bool env_pms7003;
    bool imu_bno085;
    bool imu_bno055;
    bool imu_icm20948;
    bool gps_SparkFun;
    bool gps_ATGM336H;
} Peripherals_t;

typedef struct {
    uint8_t mfg_dd;
    uint8_t mfg_mm;
    uint16_t mfg_yyyy;
    uint16_t model_number;
    uint16_t serial_number;
    uint16_t firmware_version;
    uint32_t boot_count;
} EEPROM_Device_t;

typedef struct {
    uint32_t loop_interval;
    Peripherals_t peripherals;
} EEPROM_Config_t;

typedef uint16_t tim_cnt_t[8];

typedef bool flag_t[8];

typedef void (*HandlerFunc_Arg_t)(void *arg);

typedef void (*HandlerFunc_t)();

int16_t averageAnalogRead(int pinToRead) {
    static uint8_t num_samples = 8;
    int16_t accum = 0;

    for (int x = 0; x < num_samples; x++) {
        accum += analogRead(pinToRead);
    }
    return accum / num_samples;
}

/**
 * Variadic Argument Template
 * va_list args;
 * va_start(args, arg);
 *
 * while (arg != nullptr) {
 *     arg = va_arg(args, void *);
 *     UART0.println(*((float *) arg));
 * }
 *
 * va_end(args);
 */

#endif //VNET_DEFINITIONS_H
