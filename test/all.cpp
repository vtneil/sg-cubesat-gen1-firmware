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
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#ifndef PINTO_V1_FIRMWARE_VNET_DEFINITIONS_H
#define PINTO_V1_FIRMWARE_VNET_DEFINITIONS_H

#define UART0_BAUD (115200)
#define UART0_PARITY SERIAL_8N1
#define UART0_BUFFER_SIZE (64)
#define CONFIG_SIZE (256)

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

/* AVR ATmega2560 Pin Macros */
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

/* Default UART Pin */
#define UART0_RXD PIN_D0
#define UART0_TXD PIN_D1

/* AVR ATmega2560 UART Pin Macros */
#define UART1_RXD PIN_D19
#define UART1_TXD PIN_D18
#define UART2_RXD PIN_D17
#define UART2_TXD PIN_D16
#define UART3_RXD PIN_D15
#define UART3_TXD PIN_D14

/* Millis Standardized Macros */
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
#define MILLIS_4S   (4000)
#define MILLIS_5S   (5000)
#define MILLIS_8S   (8000)
#define MILLIS_10S  (10000)

/* Communication Frequency Channel Macros */
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

/* Sensors Macros and Addresses */
#define ADDR_BME280 (0x76)
#define ADDR_CCS811 (0x5A)
#define ADDR_BNO085 (0x4A)
#define ADDR_BNO055 (0x28)
#define ADDR_ICM20948 (0x68)

/* Other Macros */
#define WEAK __attribute__((weak))
#define PAUSE_INTERRUPT(BLOCK) cli(); BLOCK sei();
#define BYTES_2(_B1, _B0) ((_B1<<1)+_B0)
#define IF_FLAG_DO(FLAG, BLOCK) if(FLAG){BLOCK FLAG=false;}
#define INCREMENT_AND_COMPARE(TIM_CNT, FLAG, INTERVAL) if(++TIM_CNT==INTERVAL){FLAG=true;TIM_CNT=0;}
#define DO_LOOP(ITERATION, CMD) for(size_t _i=0;_i<ITERATION;++_i) CMD

typedef enum {
    SYS_DISABLED = 0,
    SYS_BOOT_UP,
    SYS_SETUP,
    SYS_MAIN,
    SYS_DFU,
    DEBUG
} StateID_e;

typedef struct {
    bool env_bme280;
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

template<typename T>
T m_square(T x) {
    return x * x;
}

template<typename T>
T map_val(T x, T in_min, T in_max, T out_min, T out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int16_t averageAnalogRead(int pinToRead) {
    static uint8_t num_samples = 8;
    int16_t accum = 0;

    for (int x = 0; x < num_samples; x++) {
        accum += analogRead(pinToRead);
    }
    return accum / num_samples;
}

/*
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

#endif //PINTO_V1_FIRMWARE_VNET_DEFINITIONS_H

#ifndef PINTO_V1_FIRMWARE_VNET_TOOLS_H
#define PINTO_V1_FIRMWARE_VNET_TOOLS_H

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

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

size_t free_memory() {
    char top;
#ifdef __arm__
    return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
    return &top - __brkval;
#else  // __arm__
    return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}

#endif //PINTO_V1_FIRMWARE_VNET_TOOLS_H

#ifndef PINTO_V1_FIRMWARE_VNET_STATES_H
#define PINTO_V1_FIRMWARE_VNET_STATES_H

class State {
public:
    static constexpr uint8_t FORCE_RESET = BYTES_2('0', '0');
    static constexpr uint8_t MAIN_LOOP = BYTES_2('0', '1');
    static constexpr uint8_t DFU_LOOP = BYTES_2('F', 'E');

protected:
    bool m_has_arg = false;
    State *m_prev;
    State *m_next;
    StateID_e m_id;
    HandlerFunc_Arg_t m_func;

public:
    State(HandlerFunc_t func, StateID_e id, State *prev = nullptr, State *next = nullptr) :
            State((HandlerFunc_Arg_t) func, id, prev, next) {
        m_has_arg = false;
    }

    State(HandlerFunc_Arg_t func, StateID_e id, State *prev = nullptr, State *next = nullptr) {
        m_id = id;
        m_func = func;
        m_prev = prev;
        m_next = next;
        m_has_arg = true;
    }

    StateID_e id() const {
        return m_id;
    }

    void run(void *arg = nullptr) {
        if (m_func != nullptr) {
            if (m_has_arg)
                m_func(arg);
            else
                m_func(nullptr);
        }
    }

    State *next() {
        return m_next;
    }

    State *prev() {
        return m_prev;
    }

    bool has_next() {
        return m_next != nullptr;
    }

    bool has_prev() {
        return m_prev != nullptr;
    }
};

class OSState : public State {
public:
    explicit OSState(HandlerFunc_t func,
                     StateID_e id = SYS_DISABLED,
                     State *prev = nullptr,
                     State *next = nullptr) : State(func, id, prev, next) {}

    explicit OSState(HandlerFunc_Arg_t func,
                     StateID_e id = SYS_DISABLED,
                     State *prev = nullptr,
                     State *next = nullptr) : State(func, id, prev, next) {}
};

#endif //PINTO_V1_FIRMWARE_VNET_STATES_H


#ifndef PINTO_V1_FIRMWARE_VNET_SENSORS_H
#define PINTO_V1_FIRMWARE_VNET_SENSORS_H

/* Includes */
/* Sensors Includes */
#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"
#include "Adafruit_CCS811.h"
#include "SparkFun_u-blox_GNSS_v3.h"
#include "TinyGPS++.h"
#include "Adafruit_BNO08x.h"
#include "Adafruit_BNO055.h"
#include "ICM_20948.h"
#include "PMS.h"
#include "util/OneWire_direct_gpio.h"
#include "OneWire.h"
#include "DallasTemperature.h"

/* Definitions */
#define MSLP_HPA (1013.25)
#define M_G (9.81)

/* Typedefs */
typedef struct {
    double r;
    double i;
    double j;
    double k;
} Quaternion_t;

typedef struct {
    double x;
    double y;
    double z;
} Vec3_t;

typedef struct {
    double roll;
    double pitch;
    double yaw;
} Euler_t;

typedef struct {
    double latitude;
    double longitude;
    double altitude;
} Sensors_Data_GPS_Position_t;

typedef struct {
    float temperature;
    float humidity;
    float pressure;
    float altitude;
} Sensors_Data_PHT_t;

typedef struct {
    uint16_t co2;
    uint16_t tvoc;
} Sensors_Data_CCS811_t;

typedef struct {
    float pm10;
    float pm25;
    float pm100;
} Sensors_Data_PMS_t;

typedef struct {
    Vec3_t accel;
    Vec3_t gyro;
    Vec3_t mag;
    Euler_t rotation;
} Sensors_Data_IMU_t;

typedef enum {
    GPS_ATGM336H = 0,
    GPS_SparkFun
} GPS_Type_e;

/* Sensors Tools Classes */
namespace impl {
    /* Private Class */
    class Sensors_Global {
    protected:
        bool m_valid = false;
        Peripherals_t *list = nullptr;
        TwoWire *wire = &Wire;

        explicit Sensors_Global(Peripherals_t *sensors_list) : list{sensors_list} {
#if !defined(CORE_TEENSY)
            if (TWCR & (_BV(TWEN) == 0)) {
                wire->begin();
                wire->setClock(400000UL);
            }
#else
            wire->begin();
            wire->setClock(400000UL);
#endif
        }

    public:
        Peripherals_t &devices() {
            return *list;
        }

        bool valid() const {
            return m_valid;
        }
    };

    class Sensors_GPS : protected impl::Sensors_Global {
    protected:
        Sensors_Data_GPS_Position_t *pos = nullptr;
        uint32_t last_millis = millis();

    public:
        explicit Sensors_GPS(Peripherals_t *sensors_list) : Sensors_Global(sensors_list) {
            pos = new Sensors_Data_GPS_Position_t();
        }

        virtual Sensors_Data_GPS_Position_t &read_position() = 0;
    };

    class Sensors_IMU : protected impl::Sensors_Global {
    protected:
        Sensors_Data_IMU_t data = {};
    public:
        explicit Sensors_IMU(Peripherals_t *sensors_list) : Sensors_Global(sensors_list) {}

        virtual Euler_t read_rotation() = 0;

        virtual Vec3_t read_acceleration() = 0;

        virtual Vec3_t read_gyroscope() = 0;

        virtual Vec3_t read_magnetic_field() = 0;

        Sensors_Data_IMU_t &read_data() {
            return data;
        }

        static Euler_t quaternion_to_euler(Quaternion_t &quaternion, bool degrees = true) {
            double sqr = m_square(quaternion.r);
            double sqi = m_square(quaternion.i);
            double sqj = m_square(quaternion.j);
            double sqk = m_square(quaternion.k);

            Euler_t euler = {
                    atan2(2.0 * (quaternion.i * quaternion.j + quaternion.k * quaternion.r), (sqi - sqj - sqk + sqr)),
                    asin(-2.0 * (quaternion.i * quaternion.k - quaternion.j * quaternion.r) / (sqi + sqj + sqk + sqr)),
                    atan2(2.0 * (quaternion.j * quaternion.k + quaternion.i * quaternion.r), (-sqi - sqj + sqk + sqr)),
            };

            if (degrees) {
                euler.yaw *= RAD_TO_DEG;
                euler.pitch *= RAD_TO_DEG;
                euler.roll *= RAD_TO_DEG;
            }

            return euler;
        }
    };
}

class Sensors_Environmental : protected impl::Sensors_Global {
private:
    static constexpr uint8_t pin_uv = A0;
    static constexpr uint8_t pin_33 = A1;
    Adafruit_BME280 *bme280;
    Adafruit_CCS811 *ccs811;

public:
    explicit Sensors_Environmental(Peripherals_t *sensors_list,
                                   uint8_t bme280_addr = ADDR_BME280,
                                   uint8_t ccs811_addr = ADDR_CCS811) : Sensors_Global(sensors_list) {
        bme280 = new Adafruit_BME280();
        ccs811 = new Adafruit_CCS811();
        list->env_bme280 = bme280->begin(bme280_addr, wire);
        list->env_ccs811 = ccs811->begin(ccs811_addr, wire);
        if (!list->env_bme280) {
            delete bme280;
            bme280 = nullptr;
        }
        if (!list->env_ccs811) {
            delete ccs811;
            ccs811 = nullptr;
        }
        m_valid = list->env_bme280 && list->env_ccs811;
    }

    Sensors_Data_PHT_t read_PHT() {
        if (list->env_bme280) {
            return {
                    bme280->readTemperature(),
                    bme280->readHumidity(),
                    bme280->readPressure() / 100.f,
                    bme280->readAltitude(MSLP_HPA)
            };
        }
        return {};
    }

    Sensors_Data_CCS811_t read_CCS811() {
        if (list->env_ccs811 && ccs811->available() && !ccs811->readData()) {
            return {
                    ccs811->geteCO2(),
                    ccs811->getTVOC()
            };
        }
        return {};
    }

    Sensors_Data_PMS_t read_PMS7003() {
        if (list->env_pms7003) {
//            todo: Implement PMS7003 readings.
        }
        return {};
    }

    static double read_GY8511_UV() {
        int16_t lvl_uv = averageAnalogRead(pin_uv);
        int16_t lvl_33 = averageAnalogRead(pin_33);
        return map_val(3.3 * lvl_uv / lvl_33, 0.99, 2.8, 0.0, 15.0);
    }
};

class Sensors_GPS_SparkFun : public impl::Sensors_GPS {
private:
    SFE_UBLOX_GNSS *gps_SparkFun = nullptr;

public:
    explicit Sensors_GPS_SparkFun(Peripherals_t *sensors_list) : Sensors_GPS(sensors_list) {
        gps_SparkFun = new SFE_UBLOX_GNSS();
        m_valid = list->gps_SparkFun = gps_SparkFun->begin();
        if (list->gps_SparkFun) {
            gps_SparkFun->setI2COutput(COM_TYPE_UBX);
            gps_SparkFun->saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
        }
    }

    Sensors_Data_GPS_Position_t &read_position() override {
        if (list->gps_SparkFun && (millis() - last_millis > MILLIS_500)) {
            *pos = {
                    ((double) gps_SparkFun->getLatitude()) * 1E-7,
                    ((double) gps_SparkFun->getLongitude()) * 1E-7,
                    ((double) gps_SparkFun->getAltitude()) * 1E-3
            };
            last_millis = millis();
        }
        return *pos;
    }
};

class Sensors_GPS_Serial : public impl::Sensors_GPS {
private:
    TinyGPSPlus gps;
    HardwareSerial *SerialGPS = nullptr;
public:
    explicit Sensors_GPS_Serial(Peripherals_t *sensors_list,
                                HardwareSerial *SerialGPS) : Sensors_GPS(sensors_list) {
        this->SerialGPS = SerialGPS;
        this->SerialGPS->begin(9600);
        m_valid = list->gps_ATGM336H = true;
    }

    Sensors_Data_GPS_Position_t &read_position() override {
        if (list->gps_ATGM336H && (millis() - last_millis > MILLIS_250)) {
            while (SerialGPS->available()) {
                gps.encode((char) SerialGPS->read());
            }
            last_millis = millis();
            if (gps.location.isValid()) {
                *pos = {
                        gps.location.lat(),
                        gps.location.lng(),
                        gps.altitude.meters()
                };
            }
        }
        return *pos;
    }
};

class Sensors_IMU_BNO085 : public impl::Sensors_IMU {
private:
    Adafruit_BNO08x *bno085 = nullptr;
    sh2_SensorValue_t *sensor_value = nullptr;
    static constexpr uint32_t sensor_interval_us = 100000;
    static constexpr uint8_t sensor_count = 4;
    Quaternion_t quaternion = {};

public:
    explicit Sensors_IMU_BNO085(Peripherals_t *sensors_list,
                                uint8_t bno085_addr = ADDR_BNO085) : Sensors_IMU(sensors_list) {
        bno085 = new Adafruit_BNO08x();
        sensor_value = new sh2_SensorValue_t();

        m_valid = list->imu_bno085 = bno085->begin_I2C(bno085_addr, wire);
        if (list->imu_bno085) {
            /* Set interval to 10 ms (100 Hz) */
            set_reports();
        }
    }

    Euler_t read_rotation() override {
        if (!list->imu_bno085)
            return {};
        DO_LOOP(2 * sensor_count, read_IMU_BNO085());
        return data.rotation;
    }

    Vec3_t read_acceleration() override {
        if (!list->imu_bno085)
            return {};
        DO_LOOP(2 * sensor_count, read_IMU_BNO085());
        return data.accel;
    }

    Vec3_t read_gyroscope() override {
        if (!list->imu_bno085)
            return {};
        DO_LOOP(2 * sensor_count, read_IMU_BNO085());
        return data.gyro;
    }

    Vec3_t read_magnetic_field() override {
        if (!list->imu_bno085)
            return {};
        DO_LOOP(sensor_count, read_IMU_BNO085());
        return data.mag;
    }

    void read_IMU_BNO085() {
        if (bno085->wasReset())
            set_reports();
        if (list->imu_bno085 && bno085->getSensorEvent(sensor_value)) {
            switch (sensor_value->sensorId) {
                case SH2_ACCELEROMETER:
                    data.accel = {
                            sensor_value->un.accelerometer.x,
                            sensor_value->un.accelerometer.y,
                            sensor_value->un.accelerometer.z
                    };
                    break;
                case SH2_ROTATION_VECTOR: {
                    quaternion = {
                            sensor_value->un.rotationVector.real,
                            sensor_value->un.rotationVector.i,
                            sensor_value->un.rotationVector.j,
                            sensor_value->un.rotationVector.k,
                    };
                    data.rotation = quaternion_to_euler(quaternion);
                    break;
                }
                case SH2_GYROSCOPE_CALIBRATED:
                    data.gyro = {
                            sensor_value->un.gyroscope.x,
                            sensor_value->un.gyroscope.y,
                            sensor_value->un.gyroscope.z
                    };
                    break;
                case SH2_MAGNETIC_FIELD_CALIBRATED:
                    data.mag = {
                            sensor_value->un.magneticField.x,
                            sensor_value->un.magneticField.y,
                            sensor_value->un.magneticField.z
                    };
                    break;
            }
        }
    }

private:
    void set_reports() {
        bno085->enableReport(SH2_ROTATION_VECTOR, sensor_interval_us);
        bno085->enableReport(SH2_ACCELEROMETER, sensor_interval_us);
        bno085->enableReport(SH2_GYROSCOPE_CALIBRATED, sensor_interval_us);
        bno085->enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, sensor_interval_us);
    }
};

class Sensors_IMU_BNO055 : public impl::Sensors_IMU {
private:
    Adafruit_BNO055 *bno055 = nullptr;
    sensors_event_t orientation_data = {};
    sensors_event_t accelerometer_data = {};
    sensors_event_t gyroscope_data = {};
    sensors_event_t magnetic_data = {};
public:
    explicit Sensors_IMU_BNO055(Peripherals_t *sensors_list,
                                uint8_t bno055_addr = ADDR_BNO055) : Sensors_IMU(sensors_list) {
        bno055 = new Adafruit_BNO055(55, bno055_addr, wire);
        m_valid = list->imu_bno055 = bno055->begin();
    }

    Euler_t read_rotation() override {
        if (list->imu_bno055) {
            bno055->getEvent(&orientation_data, Adafruit_BNO055::VECTOR_EULER);
            data.rotation = {
                    orientation_data.orientation.x,
                    orientation_data.orientation.y,
                    orientation_data.orientation.z
            };
        }
        return data.rotation;
    }

    Vec3_t read_acceleration() override {
        bno055->getEvent(&accelerometer_data, Adafruit_BNO055::VECTOR_ACCELEROMETER);
        if (list->imu_bno055) {
            data.accel = {
                    accelerometer_data.acceleration.x,
                    accelerometer_data.acceleration.y,
                    accelerometer_data.acceleration.z
            };
        }
        return data.accel;
    }

    Vec3_t read_gyroscope() override {
        bno055->getEvent(&gyroscope_data, Adafruit_BNO055::VECTOR_GYROSCOPE);
        if (list->imu_bno055) {
            data.gyro = {
                    gyroscope_data.gyro.x,
                    gyroscope_data.gyro.y,
                    gyroscope_data.gyro.z
            };
        }
        return data.gyro;
    }

    Vec3_t read_magnetic_field() override {
        bno055->getEvent(&magnetic_data, Adafruit_BNO055::VECTOR_MAGNETOMETER);
        if (list->imu_bno055) {
            data.mag = {
                    magnetic_data.magnetic.x,
                    magnetic_data.magnetic.y,
                    magnetic_data.magnetic.z
            };
        }
        return data.mag;
    }
};

class Sensors_IMU_ICM20948 : public impl::Sensors_IMU {
private:
    ICM_20948_I2C *icm20948 = nullptr;

public:
    explicit Sensors_IMU_ICM20948(Peripherals_t *sensors_list) : Sensors_IMU(sensors_list) {
        icm20948 = new ICM_20948_I2C();
        icm20948->begin(*wire, true);
        m_valid = list->imu_icm20948 = true;
    }

    Euler_t read_rotation() override {
        return {};
    }

    Vec3_t read_acceleration() override {
        if (list->imu_icm20948) {
            read_IMU_ICM20948();
            data.accel = {
                    (double) icm20948->agmt.acc.axes.x,
                    (double) icm20948->agmt.acc.axes.y,
                    (double) icm20948->agmt.acc.axes.z
            };
        }
        return data.accel;
    }

    Vec3_t read_gyroscope() override {
        if (list->imu_icm20948) {
            read_IMU_ICM20948();
            data.gyro = {
                    (double) icm20948->agmt.gyr.axes.x,
                    (double) icm20948->agmt.gyr.axes.y,
                    (double) icm20948->agmt.gyr.axes.z
            };
        }
        return data.gyro;
    }

    Vec3_t read_magnetic_field() override {
        if (list->imu_icm20948) {
            read_IMU_ICM20948();
            data.mag = {
                    (double) icm20948->agmt.mag.axes.x,
                    (double) icm20948->agmt.mag.axes.y,
                    (double) icm20948->agmt.mag.axes.z
            };
        }
        return {};
    }

    void read_IMU_ICM20948() {
        if (icm20948->dataReady()) {
            icm20948->getAGMT();
        }
    }
};

namespace impl {
    void setupPD4_out() {
        DDRD |= (1 << PD4);
    }

    void setupPD4_in() {
        DDRD &= ~(1 << PD4);
    }

    uint8_t readPD4() {
        return (PIND & (1 << PD4)) >> PD4;
    }

#undef DIRECT_MODE_INPUT
#define DIRECT_MODE_INPUT() setupPD4_in()
#undef DIRECT_MODE_OUTPUT
#define DIRECT_MODE_OUTPUT() setupPD4_out()
#undef DIRECT_WRITE_LOW
#define DIRECT_WRITE_LOW() (PORTD &= ~(1 << PD4))
#undef DIRECT_READ
#define DIRECT_READ() readPD4()

    class OneWirePD4 : public OneWire {
    public:
        OneWirePD4() : OneWire(0) {}

        void write_bit(uint8_t v) {
            PAUSE_INTERRUPT(
                    if (v & 1) {
                        DIRECT_MODE_OUTPUT();
                        DIRECT_WRITE_LOW();
                        delayMicroseconds(10);
                        DIRECT_MODE_INPUT();
                        delayMicroseconds(55);
                    } else {
                        DIRECT_MODE_OUTPUT();
                        DIRECT_WRITE_LOW();
                        delayMicroseconds(65);
                        DIRECT_MODE_INPUT();
                        delayMicroseconds(5);
                    }
            )
        }

        uint8_t read_bit() {
            uint8_t r;
            PAUSE_INTERRUPT(
                    DIRECT_MODE_OUTPUT();
                    DIRECT_WRITE_LOW();
                    delayMicroseconds(3);
                    DIRECT_MODE_INPUT(); // let pin float, pull up will raise
                    delayMicroseconds(10);
                    r = readPD4();
            )
            delayMicroseconds(53);
            return r;
        }
    };
}


class Sensors_Environmental_DS18B20 : protected impl::Sensors_Global {
private:
    impl::OneWirePD4 one_wire{};
    DallasTemperature sensor{&one_wire};
public:
    explicit Sensors_Environmental_DS18B20(Peripherals_t *sensors_list) :
            Sensors_Global(sensors_list) {}

    float read_temperature() {
        sensor.requestTemperatures();
        return sensor.getTempCByIndex(0);
    }
};

/*
 * Serial LoRa class (Specialized for M0, M1 pin at PH2 and PH3 on ATmega2560 only)
 */
class LoRa_E32_915T30D {
private:
    static constexpr uint32_t baud_cfg = 9600U;

    static constexpr uint8_t LORA_8N1 = 0b00;
    static constexpr uint8_t LORA_8O1 = 0b01;
    static constexpr uint8_t LORA_8E1 = 0b10;

    static constexpr uint8_t LORA_BAUD_1200 = 0b000;
    static constexpr uint8_t LORA_BAUD_2400 = 0b001;
    static constexpr uint8_t LORA_BAUD_4800 = 0b010;
    static constexpr uint8_t LORA_BAUD_9600 = 0b011;
    static constexpr uint8_t LORA_BAUD_19200 = 0b100;
    static constexpr uint8_t LORA_BAUD_38400 = 0b101;
    static constexpr uint8_t LORA_BAUD_57600 = 0b110;
    static constexpr uint8_t LORA_BAUD_115200 = 0b111;

    static constexpr uint8_t LORA_RATE_300 = 0b000;
    static constexpr uint8_t LORA_RATE_1200 = 0b001;
    static constexpr uint8_t LORA_RATE_2400 = 0b010;
    static constexpr uint8_t LORA_RATE_4800 = 0b011;
    static constexpr uint8_t LORA_RATE_9600 = 0b100;
    static constexpr uint8_t LORA_RATE_19200 = 0b101;

    HardwareSerial *SerialLoRa = nullptr;
    uint8_t pin_M0 = PH2;
    uint8_t pin_M1 = PH3;
    uint8_t config[6] = {};
    uint32_t m_baud;

public:
    explicit LoRa_E32_915T30D(HardwareSerial *SerialLoRa, uint32_t baud = 115200U) {
        this->SerialLoRa = SerialLoRa;
        this->m_baud = baud;
        DDRH |= (1 << pin_M0) | (1 << pin_M1);      // Set M0, M1 to OUTPUT mode
    }

    void begin_normal() const {
        PORTH &= ~((1 << pin_M0) | (1 << pin_M1));  // Set M0 = 0 ; M1 = 0 (Normal Mode)
        SerialLoRa->begin(m_baud);
    }

    void begin_normal(uint32_t baud) {
        this->m_baud = baud;
        begin_normal();
    }

    void begin_cfg() const {
        PORTH |= (1 << pin_M0) | (1 << pin_M1);     // Set M0 = 1 ; M1 = 1 (Sleep Mode)

        SerialLoRa->end();
        SerialLoRa->begin(baud_cfg, SERIAL_8N1);
    }

    void cfg_from_pc() const {
        Serial.end();
        Serial.begin(baud_cfg);

        while (!Serial.available())
            delay(10);

        delay(100);

        while (Serial.available())
            SerialLoRa->write(Serial.read());
    }

    void cmd_set_params(uint16_t addr, uint32_t baud_rate,
                        uint8_t parity, uint32_t data_rate,
                        uint8_t channel, bool save_params) {
        memset(config, 0, sizeof(config));
        uint8_t addr_l = addr & 0xff;
        uint8_t addr_h = (addr >> 8) & 0xff;
        config[0] = save_params ? 0xc0 : 0xc2;
        config[1] = addr_h;
        config[2] = addr_l;
        config[3] |= (parity << 6);
        config[3] |= (baud_rate << 3);
        config[3] |= (data_rate);
        config[4] |= (channel & 0b00011111);
        config[5] = 0b01000100; // default WOR 250 ms, 30 dBm, FEC

        SerialLoRa->write(config, sizeof(config));
    }

    void cmd_get_params() {
        SerialLoRa->write(0xc1);
        SerialLoRa->write(0xc1);
        SerialLoRa->write(0xc1);
    }

    void cmd_get_versions() {
        SerialLoRa->write(0xc3);
        SerialLoRa->write(0xc3);
        SerialLoRa->write(0xc3);
    }

    void cmd_reset_module() {
        SerialLoRa->write(0xc4);
        SerialLoRa->write(0xc4);
        SerialLoRa->write(0xc4);
    }
};

using impl::Sensors_GPS;
using impl::Sensors_IMU;

#endif //PINTO_V1_FIRMWARE_VNET_SENSORS_H

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

extern void lora_cfg_handler();

extern void boot_handler();

extern void setup_handler();

extern void main_loop_handler();

extern void sig_trap();

extern void reset_all();

/* Device Configurations and Parameters */
EEPROM_Config_t device_params = {
        .loop_interval = MILLIS_1S
};

/* Global Variables */
Peripherals_t sensors_list = {};
LoRa_E32_915T30D *lora;
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
State lora_cfg_state = OSState(lora_cfg_handler, StateID_e::SYS_DFU);
State sink_state = OSState(sig_trap);

State *setup_states_list[] = {&boot_state, &setup_state};

void setup() {
    /* Run Setups: boot init and setup variables */
    for (State *&s: setup_states_list) {
        s->run();
    }
    state = &main_state;
    Serial.write(uart0_rx_buf, sizeof uart0_rx_buf);
}

void loop() {
    /* Begin Run current state */
    state->run();
    /* End Run current state */

    /* Begin Poll for Serial */
    if (state != &lora_cfg_state)
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
                case State::FORCE_RESET:
                    reset_all();
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

        packet = build_string(data.device_name,
                              data.counter,
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
                              data.ext_temperature,
                              data.battery_v);

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
    /* End Serial USB */

    /* Begin WDT */
    setup_wdt();
    /* End WDT */

    data.device_name = String("CGSAT") + String(DEVICE_NUMBER);
}

void lora_cfg_handler() {
    lora->begin_cfg();
    lora->cfg_from_pc();
//    todo: Automatically reset after done setting parameters. How?
}

/* End Handler Functions Definitions */
void init_peripherals() {
    /* Begin Pins */
    DDRD &= ~(1 << PIN_EXT_TEMP);                      // PD4 for Temperature Sensor

    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_WDT_Activate, OUTPUT);
    pinMode(PIN_WDT_PWM, OUTPUT);
    pinMode(PIN_ADC_BATT, INPUT);
    /* End Pins */

    /* Begin LoRa */
    lora = new LoRa_E32_915T30D(&SerialLoRa);
    /* End LoRa */

    /* Begin Sensors */
    env_sensors = new Sensors_Environmental(&sensors_list);
    gps0 = new Sensors_GPS_SparkFun(&sensors_list);
    gps1 = new Sensors_GPS_Serial(&sensors_list, &SerialGPS1);
    ext_sense = new Sensors_Environmental_DS18B20(&sensors_list);
    /* End Sensors */

    /* Begin Interface */
    lora->begin_normal();
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

void sig_trap() {
    PAUSE_INTERRUPT(while (true);)
}

void reset_all() {
    wdt_disable();
    wdt_enable(WDTO_15MS);
    sig_trap();
}
