#ifndef PINTO_V1_FIRMWARE_VNET_SENSORS_H
#define PINTO_V1_FIRMWARE_VNET_SENSORS_H

/* Includes */
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include "vnet_definitions.h"
#include "vnet_tools.h"
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
#include "modified/OneWire/util/OneWire_direct_gpio.h"
#include "modified/OneWire/OneWire.h"
#include "modified/DallasTemperature/DallasTemperature.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "Fonts/TomThumb.h"
/* USB Interface */
//#include "Adafruit_TinyUSB.h"
//#include "arduino/msc/Adafruit_USBD_MSC.h"

/* Definitions */
#define MSLP_HPA (1013.25)
#define M_G (9.81)

static const uint8_t log_sac[] PROGMEM = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xcf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x07, 0x87, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x0f, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f,
        0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x01, 0xe0,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 0xf0, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0xe0, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0xe0, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xe0,
        0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x00, 0x0e,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x00, 0x0e, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x03, 0x80, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x03, 0x80, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x80,
        0x30, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x78, 0x03,
        0x00, 0x0f, 0xf3, 0xfe, 0x7f, 0xcf, 0xf9, 0xff, 0x01, 0xfe, 0x3f, 0xc0, 0x07, 0x00, 0xfc, 0x03, 0x00, 0x1f,
        0xf3, 0xfe, 0x7f, 0xcf, 0xf9, 0xff, 0x01, 0xfe, 0x7f, 0xc0, 0x07, 0x01, 0xfe, 0x03, 0x80, 0x18, 0x00, 0x02,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x03, 0xff, 0x03, 0x80, 0x18, 0x00, 0x06, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0xcf, 0x83, 0x00, 0x1f, 0xf3, 0xfe, 0x7f, 0xc8, 0x01, 0xff,
        0x03, 0xff, 0x60, 0x00, 0x07, 0x0f, 0x87, 0xc3, 0x00, 0x00, 0x13, 0x00, 0x60, 0xc8, 0x00, 0x00, 0x03, 0x03,
        0x60, 0x00, 0x07, 0x1e, 0x03, 0xc3, 0x00, 0x00, 0x12, 0x00, 0x40, 0x4c, 0x00, 0x00, 0x03, 0x03, 0x60, 0x00,
        0x03, 0x9e, 0x01, 0xe3, 0x00, 0x1f, 0xf3, 0x00, 0x40, 0x4f, 0xf9, 0xff, 0x03, 0x03, 0x7f, 0xc0, 0x03, 0xbc,
        0x00, 0xf1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xb8, 0x00, 0xf9,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x90, 0x00, 0x78, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x90, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x01, 0xc0, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x01, 0xe0, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0xe0,
        0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0xe0, 0x00, 0x13,
        0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x70, 0x00, 0x3b, 0xc0, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x70, 0x00, 0x39, 0xc0, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x60, 0xfe, 0x71, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x07, 0xfc, 0x71, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x1c, 0x1f, 0xfc, 0xe0, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x1c, 0x7f, 0x01, 0xe4, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x1c, 0xf8, 0x01, 0xce, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0xe1,
        0x03, 0x8f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x83, 0x87, 0x8f,
        0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x03, 0xcf, 0x07, 0xf0, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x01, 0xfe, 0x01, 0xf0, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0xfc, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0xfc, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x78, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x20, 0x00, 0x30, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

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
                    ((double) gps_SparkFun->getLatitude(250)) * 1E-7,
                    ((double) gps_SparkFun->getLongitude(250)) * 1E-7,
                    ((double) gps_SparkFun->getAltitude(250)) * 1E-3
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

class Sensors_Environmental_DS18B20 : protected impl::Sensors_Global {
private:
    OneWire one_wire_int{0};
    DallasTemperature sensor;
    DeviceAddress addr = {};
public:
    explicit Sensors_Environmental_DS18B20(Peripherals_t *sensors_list) :
            Sensors_Global(sensors_list) {
        sensor.setOneWire(&one_wire_int);
        sensor.setResolution(9);
        sensor.begin();
        sensor.getAddress(addr, 0);
    }

    float read_temperature() {
        sensor.requestTemperaturesByAddress(addr);
        return sensor.getTempCByIndex(0);
    }
};

class Storage_SD {
private:
    SdFat _sd;
    String _save_filename;

public:
    File file;

public:
    explicit Storage_SD(String &filename) {
        _sd.begin(53, SPI_FULL_SPEED);
        next_filename(filename);
        open();
    }

    explicit Storage_SD(const char *filename) {
        _sd.begin(53, SPI_FULL_SPEED);
        String _filename_str = String(filename);
        next_filename(_filename_str);
        open();
    }

    void next_filename(String &filename) {
        uint16_t file_no = 0;
        do { _save_filename = filename + file_no++ + ".csv"; } while (_sd.exists(_save_filename));
    }

    void list_files() {
        File root = _sd.open("/");
        char tmp[256];
        while (true) {
            File entry = root.openNextFile();
            if (!entry) break;
            memset(tmp, 0, sizeof(tmp));
            entry.getName(tmp, sizeof(tmp));
            Serial.print(tmp);
            Serial.print("\t");
            Serial.println((uint32_t) entry.size());
            entry.close();
        }
    }

    void read_content(String &filename) {
        File _file = _sd.open(filename, FILE_READ);
        if (!_file) return;
        while (_file.available()) Serial.write(_file.read());
        _file.close();
    }

    void delete_file(String &filename) {
        _sd.remove(filename);
    }

    void delete_all() {
        File root = _sd.open("/");
        char tmp[256];
        while (true) {
            File entry = root.openNextFile();
            if (!entry) break;
            memset(tmp, 0, sizeof(tmp));
            entry.getName(tmp, sizeof(tmp));
            _sd.remove(tmp);
        }
    }

    void open() {
        file = _sd.open(_save_filename, FILE_WRITE);
        file.seek(EOF);
    }

    void flush() {
        file.flush();
    }

    void close() {
        file.close();
    }
};

class USB_Device {
};

class Screen_OLED {
private:
    TwoWire *wire = &Wire;
    uint8_t width;
    uint8_t height;
    bool status = false;

public:
    Adafruit_SSD1306 *display = nullptr;

public:
    explicit Screen_OLED(uint8_t w = 128, uint8_t h = 64) {
        width = w;
        height = h;
        display = new Adafruit_SSD1306(width, height, wire, -1);
        status = display->begin(SSD1306_SWITCHCAPVCC, 0x3D);
        if (!status) {
            delete display;
        } else {
            draw_space_logo();
            display->setTextColor(1);
            display->setTextSize(1);
            display->setFont(&TomThumb);
        }
    }

    void draw_space_logo() const {
        if (!status) return;
        display->clearDisplay();
        display->drawBitmap(0, 0, log_sac, 128, 64, 1);
        display->display();
    }

    void clear() const {
        if (!status) return;
        display->clearDisplay();
        display->display();
    }

    bool valid() const {
        return status;
    }
};

namespace impl {
    class LoRa_Global_Serial {
    public:
        static constexpr uint8_t LORA_TX_MAX = 0b00;
        static constexpr uint8_t LORA_TX_HIGH = 0b01;
        static constexpr uint8_t LORA_TX_LOW = 0b10;
        static constexpr uint8_t LORA_TX_MIN = 0b11;

        static constexpr uint8_t LORA_BAUD_1200 = 0b000;
        static constexpr uint8_t LORA_BAUD_2400 = 0b001;
        static constexpr uint8_t LORA_BAUD_4800 = 0b010;
        static constexpr uint8_t LORA_BAUD_9600 = 0b011;
        static constexpr uint8_t LORA_BAUD_19200 = 0b100;
        static constexpr uint8_t LORA_BAUD_38400 = 0b101;
        static constexpr uint8_t LORA_BAUD_57600 = 0b110;
        static constexpr uint8_t LORA_BAUD_115200 = 0b111;

        static constexpr uint8_t LORA_8N1 = 0b00;
        static constexpr uint8_t LORA_8O1 = 0b01;
        static constexpr uint8_t LORA_8E1 = 0b10;

        static constexpr bool LORA_SAVE_PARAMS = true;
        static constexpr bool LORA_ENABLE_FEC = true;

    protected:
        static constexpr uint32_t baud_cfg = 9600U;
        HardwareSerial *m_SerialLoRa;
        uint8_t MASK_M0; // PH2 for SG Sat
        uint8_t MASK_M1; // PH3 for SG Sat
        volatile uint8_t DDR_M0;
        volatile uint8_t DDR_M1;
        volatile uint8_t PORT_M0;
        volatile uint8_t PORT_M1;
        uint8_t config[16] = {};
        uint32_t m_baud;

    public:
        /*
         * Constructor with digital pin
         */
        LoRa_Global_Serial(HardwareSerial *SerialLoRa, uint32_t baud,
                           uint8_t digitalPin_M0, uint8_t digitalPin_M1) {
            this->m_SerialLoRa = SerialLoRa;
            this->m_baud = baud;

            MASK_M0 = digitalPinToBitMask(digitalPin_M0);
            PORT_M0 = *portOutputRegister(MASK_M0);
            DDR_M0 = *portModeRegister(MASK_M0);
            MASK_M1 = digitalPinToBitMask(digitalPin_M1);
            PORT_M1 = *portOutputRegister(MASK_M1);
            DDR_M1 = *portModeRegister(MASK_M1);

            DDR_M0 |= (1 << MASK_M0);      // Set M0 to OUTPUT mode
            DDR_M1 |= (1 << MASK_M1);      // Set M1 to OUTPUT mode
        }

        /*
         * Constructor with direct pin mapping
         */
        LoRa_Global_Serial(HardwareSerial *SerialLoRa, uint32_t baud,
                           volatile uint8_t &ddr_M0, volatile uint8_t &ddr_M1,
                           volatile uint8_t &port_M0, volatile uint8_t &port_M1,
                           uint8_t mask_M0, uint8_t mask_M1) {
            this->m_SerialLoRa = SerialLoRa;
            this->m_baud = baud;

            MASK_M0 = mask_M0;
            PORT_M0 = port_M0;
            DDR_M0 = ddr_M0;
            MASK_M1 = mask_M1;
            PORT_M1 = port_M1;
            DDR_M1 = ddr_M1;

            DDR_M0 |= (1 << MASK_M0);      // Set M0 to OUTPUT mode
            DDR_M1 |= (1 << MASK_M1);      // Set M1 to OUTPUT mode
        }

        virtual void begin_normal(uint32_t baud) {
            this->m_baud = baud;
            PORT_M0 &= ~((1 << MASK_M0));  // Set M0 = 0
            PORT_M1 &= ~((1 << MASK_M1));  // Set M1 = 0 (Normal Mode)

            delay(100);

            m_SerialLoRa->begin(m_baud);
        }

        virtual void begin_cfg() {
            PORT_M0 |= (1 << MASK_M0);     // Set M0 = 1
            PORT_M1 |= (1 << MASK_M1);     // Set M1 = 1 (Sleep Mode)

            delay(100);

            m_SerialLoRa->end();
            m_SerialLoRa->begin(baud_cfg, SERIAL_8N1);
        }

        virtual void cmd_get_params() {
            write_triple(0xc1);

            while (!m_SerialLoRa->available());

            delay(100);

            uint8_t i = 0;
            while (m_SerialLoRa->available()) {
                config[i] = m_SerialLoRa->read();
                Serial.print(config[i], 2);
                Serial.print(" ");
                ++i;
            }
        }

        virtual void cmd_write_params() {
            m_SerialLoRa->write(config, 6);
        }

        virtual void print_params() {
            print_params(config);
        }

        virtual void print_params(uint8_t config[8]) = 0;

        virtual void cmd_get_versions() = 0;

        virtual void cmd_reset_module() = 0;

    protected:
        inline void write_triple(uint8_t b) {
            write_to_module(b, b, b);
        }

        template<typename T = uint8_t>
        inline void write_to_module(T last) {
            m_SerialLoRa->write(last);
        }

        template<typename T = uint8_t, typename ...Args>
        inline void write_to_module(T first, Args... args) {
            m_SerialLoRa->write(first);
            write_to_module(args...);
        }
    };
}

/*
 * Serial LoRa E32 class for Configuration and Normal Ops.
 */
class LoRa_E32 : public impl::LoRa_Global_Serial {
public:
    static constexpr uint8_t LORA_CHANNEL_0 = 0;
    static constexpr uint8_t LORA_CHANNEL_1 = 1;
    static constexpr uint8_t LORA_CHANNEL_2 = 2;
    static constexpr uint8_t LORA_CHANNEL_3 = 3;
    static constexpr uint8_t LORA_CHANNEL_4 = 4;
    static constexpr uint8_t LORA_CHANNEL_5 = 5;
    static constexpr uint8_t LORA_CHANNEL_6 = 6;
    static constexpr uint8_t LORA_CHANNEL_7 = 7;
    static constexpr uint8_t LORA_CHANNEL_8 = 8;
    static constexpr uint8_t LORA_CHANNEL_9 = 9;
    static constexpr uint8_t LORA_CHANNEL_10 = 10;
    static constexpr uint8_t LORA_CHANNEL_11 = 11;
    static constexpr uint8_t LORA_CHANNEL_12 = 12;
    static constexpr uint8_t LORA_CHANNEL_13 = 13;
    static constexpr uint8_t LORA_CHANNEL_14 = 14;
    static constexpr uint8_t LORA_CHANNEL_15 = 15;

    static constexpr uint8_t LORA_RATE_300 = 0b000;
    static constexpr uint8_t LORA_RATE_1200 = 0b001;
    static constexpr uint8_t LORA_RATE_2400 = 0b010;
    static constexpr uint8_t LORA_RATE_4800 = 0b011;
    static constexpr uint8_t LORA_RATE_9600 = 0b100;
    static constexpr uint8_t LORA_RATE_19200 = 0b101;

public:
    using impl::LoRa_Global_Serial::LoRa_Global_Serial;

    void cmd_set_params(uint16_t addr, uint8_t baud_rate,
                        uint8_t parity, uint8_t data_rate,
                        uint8_t channel, uint8_t tx_power,
                        bool enb_FEC, bool save_params) {
        memset(config, 0, 6);
        uint8_t addr_l = addr & 0xff;
        uint8_t addr_h = (addr >> 8) & 0xff;
        config[0] = save_params ? 0xc0 : 0xc2;
        config[1] = addr_h;
        config[2] = addr_l;
        config[3] |= (parity << 6);
        config[3] |= (baud_rate << 3);
        config[3] |= (data_rate);
        config[4] |= (channel & 0b00011111);
        config[5] = 0b01000000 | enb_FEC << 2 | tx_power; // default Push-pull, WOR 250 ms
    }

    void print_params() override {
        print_params(config);
    }

    void print_params(uint8_t config[8]) override {
        Serial.print("Write Mode ");
        Serial.println(config[0], 16);

        Serial.print("Address    ");
        Serial.println((config[1] << 8) + config[2]);

        Serial.print("Parity     ");
        uint8_t parity = (config[3] & 0b11000000) >> 6;
        switch (parity) {
            case LORA_8N1:
                Serial.println("8N1");
                break;
            case LORA_8O1:
                Serial.println("8O1");
                break;
            case LORA_8E1:
                Serial.println("8E1");
                break;
            default:
                Serial.println("N/A");
                break;
        }

        Serial.print("Baud       ");
        uint8_t baud_r = (config[3] & 0b00111000) >> 3;
        switch (baud_r) {
            case LORA_BAUD_1200:
                Serial.println("1200");
                break;
            case LORA_BAUD_2400:
                Serial.println("2400");
                break;
            case LORA_BAUD_4800:
                Serial.println("4800");
                break;
            case LORA_BAUD_9600:
                Serial.println("9600");
                break;
            case LORA_BAUD_19200:
                Serial.println("19200");
                break;
            case LORA_BAUD_38400:
                Serial.println("38400");
                break;
            case LORA_BAUD_57600:
                Serial.println("57600");
                break;
            case LORA_BAUD_115200:
                Serial.println("115200");
                break;
            default:
                Serial.println("N/A");
                break;
        }

        Serial.print("Data Rate  ");
        uint8_t data_r = (config[3] & 0b00000111);
        switch (data_r) {
            case LORA_RATE_1200:
                Serial.println("1200");
                break;
            case LORA_RATE_2400:
                Serial.println("2400");
                break;
            case LORA_RATE_4800:
                Serial.println("4800");
                break;
            case LORA_RATE_9600:
                Serial.println("9600");
                break;
            case LORA_RATE_19200:
                Serial.println("19200");
                break;
            case LORA_RATE_300:
                Serial.println("300");
                break;
            default:
                Serial.println("N/A");
                break;
        }

        Serial.print("Channel    ");
        Serial.println(config[4], 16);

        Serial.print("FEC        ");
        uint8_t fec = (config[5] & 0b00000100);
        if (fec)
            Serial.println("ENABLED");
        else
            Serial.println("DISABLED");

        Serial.print("Tx Power   ");
        uint8_t tx_pow = (config[5] & 0b00000011);
        switch (tx_pow) {
            case LORA_TX_MAX:
                Serial.println("Max (4)");
                break;
            case LORA_TX_HIGH:
                Serial.println("High (3)");
                break;
            case LORA_TX_LOW:
                Serial.println("Low (2)");
                break;
            default:
                Serial.println("Min (1)");
                break;
        }

        Serial.print("FIELD 5    ");
        Serial.println(config[5], 16);
    }

    void cmd_get_versions() override {
        write_triple(0xc3);
    }

    void cmd_reset_module() override {
        write_triple(0xc4);
    }
};

/*
 * Serial LoRa E34 class for Configuration and Normal Ops.
 */
class LoRa_E34 : public impl::LoRa_Global_Serial {
public:
    static constexpr uint8_t LORA_CHANNEL_0 = 0;
    static constexpr uint8_t LORA_CHANNEL_1 = 1;
    static constexpr uint8_t LORA_CHANNEL_2 = 2;
    static constexpr uint8_t LORA_CHANNEL_3 = 3;
    static constexpr uint8_t LORA_CHANNEL_4 = 4;
    static constexpr uint8_t LORA_CHANNEL_5 = 5;
    static constexpr uint8_t LORA_CHANNEL_6 = 6;
    static constexpr uint8_t LORA_CHANNEL_7 = 7;
    static constexpr uint8_t LORA_CHANNEL_8 = 8;
    static constexpr uint8_t LORA_CHANNEL_9 = 9;
    static constexpr uint8_t LORA_CHANNEL_10 = 10;
    static constexpr uint8_t LORA_CHANNEL_11 = 11;

    static constexpr uint8_t LORA_RATE_300 = 0b000;
    static constexpr uint8_t LORA_RATE_1200 = 0b001;
    static constexpr uint8_t LORA_RATE_2400 = 0b010;
    static constexpr uint8_t LORA_RATE_4800 = 0b011;
    static constexpr uint8_t LORA_RATE_9600 = 0b100;
    static constexpr uint8_t LORA_RATE_19200 = 0b101;

public:
    using impl::LoRa_Global_Serial::LoRa_Global_Serial;

    void cmd_set_params(uint16_t addr, uint8_t baud_rate,
                        uint8_t parity, uint8_t data_rate,
                        uint8_t channel, uint8_t tx_power,
                        bool enb_FEC, bool save_params) {
        memset(config, 0, 6);
        uint8_t addr_l = addr & 0xff;
        uint8_t addr_h = (addr >> 8) & 0xff;
        config[0] = save_params ? 0xc0 : 0xc2;
        config[1] = addr_h;
        config[2] = addr_l;
        config[3] |= (parity << 6);
        config[3] |= (baud_rate << 3);
        config[3] |= (data_rate);
        config[4] |= (channel & 0b00001111);
        config[5] = 0b01000000 | enb_FEC << 2 | tx_power; // default Push-pull, WOR 250 ms
    }

    void print_params() override {
        print_params(config);
    }

    void print_params(uint8_t config[8]) override {
        Serial.print("Write Mode ");
        Serial.println(config[0], 16);

        Serial.print("Address    ");
        Serial.println((config[1] << 8) + config[2]);

        Serial.print("Parity     ");
        uint8_t parity = (config[3] & 0b11000000) >> 6;
        switch (parity) {
            case LORA_8N1:
                Serial.println("8N1");
                break;
            case LORA_8O1:
                Serial.println("8O1");
                break;
            case LORA_8E1:
                Serial.println("8E1");
                break;
            default:
                Serial.println("N/A");
                break;
        }

        Serial.print("Baud       ");
        uint8_t baud_r = (config[3] & 0b00111000) >> 3;
        switch (baud_r) {
            case LORA_BAUD_1200:
                Serial.println("1200");
                break;
            case LORA_BAUD_2400:
                Serial.println("2400");
                break;
            case LORA_BAUD_4800:
                Serial.println("4800");
                break;
            case LORA_BAUD_9600:
                Serial.println("9600");
                break;
            case LORA_BAUD_19200:
                Serial.println("19200");
                break;
            case LORA_BAUD_38400:
                Serial.println("38400");
                break;
            case LORA_BAUD_57600:
                Serial.println("57600");
                break;
            case LORA_BAUD_115200:
                Serial.println("115200");
                break;
            default:
                Serial.println("N/A");
                break;
        }

        Serial.print("Data Rate  ");
        uint8_t data_r = (config[3] & 0b00000111);
        switch (data_r) {
            case LORA_RATE_1200:
                Serial.println("1200");
                break;
            case LORA_RATE_2400:
                Serial.println("2400");
                break;
            case LORA_RATE_4800:
                Serial.println("4800");
                break;
            case LORA_RATE_9600:
                Serial.println("9600");
                break;
            case LORA_RATE_19200:
                Serial.println("19200");
                break;
            case LORA_RATE_300:
                Serial.println("300");
                break;
            default:
                Serial.println("N/A");
                break;
        }

        Serial.print("Channel    ");
        Serial.println(config[4], 16);

        Serial.print("FEC        ");
        uint8_t fec = (config[5] & 0b00000100);
        if (fec)
            Serial.println("ENABLED");
        else
            Serial.println("DISABLED");

        Serial.print("Tx Power   ");
        uint8_t tx_pow = (config[5] & 0b00000011);
        switch (tx_pow) {
            case LORA_TX_MAX:
                Serial.println("Max (4)");
                break;
            case LORA_TX_HIGH:
                Serial.println("High (3)");
                break;
            case LORA_TX_LOW:
                Serial.println("Low (2)");
                break;
            default:
                Serial.println("Min (1)");
                break;
        }

        Serial.print("FIELD 5    ");
        Serial.println(config[5], 16);
    }

    void cmd_get_versions() override {
        write_triple(0xc3);
    }

    void cmd_reset_module() override {
        write_triple(0xc4);
    }
};

#ifdef DEV_E22
/*
 * Serial LoRa E22 class for Configuration and Normal Ops.
 */
class LoRa_E22 : public impl::LoRa_Global_Serial {
public:
    static constexpr uint8_t LORA_CHANNEL_0 = 0;

    static constexpr uint8_t LORA_RATE_300 = 0b000;
    static constexpr uint8_t LORA_RATE_1200 = 0b001;
    static constexpr uint8_t LORA_RATE_2400 = 0b010;
    static constexpr uint8_t LORA_RATE_4800 = 0b011;
    static constexpr uint8_t LORA_RATE_9600 = 0b100;
    static constexpr uint8_t LORA_RATE_19200 = 0b101;

public:
    using impl::LoRa_Global_Serial::LoRa_Global_Serial;

    void begin_cfg() override {
        PORT_M0 &= ~((1 << MASK_M0));  // Set M0 = 1
        PORT_M1 |= (1 << MASK_M1);     // Set M1 = 1 (Configuration Mode)

        delay(100);

        m_SerialLoRa->end();
        m_SerialLoRa->begin(baud_cfg, SERIAL_8N1);
    }

    void cmd_get_params() override {
        write_to_module(0xc0, 0x0, 8);

        while (!m_SerialLoRa->available());

        delay(100);

        uint8_t i = 0;
        while (m_SerialLoRa->available()) {
            config[i] = m_SerialLoRa->read();
            Serial.print(config[i], 2);
            Serial.print(" ");
            ++i;
        }
    }

    void cmd_set_params(uint16_t addr, uint8_t baud_rate,
                        uint8_t parity, uint8_t data_rate,
                        uint8_t channel, uint8_t tx_power,
                        bool enb_FEC, bool save_params) {
        memset(config, 0, 6);
        uint8_t addr_l = addr & 0xff;
        uint8_t addr_h = (addr >> 8) & 0xff;
        config[0] = save_params ? 0xc0 : 0xc2;
        config[1] = addr_h;
        config[2] = addr_l;
        config[3] |= (parity << 6);
        config[3] |= (baud_rate << 3);
        config[3] |= (data_rate);
        config[4] |= (channel & 0b00001111);
        config[5] = 0b01000000 | enb_FEC << 2 | tx_power; // default Push-pull, WOR 250 ms
    }

    void print_params() override {
        print_params(config);
    }

    void print_params(uint8_t config[8]) override {
        Serial.print("Write Mode ");
        Serial.println(config[0], 16);

        Serial.print("Address    ");
        Serial.println((config[1] << 8) + config[2]);

        Serial.print("Parity     ");
        uint8_t parity = (config[3] & 0b11000000) >> 6;
        switch (parity) {
            case LORA_8N1:
                Serial.println("8N1");
                break;
            case LORA_8O1:
                Serial.println("8O1");
                break;
            case LORA_8E1:
                Serial.println("8E1");
                break;
            default:
                Serial.println("N/A");
                break;
        }

        Serial.print("Baud       ");
        uint8_t baud_r = (config[3] & 0b00111000) >> 3;
        switch (baud_r) {
            case LORA_BAUD_1200:
                Serial.println("1200");
                break;
            case LORA_BAUD_2400:
                Serial.println("2400");
                break;
            case LORA_BAUD_4800:
                Serial.println("4800");
                break;
            case LORA_BAUD_9600:
                Serial.println("9600");
                break;
            case LORA_BAUD_19200:
                Serial.println("19200");
                break;
            case LORA_BAUD_38400:
                Serial.println("38400");
                break;
            case LORA_BAUD_57600:
                Serial.println("57600");
                break;
            case LORA_BAUD_115200:
                Serial.println("115200");
                break;
            default:
                Serial.println("N/A");
                break;
        }

        Serial.print("Data Rate  ");
        uint8_t data_r = (config[3] & 0b00000111);
        switch (data_r) {
            case LORA_RATE_1200:
                Serial.println("1200");
                break;
            case LORA_RATE_2400:
                Serial.println("2400");
                break;
            case LORA_RATE_4800:
                Serial.println("4800");
                break;
            case LORA_RATE_9600:
                Serial.println("9600");
                break;
            case LORA_RATE_19200:
                Serial.println("19200");
                break;
            case LORA_RATE_300:
                Serial.println("300");
                break;
            default:
                Serial.println("N/A");
                break;
        }

        Serial.print("Channel    ");
        Serial.println(config[4], 16);

        Serial.print("FEC        ");
        uint8_t fec = (config[5] & 0b00000100);
        if (fec)
            Serial.println("ENABLED");
        else
            Serial.println("DISABLED");

        Serial.print("Tx Power   ");
        uint8_t tx_pow = (config[5] & 0b00000011);
        switch (tx_pow) {
            case LORA_TX_MAX:
                Serial.println("Max (4)");
                break;
            case LORA_TX_HIGH:
                Serial.println("High (3)");
                break;
            case LORA_TX_LOW:
                Serial.println("Low (2)");
                break;
            default:
                Serial.println("Min (1)");
                break;
        }

        Serial.print("FIELD 5    ");
        Serial.println(config[5], 16);
    }

    void cmd_get_versions() override {
        write_triple(0xc3);
    }

    void cmd_reset_module() override {
        write_triple(0xc4);
    }
};
#endif

using impl::Sensors_GPS;
using impl::Sensors_IMU;

#endif //PINTO_V1_FIRMWARE_VNET_SENSORS_H
