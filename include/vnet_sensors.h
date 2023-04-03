#ifndef PINTO_V1_FIRMWARE_VNET_SENSORS_H
#define PINTO_V1_FIRMWARE_VNET_SENSORS_H

/* Includes */
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
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
    void setupPD4() {
        DDRD &= ~(1 << PD4);
    }

    uint8_t readPD4() {
        return (PIND & (1 << PD4)) >> PD4;
    }

#undef DIRECT_MODE_INPUT
#define DIRECT_MODE_INPUT() impl::setupPD4()
#undef DIRECT_MODE_OUTPUT
#define DIRECT_MODE_OUTPUT() impl::setupPD4()
#undef DIRECT_WRITE_LOW
#define DIRECT_WRITE_LOW() (PORTD &= ~(1 << PD4))
#undef DIRECT_READ
#define DIRECT_READ() impl::readPD4()

    class OneWirePD4 : public OneWire {
    public:
        OneWirePD4() : OneWire(0) {}

        void write_bit(uint8_t v) {
            PAUSE_INTERRUPT(
                    if (v & 1) {
                        DIRECT_MODE_OUTPUT(); // make pin an output, do first since it
                        DIRECT_WRITE_LOW();   // will allow change to input while in output
                        delayMicroseconds(10);
                        DIRECT_MODE_INPUT(); // now input
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
                    r = impl::readPD4();
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

using impl::Sensors_GPS;
using impl::Sensors_IMU;

#endif //PINTO_V1_FIRMWARE_VNET_SENSORS_H
