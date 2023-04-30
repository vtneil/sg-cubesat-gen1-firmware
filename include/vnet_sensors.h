#ifndef VNET_SENSORS_H
#define VNET_SENSORS_H

/** Includes */
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

/** Other Includes */
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

/** Definitions */

/**
 * Mean Sea Level Pressure in hPa
 */
#define MSLP_HPA (1013.25)
#define M_G (9.81)

/** Typedefs */
typedef struct {
    double r;
    double i;
    double j;
    double k;
} Vec4_t;

typedef Vec4_t Quaternion_t;

typedef struct {
    double x;
    double y;
} Vec2_t;

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
} GPS_Type;

/** Sensors Tools Classes */
namespace impl {
    class Sensors_Global {
    protected:
        bool m_valid = false;
        Peripherals_t *list = nullptr;

        explicit
        Sensors_Global(Peripherals_t *sensors_list) : list{sensors_list} {}

    public:
        Peripherals_t &devices() {
            return *list;
        }

        bool valid() const {
            return m_valid;
        }
    };

    class Device_I2C {
    protected:
        TwoWire *wire;

        explicit Device_I2C(TwoWire *wire) : wire(wire) {
            static bool _x = begin_I2C();
            static_cast<void>(_x);  // to suppress compiler warnings.
        }

        bool begin_I2C() {
            wire->begin();
            wire->setClock(400000UL);
            return true;
        }
    };

    class Sensors_GPS : protected impl::Sensors_Global {
    protected:
        Sensors_Data_GPS_Position_t *pos = nullptr;
        uint32_t last_millis = millis();

    public:
        explicit
        Sensors_GPS(Peripherals_t *sensors_list) : Sensors_Global(sensors_list) {
            pos = new Sensors_Data_GPS_Position_t();
        }

        virtual
        Sensors_Data_GPS_Position_t &read_position() = 0;
    };

    class Sensors_IMU : protected impl::Sensors_Global {
    protected:
        Sensors_Data_IMU_t data = {};
    public:
        explicit
        Sensors_IMU(Peripherals_t *sensors_list) : Sensors_Global(sensors_list) {}

        virtual
        Euler_t read_rotation() = 0;

        virtual
        Vec3_t read_acceleration() = 0;

        virtual
        Vec3_t read_gyroscope() = 0;

        virtual
        Vec3_t read_magnetic_field() = 0;

        Sensors_Data_IMU_t &read_data() {
            return data;
        }

        static Euler_t quaternion_to_euler(Quaternion_t &quaternion, bool degrees = true) {
            double sqr = SQUARE(quaternion.r);
            double sqi = SQUARE(quaternion.i);
            double sqj = SQUARE(quaternion.j);
            double sqk = SQUARE(quaternion.k);

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

class Sensors_Environmental : protected impl::Sensors_Global, protected impl::Device_I2C {
private:
    static constexpr uint8_t pin_uv = A0;
    static constexpr uint8_t pin_33 = A1;
    uint8_t bme_id = 0;
    Adafruit_BME280 *bme280;
    Adafruit_CCS811 *ccs811;

public:
    explicit
    Sensors_Environmental(Peripherals_t *sensors_list,
                          uint8_t bme280_addr = ADDR_BME280,
                          uint8_t ccs811_addr = ADDR_CCS811,
                          TwoWire *wire = &Wire) : Sensors_Global(sensors_list), Device_I2C(wire) {
        bme_id = bme280_addr - ADDR_BME280;
        bme280 = new Adafruit_BME280();
        ccs811 = new Adafruit_CCS811();
        list->env_bme280[bme_id] = bme280->begin(bme280_addr, wire);
        list->env_ccs811 = ccs811->begin(ccs811_addr, wire);
        if (!list->env_bme280[bme_id]) {
            delete bme280;
            bme280 = nullptr;
        }
        if (!list->env_ccs811) {
            delete ccs811;
            ccs811 = nullptr;
        }
        m_valid = list->env_bme280[bme_id] && list->env_ccs811;
    }

    Sensors_Data_PHT_t read_PHT() {
        if (list->env_bme280[bme_id]) {
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
        return MAP(3.3 * lvl_uv / lvl_33, 0.99, 2.8, 0.0, 15.0);
    }
};

class Sensors_Environmental_DS18B20 : protected impl::Sensors_Global {
private:
    OneWire one_wire_int{0};
    DallasTemperature sensor;
    DeviceAddress addr = {};
public:
    explicit
    Sensors_Environmental_DS18B20(Peripherals_t *sensors_list) :
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

class Sensors_GPS_SparkFun : public impl::Sensors_GPS {
private:
    SFE_UBLOX_GNSS *gps_SparkFun = nullptr;

public:
    explicit
    Sensors_GPS_SparkFun(Peripherals_t *sensors_list) : Sensors_GPS(sensors_list) {
        gps_SparkFun = new SFE_UBLOX_GNSS();
        m_valid = list->gps_SparkFun = gps_SparkFun->begin();
        if (list->gps_SparkFun) {
            gps_SparkFun->setI2COutput(COM_TYPE_UBX);
            gps_SparkFun->setDynamicModel(DYN_MODEL_AIRBORNE2g);
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
    explicit
    Sensors_GPS_Serial(Peripherals_t *sensors_list,
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

class Sensors_IMU_BNO085 : public impl::Sensors_IMU, protected impl::Device_I2C {
private:
    Adafruit_BNO08x *bno085 = nullptr;
    sh2_SensorValue_t *sensor_value = nullptr;
    static constexpr uint32_t sensor_interval_us = 100000;
    static constexpr uint8_t sensor_count = 4;
    Quaternion_t quaternion = {};

public:
    explicit
    Sensors_IMU_BNO085(Peripherals_t *sensors_list,
                       uint8_t bno085_addr = ADDR_BNO085,
                       TwoWire *wire = &Wire) : Sensors_IMU(sensors_list), Device_I2C(wire) {
        bno085 = new Adafruit_BNO08x();
        sensor_value = new sh2_SensorValue_t();

        m_valid = list->imu_bno085 = bno085->begin_I2C(bno085_addr, wire);
        if (list->imu_bno085) {
            /** Set interval to 10 ms (100 Hz) */
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

class Sensors_IMU_BNO055 : public impl::Sensors_IMU, protected impl::Device_I2C {
private:
    Adafruit_BNO055 *bno055 = nullptr;
    sensors_event_t orientation_data = {};
    sensors_event_t accelerometer_data = {};
    sensors_event_t gyroscope_data = {};
    sensors_event_t magnetic_data = {};
public:
    explicit
    Sensors_IMU_BNO055(Peripherals_t *sensors_list,
                       uint8_t bno055_addr = ADDR_BNO055,
                       TwoWire *wire = &Wire) : Sensors_IMU(sensors_list), Device_I2C(wire) {
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

class Sensors_IMU_ICM20948 : public impl::Sensors_IMU, protected impl::Device_I2C {
private:
    ICM_20948_I2C *icm20948 = nullptr;

public:
    explicit
    Sensors_IMU_ICM20948(Peripherals_t *sensors_list,
                         TwoWire *wire) : Sensors_IMU(sensors_list), Device_I2C(wire) {
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

#endif //VNET_SENSORS_H
