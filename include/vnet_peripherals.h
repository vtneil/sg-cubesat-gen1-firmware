#ifndef VNET_PERIPHERALS_H
#define VNET_PERIPHERALS_H

/** Includes */
#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include "vnet_definitions.h"
#include "vnet_arts.h"
#include "vnet_sensors.h"
#include "vnet_lora.h"
#include "vnet_tools.h"

/** Other Includes */
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "Fonts/TomThumb.h"

/** USB Interface */
//#include "Adafruit_TinyUSB.h"
//#include "arduino/msc/Adafruit_USBD_MSC.h"

class Storage_SD {
private:
    SdFat _sd;
    String _save_filename;

public:
    File file;

public:
    explicit
    Storage_SD(String &filename) {
        _sd.begin(53, SPI_FULL_SPEED);
        next_filename(filename);
        open();
    }

    explicit
    Storage_SD(const char *filename) {
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
    explicit
    Screen_OLED(uint8_t w = 128, uint8_t h = 64) {
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
        display->drawBitmap(0, 0, logo_spaceac_boot, 128, 64, 1);
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

using impl::Sensors_GPS;
using impl::Sensors_IMU;

#endif //VNET_PERIPHERALS_H