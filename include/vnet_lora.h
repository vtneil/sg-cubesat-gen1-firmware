#ifndef VNET_LORA_H
#define VNET_LORA_H

#include <Arduino.h>

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
        /**
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

        /**
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

/**
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

/**
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
/**
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

#endif //VNET_LORA_H
