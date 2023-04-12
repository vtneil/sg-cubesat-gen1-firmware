/*
 * C++ Struct Serializer_S-Deserializer
 *
 * Source code by Vivatsathorn Thitasirivit
 *
 * **Warning**: Arrays in struct is NOT supported for cross-endian network.
 * If your network has the same endianness (not needing conversion),
 * you can use arrays.
 */

#ifndef VNET_SERIALIZER_H
#define VNET_SERIALIZER_H

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define VNET_DEFAULT_NETWORK_ENDIANNESS vnet::SYS_LITTLE_ENDIAN

#define packed_struct struct __attribute__((packed))
#define typedef_packed_struct typedef packed_struct

namespace vnet {
    template<typename S, typename... Ts>
    class Serializer_S;

    typedef enum {
        SYS_BIG_ENDIAN = 0,
        SYS_LITTLE_ENDIAN
    } Endianness;

    Endianness get_sys_endianness();

    template<typename S>
    uint8_t *make_buffer();

    template<typename S>
    uint8_t *make_buffer_clear();

    void clear_buffer(void *buff, size_t n);

    void rmemcpy(void *dst, const void *src, size_t n);

    template<typename T>
    T reverse_bits(T data);

    template<typename T>
    void printb(const T &data, size_t size = sizeof(T));

    template<typename T>
    void printb(T *data, size_t size);

    static Endianness SYSTEM_ENDIANNESS = get_sys_endianness();
    static Endianness NETWORK_ENDIANNESS = SYS_BIG_ENDIAN;

    static void __set_network_endianness(Endianness endianness) {
        NETWORK_ENDIANNESS = endianness;
    }

    namespace {
        template<typename T>
        class FuncIter {
        public:
            void operator()(uint8_t *dst, size_t &i) const {
                *(dst + i++) = sizeof(T);
            }
        };

        template<template<typename> class F, typename S, typename... Ts>
        void iter_types(uint8_t *dst, size_t &i) {
            int dummy[]{(F<Ts>{}(dst, i), 0)...};
            static_cast<void>(dummy);
        }
    }

    template<typename S, typename... Ts>
    class Serializer_S {
    private:
        size_t _param_count{sizeof...(Ts)};
        uint8_t *_param_sizes{nullptr};
        uint8_t *_ser_buf{nullptr};
        S _deser_buf{};

    public:
        uint8_t term[4] = {0xff, 0xff, 0xff, 0xff};

    public:
        Serializer_S() {
            _param_sizes = (uint8_t *) malloc((_param_count + 1) * sizeof(uint8_t));
            *_param_sizes = 0;
            size_t _i = 1;
            iter_types<FuncIter, S, Ts...>(_param_sizes, _i);
            _ser_buf = make_buffer_clear<S>();
        }

        ~Serializer_S() {
            if (_param_sizes)
                free(_param_sizes);
            if (_ser_buf)
                free(_ser_buf);
        }

        void serialize_to(uint8_t *dst, const S &src) {
            if (SYSTEM_ENDIANNESS == NETWORK_ENDIANNESS) {
                memcpy((void *) dst, (void *) &src, struct_size());
            } else {
                size_t curr_pos = 0;
                uint8_t *c_src = (uint8_t *) &src;
                for (size_t i = 0; i < _param_count; ++i) {
                    curr_pos += _param_sizes[i];
                    rmemcpy((void *) (dst + curr_pos), (void *) (c_src + curr_pos), _param_sizes[i + 1]);
                }
            }
        }

        uint8_t *serialize(const S &src) {
            serialize_to(_ser_buf, src);
            return _ser_buf;
        }

        void deserialize_to(S &dst, const uint8_t *src, size_t n = sizeof(S)) {
            if (SYSTEM_ENDIANNESS == NETWORK_ENDIANNESS) {
                memcpy(&dst, src, n);
            } else {
                size_t curr_pos = 0;
                uint8_t *c_dst = (uint8_t *) &dst;
                for (size_t i = 0; i < _param_count; ++i) {
                    curr_pos += _param_sizes[i];
                    rmemcpy((void *) (c_dst + curr_pos), (void *) (src + curr_pos), _param_sizes[i + 1]);
                }
            }
        }

        S deserialize(const uint8_t *src) {
            deserialize_to(_deser_buf, src);
            return _deser_buf;
        }

        constexpr size_t buffer_size() const {
            return sizeof(uint8_t) * struct_size();
        }

        constexpr size_t struct_size() const {
            return sizeof(S);
        }

        constexpr size_t param_count() const {
            return _param_count;
        }

        uint8_t *param_sizes() const {
            return _param_sizes;
        }

        uint8_t *buf() {
            return _ser_buf;
        }
    };

    Endianness get_sys_endianness() {
        uint16_t word = 0x01;
        auto *byte_arr = (uint8_t *) &word;
        return *byte_arr ? SYS_LITTLE_ENDIAN : SYS_BIG_ENDIAN;
    }

    void rmemcpy(void *dst, const void *src, size_t n) {
        uint8_t *c_dst = (uint8_t *) dst;
        uint8_t *c_src = (uint8_t *) src + n - 1;
        while (n--) {
            *c_dst++ = *c_src--;
        }
    }

    template<typename S>
    S reverse_bits(S data) {
        S result = {0};
        size_t n = sizeof(S) / sizeof(uint8_t);
        uint8_t *c_data = (uint8_t *) &data;
        uint8_t *tmp = (uint8_t *) &result;

        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < 8 * sizeof(uint8_t); ++j) {
                *(tmp + n - i - 1) <<= 1;
                *(tmp + n - i - 1) |= *(c_data + i) & 1;
                *(c_data + i) >>= 1;
            }
        }

        return *((S *) tmp);
    }

    template<typename T>
    void printb(const T &data, size_t size) {
        uint8_t *data_r = (uint8_t *) &data;
        printf("DATA | ");
        for (size_t i = 0; i < size; ++i)
            printf("%02x ", data_r[i]);
        if (SYSTEM_ENDIANNESS) printf("(MSB)");
        else printf("(LSB)");
        printf("\n");
        printf("ADDR | ");
        for (size_t i = 0; i < size; ++i)
            printf("%02lx ", i);
        printf("(HIGH ADDR)\n");
    }

    template<typename S>
    void printb(S *data, size_t size) {
        uint8_t *data_r = (uint8_t *) data;
        printf("DATA | ");
        for (size_t i = 0; i < size; ++i)
            printf("%02x ", data_r[i]);
        if (SYSTEM_ENDIANNESS) printf("(MSB)");
        else printf("(LSB)");
        printf("\n");
        printf("ADDR | ");
        for (size_t i = 0; i < size; ++i)
            printf("%02lx ", i);
        printf("(HIGH ADDR)\n");
    }

    template<typename S>
    uint8_t *make_buffer() {
        return (uint8_t *) malloc(sizeof(S));
    }

    template<typename S>
    uint8_t *make_buffer_clear() {
        return (uint8_t *) calloc(1, sizeof(S));
    }

    void clear_buffer(void *buff, size_t n) {
        memset(buff, 0, n);
    }

    void hrx(char c, size_t n = 50) {
        while (--n)
            printf("%c", c);
        printf("\n");
    }

    void hr(size_t n = 50) {
        hrx('-', n);
    }

    void hrb(size_t n = 50) {
        hrx('#', n);
    }

    void hrd(size_t n = 50) {
        hrx('=', n);
    }

    void hrt(size_t n = 50) {
        hrx('~', n);
    }

    void hrp(size_t n = 50) {
        hrx('+', n);
    }

    template<typename T>
    using Serializer = Serializer_S<T, T>;
}

using vnet::Serializer;
using vnet::Serializer_S;

#endif  //VNET_SERIALIZER_H
