#pragma once
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <deque>
#include <string>
#include <vector>

// Host model only. Its clock/I2C costs are synthetic, NOT measured MCU timing.
static uint64_t model_time_us = 0;
static uint64_t next_packet_us = 0;
static uint8_t registers_model[128] = {};
static std::deque<uint8_t> fifo_model;
static bool overflow_model = false;
static bool fail_reads = false;
static bool retained_boot = false;
static bool invalid_boot = false;
static bool invalid_gyro_boot = false;
static bool corrupt_trim_writes = false;
static bool reject_fifo_enable = false;
static unsigned failed_read_address = 255;
static unsigned bus_cost_multiplier = 1;
static unsigned pin_calls = 0;
struct PinModeCall
{
    int pin;
    int mode;
};
static std::vector<PinModeCall> pin_mode_calls;
static int16_t model_gyro_counts[3] = {1,-2,3};

static void set_word(unsigned address, int16_t value)
{
    registers_model[address] = (uint16_t)value >> 8;
    registers_model[address + 1] = (uint16_t)value & 255;
}

static int16_t get_word(unsigned address)
{
    return (int16_t)(((uint16_t)registers_model[address] << 8) | registers_model[address + 1]);
}

static void advance_model(uint64_t amount)
{
    model_time_us += amount;
    if (!(registers_model[0x6A] & 0x40) || registers_model[0x23] != 0xF8)
    {
        next_packet_us = model_time_us + 1000;
        return;
    }
    if (!next_packet_us)
    {
        next_packet_us = model_time_us + 1000;
    }
    while (next_packet_us <= model_time_us)
    {
        const int16_t packet[7] = {1000, -100, 4000, 30, model_gyro_counts[0], model_gyro_counts[1], model_gyro_counts[2]};
        if (fifo_model.size() + 14 > 1024)
        {
            overflow_model = true;
            next_packet_us = model_time_us + 1000;
            return;
        }
        for (int16_t value : packet)
        {
            fifo_model.push_back((uint16_t)value >> 8);
            fifo_model.push_back((uint16_t)value & 255);
        }
        next_packet_us += 1000;
    }
}

static uint32_t micros()
{
    advance_model(2);
    return (uint32_t)model_time_us;
}
static void delay(unsigned ms) { advance_model((uint64_t)ms * 1000); }
static void delayMicroseconds(unsigned us) { advance_model(us); }
static const int INPUT_PULLDOWN = 1;
static const int MSBFIRST = 1;
static void pinMode(int pin, int mode)
{
    assert(mode == INPUT_PULLDOWN);
    assert(pin == 12 || pin == 1 || pin == 2 || pin == 14 || pin == 4 || pin == 21);
    pin_mode_calls.push_back({pin, mode});
    pin_calls++;
}
#define PI 3.14159265358979323846
#define RAD_TO_DEG (180.0 / PI)
template<typename T> static T max(T a, T b) { return a > b ? a : b; }

struct SerialModel
{
    std::deque<char> input;
    std::string output;
    void setTxBufferSize(unsigned) {}
    void begin(unsigned baud) { assert(baud == 921600); }
    bool available() { return !input.empty(); }
    char read() { char c = input.front(); input.pop_front(); return c; }
    void flush() {}
    void printf(const char *format, ...)
    {
        char buffer[1024];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        output += buffer;
    }
};
static SerialModel Serial;
struct EspModel
{
    const char *getChipModel() { return "HOST_MODEL_NOT_ESP32"; }
    unsigned long long getEfuseMac() { return 0; }
};
static EspModel ESP;
struct WireModel
{
    uint32_t clock = 400000;
    bool begin(int sda, int scl, unsigned hz) { assert(sda == 9 && scl == 8); clock = hz; return true; }
    void setClock(unsigned value) { clock = value; }
    uint32_t getClock() { return clock; }
    void setTimeOut(unsigned) {}
};
static WireModel Wire;
static const int MPU6050_RANGE_8_G = 2;
static const int MPU6050_RANGE_500_DEG = 1;
static const int MPU6050_BAND_260_HZ = 0;
static const int MPU6050_PLL_GYROX = 1;
struct Adafruit_MPU6050
{
    bool begin(int address, WireModel *)
    {
        assert(address == 0x68);
        for (auto &value : registers_model) { value = 0; }
        const int16_t baseline[3] = {-2074, 2979, 1074};
        const int16_t targets[3] = {-2348, 2993, 1212};
        const int16_t gyro[3] = {-6, 9, 27};
        for (unsigned i = 0; i < 3; ++i)
        {
            set_word(0x06 + i * 2, retained_boot ? targets[i] : baseline[i]);
            set_word(0x13 + i * 2, retained_boot ? gyro[i] : 0);
        }
        if (invalid_boot) { set_word(0x06, -999); }
        if (invalid_gyro_boot) { set_word(0x13, 123); }
        return true;
    }
    void setAccelerometerRange(int v) { registers_model[0x1C] = (uint8_t)(v << 3); }
    int getAccelerometerRange() { return registers_model[0x1C] >> 3; }
    void setGyroRange(int v) { registers_model[0x1B] = (uint8_t)(v << 3); }
    int getGyroRange() { return registers_model[0x1B] >> 3; }
    void setFilterBandwidth(int v) { registers_model[0x1A] = (uint8_t)v; }
    int getFilterBandwidth() { return registers_model[0x1A]; }
    void setSampleRateDivisor(int v) { registers_model[0x19] = (uint8_t)v; }
    int getSampleRateDivisor() { return registers_model[0x19]; }
    void setClock(int v) { registers_model[0x6B] = (uint8_t)v; }
    int getClock() { return registers_model[0x6B]; }
};

struct Adafruit_I2CDevice
{
    Adafruit_I2CDevice(int address, WireModel *) { assert(address == 0x68); }
    bool begin() { return true; }
    bool write_then_read(const uint8_t *address, size_t sent, uint8_t *bytes, size_t length)
    {
        assert(sent == 1);
        advance_model((9 * (length + 3) * 5 + 1) / 2 * bus_cost_multiplier);
        if (fail_reads || *address == failed_read_address) { return false; }
        if (*address == 0x74)
        {
            if (fifo_model.size() < length) { return false; }
            for (size_t i = 0; i < length; ++i) { bytes[i] = fifo_model.front(); fifo_model.pop_front(); }
        }
        else if (*address == 0x72 && length == 2)
        {
            bytes[0] = (uint16_t)fifo_model.size() >> 8;
            bytes[1] = (uint16_t)fifo_model.size() & 255;
        }
        else if (*address == 0x3A && length == 1)
        {
            bytes[0] = overflow_model ? 0x10 : 0;
            overflow_model = false;
        }
        else
        {
            for (size_t i = 0; i < length; ++i) { bytes[i] = registers_model[*address + i]; }
        }
        return true;
    }
};

struct Adafruit_BusIO_Register
{
    unsigned address, width;
    Adafruit_BusIO_Register(Adafruit_I2CDevice *, unsigned a, unsigned w = 1, int = MSBFIRST) : address(a), width(w) {}
    bool read(uint16_t *value)
    {
        advance_model((9 * (width + 3) * 5 + 1) / 2 * bus_cost_multiplier);
        if (fail_reads) { return false; }
        *value = width == 2 ? (uint16_t)get_word(address) : registers_model[address];
        return true;
    }
    bool write(uint16_t value)
    {
        advance_model((9 * (width + 2) * 5 + 1) / 2 * bus_cost_multiplier);
        if (address == 0x23 && value == 0xF8 && reject_fifo_enable) { return false; }
        if (width == 2) { set_word(address, (int16_t)(corrupt_trim_writes ? value ^ 1 : value)); }
        else
        {
            registers_model[address] = (uint8_t)value;
            if (address == 0x6A && (value & 0x04))
            {
                fifo_model.clear(); overflow_model = false; registers_model[address] &= ~0x04;
            }
            if (address == 0x23 && value == 0xF8) { next_packet_us = model_time_us + 1000; }
        }
        return true;
    }
};
