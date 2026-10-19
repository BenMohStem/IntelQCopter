#ifndef QCOPTER_PROTOCOL_H
#define QCOPTER_PROTOCOL_H

/* Fixed-size, endian-stable command frame shared by the two Pico firmwares. */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define QCOPTER_FRAME_SIZE 22u
#define QCOPTER_PROTOCOL_VERSION 1u
#define QCOPTER_AXIS_SCALE 10000
#define QCOPTER_TRIGGER_SCALE 1000u
#define QCOPTER_THROTTLE_SCALE 10000u

enum {
    QCOPTER_BUTTON_LB = 1u << 0,
    QCOPTER_BUTTON_RB = 1u << 1,
    QCOPTER_BUTTON_NORTH = 1u << 2,
    QCOPTER_BUTTON_EAST = 1u << 3,
    QCOPTER_BUTTON_SOUTH = 1u << 4,
    QCOPTER_BUTTON_WEST = 1u << 5
};

typedef struct {
    uint16_t sequence;
    int16_t roll;
    int16_t pitch;
    int16_t yaw_rate;
    uint16_t throttle;
    uint16_t lt;
    uint16_t rt;
    uint16_t buttons;
} qcopter_command_frame_t;

static inline uint16_t qcopter_crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
    }
    return crc;
}

static inline void qcopter_put_u16(uint8_t *p, uint16_t value) {
    p[0] = (uint8_t)(value & 0xFFu);
    p[1] = (uint8_t)(value >> 8);
}

static inline uint16_t qcopter_get_u16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline bool qcopter_frame_values_valid(const qcopter_command_frame_t *f) {
    return f != NULL
        && f->roll >= -QCOPTER_AXIS_SCALE && f->roll <= QCOPTER_AXIS_SCALE
        && f->pitch >= -QCOPTER_AXIS_SCALE && f->pitch <= QCOPTER_AXIS_SCALE
        && f->yaw_rate >= -QCOPTER_AXIS_SCALE && f->yaw_rate <= QCOPTER_AXIS_SCALE
        && f->throttle <= QCOPTER_THROTTLE_SCALE
        && f->lt <= QCOPTER_TRIGGER_SCALE && f->rt <= QCOPTER_TRIGGER_SCALE
        && (f->buttons & (uint16_t)~0x003Fu) == 0u;
}

static inline bool qcopter_encode_frame(const qcopter_command_frame_t *f,
                                        uint8_t out[QCOPTER_FRAME_SIZE]) {
    if (out == NULL || !qcopter_frame_values_valid(f)) return false;
    out[0] = 0x51u;
    out[1] = 0x43u;
    out[2] = QCOPTER_PROTOCOL_VERSION;
    out[3] = 0u;
    qcopter_put_u16(&out[4], f->sequence);
    qcopter_put_u16(&out[6], (uint16_t)f->roll);
    qcopter_put_u16(&out[8], (uint16_t)f->pitch);
    qcopter_put_u16(&out[10], (uint16_t)f->yaw_rate);
    qcopter_put_u16(&out[12], f->throttle);
    qcopter_put_u16(&out[14], f->lt);
    qcopter_put_u16(&out[16], f->rt);
    qcopter_put_u16(&out[18], f->buttons);
    qcopter_put_u16(&out[20], qcopter_crc16(out, 20u));
    return true;
}

static inline bool qcopter_decode_frame(const uint8_t *data, size_t length,
                                        qcopter_command_frame_t *out) {
    if (data == NULL || out == NULL || length != QCOPTER_FRAME_SIZE
        || data[0] != 0x51u || data[1] != 0x43u
        || data[2] != QCOPTER_PROTOCOL_VERSION || data[3] != 0u
        || qcopter_get_u16(&data[20]) != qcopter_crc16(data, 20u))
        return false;
    qcopter_command_frame_t f = {
        .sequence = qcopter_get_u16(&data[4]),
        .roll = (int16_t)qcopter_get_u16(&data[6]),
        .pitch = (int16_t)qcopter_get_u16(&data[8]),
        .yaw_rate = (int16_t)qcopter_get_u16(&data[10]),
        .throttle = qcopter_get_u16(&data[12]),
        .lt = qcopter_get_u16(&data[14]),
        .rt = qcopter_get_u16(&data[16]),
        .buttons = qcopter_get_u16(&data[18])
    };
    if (!qcopter_frame_values_valid(&f)) return false;
    *out = f;
    return true;
}

/* Valid for sequence gaps shorter than half the 16-bit sequence space. */
static inline bool qcopter_sequence_is_newer(uint16_t value, uint16_t previous) {
    return (int16_t)(value - previous) > 0;
}

#endif
