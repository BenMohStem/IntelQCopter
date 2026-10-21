/*
 * RP2040 handheld input scanner and command-frame producer.
 *
 * Board assumptions are centralized below. The proposed six-channel analog
 * mux is required for four joystick axes plus two analog triggers on a normal
 * Pico, which exposes only three general-purpose ADC inputs. Radio hardware
 * and RF settings stay disabled until the LoRa module/band are confirmed.
 */
#include <math.h>
#include <stdio.h>
#include <stdint.h>

#ifndef QCOPTER_HANDLER_HOST_TEST
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#endif
/* BEGIN WIRE PROTOCOL v1: keep identical in both firmware files. */
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
/* END WIRE PROTOCOL v1 */


#define HAND_LOOP_HZ 100u
#define HAND_LOOP_US (1000000u / HAND_LOOP_HZ)
#define ADC_MUX_GPIO 26u
#define ADC_MUX_S0_GPIO 6u
#define ADC_MUX_S1_GPIO 7u
#define ADC_MUX_S2_GPIO 8u
#define BUTTON_DEBOUNCE_SAMPLES 3u
#define JOYSTICK_DEADBAND 0.04f
#define CENTER_CALIBRATION_SAMPLES 128u
#define BUTTON_COUNT 6u

/* Pure input processing: shared by the Pico loop and host regression tests.
 * Endpoints are provisional until a guided full-travel capture is available. */
typedef struct {
    float minimum[4], center[4], maximum[4];
    float trigger_released[2], trigger_pressed[2];
    float deadband, expo, filter_tau_s, slew_per_s;
    bool valid;
} handler_calibration_t;
typedef struct {
    float filtered[4], shaped[4],takeoff_hold_s,land_hold_s;
    bool initialized,takeoff_latched,land_latched;
    uint16_t sequence;uint8_t mission_events; /* 1 takeoff REQUEST, 2 land REQUEST; never arming */
} handler_state_t;

static bool handler_process(handler_state_t *state, const handler_calibration_t *cal,
                             const uint16_t raw[6], uint16_t buttons, float dt,
                             qcopter_command_frame_t *out) {
    if(!state || !cal || !raw || !out || !cal->valid || !isfinite(dt) || dt<=0 || dt>.05f
       || !isfinite(cal->deadband) || cal->deadband<0 || cal->deadband>=.5f
       || !isfinite(cal->expo) || cal->expo<0 || cal->expo>1
       || !isfinite(cal->filter_tau_s) || cal->filter_tau_s<0
       || !isfinite(cal->slew_per_s) || cal->slew_per_s<=0 || (buttons&~0x3Fu))return false;
    for(unsigned i=0;i<6;i++)if(raw[i]>4095)return false;
    float values[4];
    for(unsigned i=0;i<4;i++) {
        if(!isfinite(cal->minimum[i]) || !isfinite(cal->center[i]) || !isfinite(cal->maximum[i])
           || cal->minimum[i]<0 || cal->maximum[i]>4095
           || cal->center[i]-cal->minimum[i]<256 || cal->maximum[i]-cal->center[i]<256)return false;
        float delta=raw[i]-cal->center[i];
        float v=delta/(delta>=0 ? cal->maximum[i]-cal->center[i] : cal->center[i]-cal->minimum[i]);
        v=fmaxf(-1,fminf(1,v));
        v=fabsf(v)<=cal->deadband ? 0 : copysignf((fabsf(v)-cal->deadband)/(1-cal->deadband),v);
        values[i]=(1-cal->expo)*v+cal->expo*v*v*v;
    }
    float triggers[2];
    for(unsigned i=0;i<2;i++) {
        float travel=cal->trigger_pressed[i]-cal->trigger_released[i];
        if(!isfinite(travel) || !isfinite(cal->trigger_released[i]) || fabsf(travel)<256
          || cal->trigger_pressed[i]<0 || cal->trigger_pressed[i]>4095
          || cal->trigger_released[i]<0 || cal->trigger_released[i]>4095)return false;
        triggers[i]=fmaxf(0,fminf(1,(raw[i+4]-cal->trigger_released[i])/travel));
    }
    // Validate the entire capture before changing state; malformed input is atomic.
    const float alpha=cal->filter_tau_s>0 ? -expm1f(-dt/cal->filter_tau_s) : 1;
    for(unsigned i=0;i<4;i++) {
        if(!state->initialized)state->filtered[i]=state->shaped[i]=0;
        state->filtered[i]+=alpha*(values[i]-state->filtered[i]);
        float change=state->filtered[i]-state->shaped[i],limit=cal->slew_per_s*dt;
        state->shaped[i]+=fmaxf(-limit,fminf(limit,change));
    }
    state->initialized=true;
    *out=(qcopter_command_frame_t){.sequence=state->sequence++,
        .roll=(int16_t)lroundf(state->shaped[2]*QCOPTER_AXIS_SCALE),
        .pitch=(int16_t)lroundf(-state->shaped[3]*QCOPTER_AXIS_SCALE),
        .yaw_rate=(int16_t)lroundf(state->shaped[0]*QCOPTER_AXIS_SCALE),
        .throttle=(uint16_t)lroundf((1-state->shaped[1])*.5f*QCOPTER_THROTTLE_SCALE),
        .lt=(uint16_t)lroundf(triggers[0]*QCOPTER_TRIGGER_SCALE),
        .rt=(uint16_t)lroundf(triggers[1]*QCOPTER_TRIGGER_SCALE),.buttons=buttons};
    bool neutral=triggers[1]<.05f;
    for(unsigned i=0;i<4;i++)neutral=neutral&&fabsf(state->shaped[i])<.05f;
    const bool takeoff=neutral&&(buttons&(QCOPTER_BUTTON_RB|QCOPTER_BUTTON_NORTH))==(QCOPTER_BUTTON_RB|QCOPTER_BUTTON_NORTH);
    const bool land=neutral&&(buttons&(QCOPTER_BUTTON_LB|QCOPTER_BUTTON_SOUTH))==(QCOPTER_BUTTON_LB|QCOPTER_BUTTON_SOUTH);
    state->mission_events=0;
    state->takeoff_hold_s=takeoff&&!land ? state->takeoff_hold_s+dt : 0;
    state->land_hold_s=land&&!takeoff ? state->land_hold_s+dt : 0;
    if(!takeoff)state->takeoff_latched=false;
    if(!land)state->land_latched=false;
    if(state->takeoff_hold_s>=1&&!state->takeoff_latched){state->mission_events|=1;state->takeoff_latched=true;}
    if(state->land_hold_s>=1&&!state->land_latched){state->mission_events|=2;state->land_latched=true;}
    return qcopter_frame_values_valid(out);
}

#ifndef QCOPTER_HANDLER_HOST_TEST

/* GP26/ADC0 reads a 74HC4051 analog mux. Mux channels: LX, LY, RX, RY, LT, RT. */
static const uint8_t mux_select_gpio[3] = {
    ADC_MUX_S0_GPIO, ADC_MUX_S1_GPIO, ADC_MUX_S2_GPIO
};
static const uint8_t button_gpio[BUTTON_COUNT] = {9u, 10u, 11u, 12u, 13u, 14u};
static const uint16_t button_mask[BUTTON_COUNT] = {
    QCOPTER_BUTTON_LB, QCOPTER_BUTTON_RB,
    QCOPTER_BUTTON_NORTH, QCOPTER_BUTTON_EAST,
    QCOPTER_BUTTON_SOUTH, QCOPTER_BUTTON_WEST
};

static void mux_select(uint8_t channel) {
    for (unsigned bit = 0; bit < 3; ++bit)
        gpio_put(mux_select_gpio[bit], (channel >> bit) & 1u);
    /* Allow the mux output and ADC sample capacitor to settle. */
    busy_wait_us_32(8u);
}

static uint16_t read_mux_channel(uint8_t channel) {
    mux_select(channel);
    (void)adc_read(); /* discard the first conversion after switching channels */
    return adc_read();
}

static uint16_t read_buttons(uint8_t debounce[BUTTON_COUNT],
                             uint16_t *stable_state) {
    for (unsigned i = 0; i < BUTTON_COUNT; ++i) {
        const bool pressed = !gpio_get(button_gpio[i]); /* active-low switches */
        if (pressed && debounce[i] < BUTTON_DEBOUNCE_SAMPLES) debounce[i]++;
        if (!pressed && debounce[i] > 0u) debounce[i]--;
        if (debounce[i] == BUTTON_DEBOUNCE_SAMPLES)
            *stable_state |= button_mask[i];
        else if (debounce[i] == 0u)
            *stable_state &= (uint16_t)~button_mask[i];
    }
    return *stable_state;
}

static void initialize_io(void) {
    adc_init();
    adc_gpio_init(ADC_MUX_GPIO);
    adc_select_input(0u);
    for (unsigned i = 0; i < 3; ++i) {
        gpio_init(mux_select_gpio[i]);
        gpio_set_dir(mux_select_gpio[i], GPIO_OUT);
        gpio_put(mux_select_gpio[i], false);
    }
    for (unsigned i = 0; i < BUTTON_COUNT; ++i) {
        gpio_init(button_gpio[i]);
        gpio_set_dir(button_gpio[i], GPIO_IN);
        gpio_pull_up(button_gpio[i]);
    }
}

static bool calibrate_neutral(float center[4], uint16_t trigger_released[2]) {
    uint32_t sum[6] = {0u};
    uint16_t minimum[4] = {4095u, 4095u, 4095u, 4095u};
    uint16_t maximum[4] = {0u, 0u, 0u, 0u};
    for (unsigned sample = 0; sample < CENTER_CALIBRATION_SAMPLES; ++sample) {
        for (uint8_t channel = 0; channel < 6u; ++channel) {
            const uint16_t raw = read_mux_channel(channel);
            sum[channel] += raw;
            if (channel < 4u) {
                if (raw < minimum[channel]) minimum[channel] = raw;
                if (raw > maximum[channel]) maximum[channel] = raw;
            }
        }
    }
    for (unsigned axis = 0; axis < 4; ++axis) {
        center[axis] = (float)sum[axis] / (float)CENTER_CALIBRATION_SAMPLES;
        if (center[axis] < 500.0f || center[axis] > 3595.0f
            || (uint16_t)(maximum[axis] - minimum[axis]) > 160u)
            return false;
    }
    for (unsigned trigger = 0; trigger < 2; ++trigger)
        trigger_released[trigger] = (uint16_t)(sum[4u + trigger]
            / CENTER_CALIBRATION_SAMPLES);
    return true;
}

static void print_frame_debug(const qcopter_command_frame_t *frame,
                              const uint8_t encoded[QCOPTER_FRAME_SIZE]) {
    printf("seq=%u roll=%d pitch=%d yaw=%d throttle=%u LT=%u RT=%u buttons=0x%02X frame=",
           frame->sequence, frame->roll, frame->pitch, frame->yaw_rate,
           frame->throttle, frame->lt, frame->rt, frame->buttons);
    for (unsigned i = 0; i < QCOPTER_FRAME_SIZE; ++i) printf("%02X", encoded[i]);
    puts("");
}

int main(void) {
    stdio_init_all();
    initialize_io();
    sleep_ms(1200u);
    puts("QCopter RP2040 controller input scanner");
    puts("Keep both sticks centered and both triggers released for startup calibration.");
    puts("RF disabled: LoRa module, frequency, antenna, and link timing are unconfirmed.");

    float center[4] = {2048.0f, 2048.0f, 2048.0f, 2048.0f};
    uint16_t trigger_released[2] = {0u, 0u};
    if (!calibrate_neutral(center, trigger_released)) {
        puts("Stick-center calibration failed: center both sticks and retry power-up.");
        for (;;) sleep_ms(1000u);
    }
    uint8_t debounce[BUTTON_COUNT] = {0u};
    uint16_t stable_buttons = 0u;
    handler_state_t processor={0};
    handler_calibration_t cal={.minimum={0,0,0,0},.maximum={4095,4095,4095,4095},
        .deadband=JOYSTICK_DEADBAND,.expo=.25f,.filter_tau_s=.035f,.slew_per_s=3,.valid=true};
    for(unsigned i=0;i<4;i++)cal.center[i]=center[i];
    for(unsigned i=0;i<2;i++) {
        cal.trigger_released[i]=trigger_released[i];
        cal.trigger_pressed[i]=trigger_released[i]<2048 ? 4095 : 0;
    }
    absolute_time_t deadline = make_timeout_time_us(HAND_LOOP_US);
    uint64_t last_sample_us=time_us_64();

    for (;;) {
        sleep_until(deadline);
        deadline = absolute_time_diff_us(deadline,get_absolute_time())>(int64_t)HAND_LOOP_US
            ? make_timeout_time_us(HAND_LOOP_US) : delayed_by_us(deadline,HAND_LOOP_US);
        const uint64_t sampled_us=time_us_64();
        const float sample_dt=(float)(sampled_us-last_sample_us)*1e-6f;
        last_sample_us=sampled_us;

        const uint16_t left_x = read_mux_channel(0u);
        const uint16_t left_y = read_mux_channel(1u);
        const uint16_t right_x = read_mux_channel(2u);
        const uint16_t right_y = read_mux_channel(3u);
        const uint16_t lt = read_mux_channel(4u);
        const uint16_t rt = read_mux_channel(5u);
        const uint16_t raw[6]={left_x,left_y,right_x,right_y,lt,rt};
        qcopter_command_frame_t command;
        if(!handler_process(&processor,&cal,raw,read_buttons(debounce,&stable_buttons),
                            sample_dt,&command))continue;
        if(processor.mission_events&1)puts("TAKEOFF REQUEST: requires companion authorization; hardware remains disarmed");
        if(processor.mission_events&2)puts("LAND REQUEST: requires companion authorization; hardware remains disarmed");
        uint8_t encoded[QCOPTER_FRAME_SIZE];
        if (qcopter_encode_frame(&command, encoded) && (command.sequence % 10u) == 0u)
            print_frame_debug(&command, encoded); /* 10 Hz USB debug, not RF */
    }
}
#endif
