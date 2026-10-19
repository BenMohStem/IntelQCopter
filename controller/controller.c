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

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "../common/qcopter_protocol.h"

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

static float centered_axis(uint16_t raw, float center) {
    const float delta = (float)raw - center;
    const float denominator = delta >= 0.0f ? 4095.0f - center : center;
    if (denominator < 1.0f) return 0.0f;
    float value = delta / denominator;
    if (fabsf(value) <= JOYSTICK_DEADBAND) return 0.0f;
    value = copysignf((fabsf(value) - JOYSTICK_DEADBAND)
                      / (1.0f - JOYSTICK_DEADBAND), value);
    return value < -1.0f ? -1.0f : (value > 1.0f ? 1.0f : value);
}

static int16_t axis_scaled(float value) {
    const long scaled = lroundf(value * (float)QCOPTER_AXIS_SCALE);
    return (int16_t)(scaled < -QCOPTER_AXIS_SCALE ? -QCOPTER_AXIS_SCALE
        : (scaled > QCOPTER_AXIS_SCALE ? QCOPTER_AXIS_SCALE : scaled));
}

static uint16_t trigger_scaled(uint16_t raw, uint16_t released) {
    const int32_t delta = (int32_t)raw - (int32_t)released;
    const uint32_t travel = (uint32_t)(delta < 0 ? -delta : delta);
    const uint32_t available = released > (4095u - released)
        ? released : (4095u - released);
    if (available == 0u) return 0u;
    const uint32_t value = travel * QCOPTER_TRIGGER_SCALE / available;
    return (uint16_t)(value > QCOPTER_TRIGGER_SCALE
        ? QCOPTER_TRIGGER_SCALE : value);
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
    uint16_t sequence = 0u;
    absolute_time_t deadline = make_timeout_time_us(HAND_LOOP_US);

    for (;;) {
        sleep_until(deadline);
        deadline = delayed_by_us(deadline, HAND_LOOP_US);

        const uint16_t left_x = read_mux_channel(0u);
        const uint16_t left_y = read_mux_channel(1u);
        const uint16_t right_x = read_mux_channel(2u);
        const uint16_t right_y = read_mux_channel(3u);
        const uint16_t lt = read_mux_channel(4u);
        const uint16_t rt = read_mux_channel(5u);
        const float yaw_axis = centered_axis(left_x, center[0]);
        const float vertical_axis = -centered_axis(left_y, center[1]);
        const float roll_axis = centered_axis(right_x, center[2]);
        const float pitch_axis = -centered_axis(right_y, center[3]);

        qcopter_command_frame_t command = {
            .sequence = sequence++,
            .roll = axis_scaled(roll_axis),
            .pitch = axis_scaled(pitch_axis),
            .yaw_rate = axis_scaled(yaw_axis),
            .throttle = (uint16_t)lroundf((vertical_axis + 1.0f) * 0.5f
                                          * QCOPTER_THROTTLE_SCALE),
            .lt = trigger_scaled(lt, trigger_released[0]),
            .rt = trigger_scaled(rt, trigger_released[1]),
            .buttons = read_buttons(debounce, &stable_buttons)
        };
        uint8_t encoded[QCOPTER_FRAME_SIZE];
        if (qcopter_encode_frame(&command, encoded) && (command.sequence % 10u) == 0u)
            print_frame_debug(&command, encoded); /* 10 Hz USB debug, not RF */
    }
}
