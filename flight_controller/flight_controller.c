/*
 * QCopter flight-controller foundation.
 *
 * This file contains hardware-independent estimation and control logic so the
 * same C implementation can be connected to a future Isaac Sim HIL backend.
 * Hardware sensor and ESC drivers are intentionally not enabled: board pins,
 * ESC protocol, frame orientation, and motor order have not been confirmed.
 * Therefore this firmware remains disarmed and never produces motor outputs.
 */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include "../common/qcopter_protocol.h"

#ifndef QCOPTER_HOST_TEST
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#endif

#define FC_LOOP_HZ 500.0f
#define FC_LOOP_PERIOD_US 2000u
#define FC_LOOP_DT (1.0f / FC_LOOP_HZ)
#define FC_MAX_CONTROL_DT 0.010f
#define FC_RAD_TO_DEG 57.2957795f
#define FC_DEG_TO_RAD 0.0174532925f
#define FC_HOVER_THROTTLE_PLACEHOLDER 0.50f
#define FC_MAX_VERTICAL_SPEED_MPS 2.0f
#define FC_RADIO_TIMEOUT_US 150000u
#define FC_SENSOR_LOOP_HZ 1000u

typedef struct {
    float kp;
    float ki;
    float kd;
    float integrator;
    float previous_measurement;
    float integrator_limit;
    float output_limit;
    bool initialized;
} pid_t;

typedef struct {
    float angle_deg;
    float gyro_bias_dps;
    float p00;
    float p01;
    float p10;
    float p11;
    float gyro_noise;
    float bias_walk;
    float accel_noise;
} angle_kf_t;

typedef struct {
    float height_m;
    float velocity_mps;
    float p00;
    float p01;
    float p10;
    float p11;
    float accel_noise;
    float baro_noise;
} altitude_kf_t;

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_rate_dps;
    float throttle; /* [0,1], center requests zero vertical speed */
    bool valid;
    bool arm_request;
} pilot_command_t;

typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float vertical_acceleration_mps2; /* gravity compensated, world-up positive */
    float barometer_height_m;
    bool imu_valid;
    bool barometer_valid;
} sensor_sample_t;

typedef struct {
    float motor[4]; /* normalized math result only; no ESC output is connected */
    bool armed;
    bool sensor_fault;
} control_output_t;

typedef struct {
    pilot_command_t latest;
    uint16_t last_sequence;
    uint64_t last_received_us;
    bool received;
} pilot_link_state_t;

static bool pilot_link_accept(pilot_link_state_t *link, const uint8_t *bytes,
                              size_t length, uint64_t received_us) {
    qcopter_command_frame_t frame;
    if (link == NULL || !qcopter_decode_frame(bytes, length, &frame)) return false;
    if (link->received
        && !qcopter_sequence_is_newer(frame.sequence, link->last_sequence))
        return false;
    link->latest = (pilot_command_t){
        .roll_deg = (float)frame.roll * 35.0f / QCOPTER_AXIS_SCALE,
        .pitch_deg = (float)frame.pitch * 35.0f / QCOPTER_AXIS_SCALE,
        .yaw_rate_dps = (float)frame.yaw_rate * 180.0f / QCOPTER_AXIS_SCALE,
        .throttle = (float)frame.throttle / QCOPTER_THROTTLE_SCALE,
        .valid = true,
        .arm_request = false /* no arming gesture is defined or enabled yet */
    };
    link->last_sequence = frame.sequence;
    link->last_received_us = received_us;
    link->received = true;
    return true;
}

static pilot_command_t pilot_link_current(const pilot_link_state_t *link,
                                          uint64_t now_us) {
    pilot_command_t command = {0};
    if (link != NULL && link->received && now_us >= link->last_received_us
        && now_us - link->last_received_us <= FC_RADIO_TIMEOUT_US)
        command = link->latest;
    else
        command.valid = false;
    return command;
}

static angle_kf_t roll_estimator = {
    .p00 = 1.0f, .p11 = 0.01f,
    .gyro_noise = 0.04f, .bias_walk = 0.0001f, .accel_noise = 4.0f
};
static angle_kf_t pitch_estimator = {
    .p00 = 1.0f, .p11 = 0.01f,
    .gyro_noise = 0.04f, .bias_walk = 0.0001f, .accel_noise = 4.0f
};
static altitude_kf_t altitude_estimator = {
    .p00 = 1.0f, .p11 = 1.0f,
    .accel_noise = 1.5f, .baro_noise = 2.0f
};

/* Gains are placeholders for simulation/bench tuning, not flight values. */
static pid_t roll_angle_pid = {
    .kp = 4.0f, .ki = 0.0f, .kd = 0.0f,
    .integrator_limit = 0.0f, .output_limit = 180.0f
};
static pid_t pitch_angle_pid = {
    .kp = 4.0f, .ki = 0.0f, .kd = 0.0f,
    .integrator_limit = 0.0f, .output_limit = 180.0f
};
static pid_t roll_rate_pid = {
    .kp = 0.006f, .ki = 0.001f, .kd = 0.0001f,
    .integrator_limit = 0.25f, .output_limit = 0.35f
};
static pid_t pitch_rate_pid = {
    .kp = 0.006f, .ki = 0.001f, .kd = 0.0001f,
    .integrator_limit = 0.25f, .output_limit = 0.35f
};
static pid_t yaw_rate_pid = {
    .kp = 0.006f, .ki = 0.001f, .kd = 0.0001f,
    .integrator_limit = 0.25f, .output_limit = 0.35f
};
static pid_t vertical_rate_pid = {
    .kp = 0.08f, .ki = 0.0f, .kd = 0.0f,
    .integrator_limit = 0.0f, .output_limit = 0.2f
};
static float altitude_hold_reference_m;
static const float altitude_position_to_rate = 0.6f; /* placeholder */
static bool altitude_reference_initialized;

static float clampf(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static void pid_reset(pid_t *pid) {
    pid->integrator = 0.0f;
    pid->previous_measurement = 0.0f;
    pid->initialized = false;
}

static void reset_control_state(void) {
    pid_reset(&roll_angle_pid);
    pid_reset(&pitch_angle_pid);
    pid_reset(&roll_rate_pid);
    pid_reset(&pitch_rate_pid);
    pid_reset(&yaw_rate_pid);
    pid_reset(&vertical_rate_pid);
    altitude_reference_initialized = false;
}

/* Derivative is taken on the measurement to avoid setpoint kick. */
static float pid_update(pid_t *pid, float setpoint, float measurement, float dt) {
    if (!(dt > 0.0f) || dt > FC_MAX_CONTROL_DT) {
        pid_reset(pid);
        return 0.0f;
    }
    const float error = setpoint - measurement;
    const float derivative = pid->initialized
        ? (measurement - pid->previous_measurement) / dt : 0.0f;
    const float candidate_i = clampf(pid->integrator + error * dt,
                                     -pid->integrator_limit,
                                     pid->integrator_limit);
    const float raw = pid->kp * error + pid->ki * candidate_i
                    - pid->kd * derivative;
    const float output = clampf(raw, -pid->output_limit, pid->output_limit);
    /* Conditional integration prevents windup when saturation is driven farther. */
    if (raw == output || (raw > output && error < 0.0f)
                      || (raw < output && error > 0.0f)) {
        pid->integrator = candidate_i;
    }
    pid->previous_measurement = measurement;
    pid->initialized = true;
    return output;
}

static void angle_kf_update(angle_kf_t *kf, float gyro_dps,
                            float accel_angle_deg, float dt) {
    const float corrected_rate = gyro_dps - kf->gyro_bias_dps;
    kf->angle_deg += dt * corrected_rate;

    const float p00 = kf->p00;
    const float p01 = kf->p01;
    const float p10 = kf->p10;
    const float p11 = kf->p11;
    kf->p00 = p00 + dt * (dt * p11 - p01 - p10 + kf->gyro_noise);
    kf->p01 = p01 - dt * p11;
    kf->p10 = p10 - dt * p11;
    kf->p11 = p11 + kf->bias_walk * dt;

    const float innovation_variance = kf->p00 + kf->accel_noise;
    const float k0 = kf->p00 / innovation_variance;
    const float k1 = kf->p10 / innovation_variance;
    const float innovation = accel_angle_deg - kf->angle_deg;
    kf->angle_deg += k0 * innovation;
    kf->gyro_bias_dps += k1 * innovation;

    const float updated_p00 = kf->p00;
    const float updated_p01 = kf->p01;
    kf->p00 -= k0 * updated_p00;
    kf->p01 -= k0 * updated_p01;
    kf->p10 -= k1 * updated_p00;
    kf->p11 -= k1 * updated_p01;
}

/* 2-state constant-acceleration filter: state = [height, vertical speed]. */
static void altitude_kf_predict(altitude_kf_t *kf, float accel_up_mps2,
                                float dt) {
    kf->height_m += kf->velocity_mps * dt
                  + 0.5f * accel_up_mps2 * dt * dt;
    kf->velocity_mps += accel_up_mps2 * dt;

    const float p00 = kf->p00;
    const float p01 = kf->p01;
    const float p10 = kf->p10;
    const float p11 = kf->p11;
    const float q = kf->accel_noise * kf->accel_noise;
    kf->p00 = p00 + dt * (p01 + p10) + dt * dt * p11
            + 0.25f * dt * dt * dt * dt * q;
    kf->p01 = p01 + dt * p11 + 0.5f * dt * dt * dt * q;
    kf->p10 = p10 + dt * p11 + 0.5f * dt * dt * dt * q;
    kf->p11 = p11 + dt * dt * q;
}

static void altitude_kf_baro_update(altitude_kf_t *kf, float height_m) {
    const float innovation_variance = kf->p00 + kf->baro_noise;
    const float k0 = kf->p00 / innovation_variance;
    const float k1 = kf->p10 / innovation_variance;
    const float innovation = height_m - kf->height_m;
    const float p00 = kf->p00;
    const float p01 = kf->p01;
    kf->height_m += k0 * innovation;
    kf->velocity_mps += k1 * innovation;
    kf->p00 -= k0 * p00;
    kf->p01 -= k0 * p01;
    kf->p10 -= k1 * p00;
    kf->p11 -= k1 * p01;
}

static bool sample_is_healthy(const sensor_sample_t *s) {
    return s->imu_valid
        && isfinite(s->accel_x_g) && isfinite(s->accel_y_g)
        && isfinite(s->accel_z_g) && isfinite(s->gyro_x_dps)
        && isfinite(s->gyro_y_dps) && isfinite(s->gyro_z_dps);
}

/*
 * This function is currently a math/control core only. Mixer motor order and
 * signs must be validated against the chosen frame before its outputs are used.
 */
static control_output_t flight_control_step(const sensor_sample_t *s,
                                             const pilot_command_t *cmd,
                                             float dt) {
    control_output_t out = { .motor = {0.0f, 0.0f, 0.0f, 0.0f},
                             .armed = false, .sensor_fault = false };
    if (!sample_is_healthy(s) || !isfinite(dt) || dt <= 0.0f
        || dt > FC_MAX_CONTROL_DT) {
        reset_control_state();
        out.sensor_fault = true;
        return out;
    }

    const float accel_roll_deg = atan2f(s->accel_y_g, s->accel_z_g)
                               * FC_RAD_TO_DEG;
    const float accel_pitch_deg = atan2f(-s->accel_x_g,
        sqrtf(s->accel_y_g * s->accel_y_g + s->accel_z_g * s->accel_z_g))
        * FC_RAD_TO_DEG;
    angle_kf_update(&roll_estimator, s->gyro_x_dps, accel_roll_deg, dt);
    angle_kf_update(&pitch_estimator, s->gyro_y_dps, accel_pitch_deg, dt);

    if (s->barometer_valid && isfinite(s->barometer_height_m)
        && isfinite(s->vertical_acceleration_mps2)) {
        altitude_kf_predict(&altitude_estimator,
                            s->vertical_acceleration_mps2, dt);
        altitude_kf_baro_update(&altitude_estimator, s->barometer_height_m);
    }

    if (!cmd->valid || !isfinite(cmd->roll_deg) || !isfinite(cmd->pitch_deg)
        || !isfinite(cmd->yaw_rate_dps) || !isfinite(cmd->throttle)) {
        reset_control_state();
        out.sensor_fault = true;
        return out;
    }
    /* The only implemented mode requires valid height/rate aiding. */
    if (!s->barometer_valid || !isfinite(s->barometer_height_m)
        || !isfinite(s->vertical_acceleration_mps2)) {
        reset_control_state();
        out.sensor_fault = true;
        return out;
    }

    /* Bound pilot inputs before they enter the control math. */
    const float roll_setpoint = clampf(cmd->roll_deg, -35.0f, 35.0f);
    const float pitch_setpoint = clampf(cmd->pitch_deg, -35.0f, 35.0f);
    const float yaw_rate_setpoint = clampf(cmd->yaw_rate_dps, -180.0f, 180.0f);
    const float throttle = clampf(cmd->throttle, 0.0f, 1.0f);
    const float roll_rate_setpoint = pid_update(&roll_angle_pid, roll_setpoint,
                                                 roll_estimator.angle_deg, dt);
    const float pitch_rate_setpoint = pid_update(&pitch_angle_pid, pitch_setpoint,
                                                  pitch_estimator.angle_deg, dt);
    const float roll = pid_update(&roll_rate_pid, roll_rate_setpoint,
                                  s->gyro_x_dps, dt);
    const float pitch = pid_update(&pitch_rate_pid, pitch_rate_setpoint,
                                   s->gyro_y_dps, dt);
    const float yaw = pid_update(&yaw_rate_pid, yaw_rate_setpoint,
                                 s->gyro_z_dps, dt);

    if (!altitude_reference_initialized) {
        altitude_hold_reference_m = altitude_estimator.height_m;
        pid_reset(&vertical_rate_pid);
        altitude_reference_initialized = true;
    }
    const float pilot_vertical_speed = (throttle - 0.5f)
                                     * 2.0f * FC_MAX_VERTICAL_SPEED_MPS;
    altitude_hold_reference_m += pilot_vertical_speed * dt;
    const float altitude_velocity_correction = clampf(
        altitude_position_to_rate
            * (altitude_hold_reference_m - altitude_estimator.height_m),
        -FC_MAX_VERTICAL_SPEED_MPS, FC_MAX_VERTICAL_SPEED_MPS);
    const float vertical_speed_setpoint = clampf(
        pilot_vertical_speed + altitude_velocity_correction,
        -FC_MAX_VERTICAL_SPEED_MPS, FC_MAX_VERTICAL_SPEED_MPS);
    const float collective_correction = pid_update(&vertical_rate_pid,
        vertical_speed_setpoint, altitude_estimator.velocity_mps, dt);
    const float collective = clampf(FC_HOVER_THROTTLE_PLACEHOLDER
                                    + collective_correction, 0.0f, 1.0f);

    /* Provisional X-frame math in logical order FL, FR, RR, RL. The result is
       for simulation only until motor order, spin directions, and frame axes
       are verified. No PWM/DShot/ESC driver consumes these values. */
    out.motor[0] = clampf(collective + roll + pitch - yaw, 0.0f, 1.0f);
    out.motor[1] = clampf(collective - roll + pitch + yaw, 0.0f, 1.0f);
    out.motor[2] = clampf(collective - roll - pitch - yaw, 0.0f, 1.0f);
    out.motor[3] = clampf(collective + roll - pitch + yaw, 0.0f, 1.0f);

    /* There is deliberately no hardware arming/output path in this scaffold. */
    out.armed = false;
    return out;
}

#ifndef QCOPTER_HOST_TEST
typedef struct {
    uint8_t bytes[QCOPTER_FRAME_SIZE];
    size_t length;
    uint64_t received_us;
} radio_rx_item_t;

static queue_t radio_rx_queue;

/* Nonblocking placeholder: connect this to the selected LoRa module driver. */
static bool radio_receive_frame(uint8_t *bytes, size_t capacity, size_t *length) {
    (void)bytes;
    (void)capacity;
    (void)length;
    return false;
}

static void slow_core(void) {
    for (;;) {
        /*
         * Core 1 owns nonblocking radio and slower sensor acquisition.
         * The radio stub returns no data until a module driver is selected.
         */
        radio_rx_item_t item = {0};
        if (radio_receive_frame(item.bytes, sizeof(item.bytes), &item.length)
            && item.length <= sizeof(item.bytes)) {
            item.received_us = time_us_64();
            (void)queue_try_add(&radio_rx_queue, &item);
        }
        sleep_ms(1u);
    }
}
#endif

#ifndef QCOPTER_HOST_TEST
int main(void) {
    stdio_init_all();
    queue_init(&radio_rx_queue, sizeof(radio_rx_item_t), 4u);
    multicore_launch_core1(slow_core);
    sleep_ms(1500);
    puts("QCopter RP2350 FC: 500 Hz loop, radio/sensors/ESC outputs not connected.");
    puts("Core 1 radio receive is a stub; outputs stay disarmed.");

    pilot_link_state_t pilot_link = {0};
    uint64_t previous_us = time_us_64();
    absolute_time_t next_cycle = delayed_by_us(get_absolute_time(),
                                                FC_LOOP_PERIOD_US);
    uint32_t loop_count = 0u;
    uint32_t missed_deadlines = 0u;
    for (;;) {
        sleep_until(next_cycle);
        next_cycle = delayed_by_us(next_cycle, FC_LOOP_PERIOD_US);
        const uint64_t now_us = time_us_64();
        const uint64_t scheduled_us = to_us_since_boot(next_cycle)
                                    - FC_LOOP_PERIOD_US;
        if (now_us > scheduled_us + 100u) missed_deadlines++;
        const float dt = (float)(now_us - previous_us) * 1e-6f;
        previous_us = now_us;

        radio_rx_item_t item;
        while (queue_try_remove(&radio_rx_queue, &item))
            (void)pilot_link_accept(&pilot_link, item.bytes, item.length,
                                    item.received_us);

        /* Sensor acquisition is not implemented: safe zero outputs by design. */
        const sensor_sample_t no_sensors = {0};
        const pilot_command_t command = pilot_link_current(&pilot_link, now_us);
        const control_output_t status = flight_control_step(&no_sensors,
                                                             &command, dt);
        (void)status;
        if (++loop_count == FC_LOOP_HZ) {
            printf("fc_loop=500Hz target; missed=%lu; command=%s; motors=DISABLED\n",
                   (unsigned long)missed_deadlines,
                   command.valid ? "fresh" : "failsafe");
            loop_count = 0u;
            missed_deadlines = 0u;
        }
    }
}
#endif
