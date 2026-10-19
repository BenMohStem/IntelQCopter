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

#ifndef QCOPTER_HOST_TEST
#include "pico/stdlib.h"
#include "pico/multicore.h"
#endif

#define FC_LOOP_HZ 500.0f
#define FC_LOOP_DT (1.0f / FC_LOOP_HZ)
#define FC_MAX_CONTROL_DT 0.010f
#define FC_RAD_TO_DEG 57.2957795f
#define FC_DEG_TO_RAD 0.0174532925f

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
    float throttle;
    bool valid;
    bool arm_request;
    bool altitude_hold;
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
static pid_t altitude_pid = {
    .kp = 0.5f, .ki = 0.0f, .kd = 0.0f,
    .integrator_limit = 0.0f, .output_limit = 0.2f
};
static float altitude_hold_reference_m;
static bool altitude_hold_was_enabled;

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

    float collective = throttle;
    if (cmd->altitude_hold && s->barometer_valid
        && isfinite(s->barometer_height_m)
        && isfinite(s->vertical_acceleration_mps2)) {
        if (!altitude_hold_was_enabled) {
            altitude_hold_reference_m = altitude_estimator.height_m;
            pid_reset(&altitude_pid);
        }
        altitude_hold_was_enabled = true;
        const float altitude_correction = pid_update(&altitude_pid,
            altitude_hold_reference_m, altitude_estimator.height_m, dt);
        collective = clampf(throttle + altitude_correction, 0.0f, 1.0f);
    } else {
        altitude_hold_was_enabled = false;
        pid_reset(&altitude_pid);
    }

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
static void slow_core(void) {
    for (;;) {
        /* Reserved for nonblocking radio, barometer, magnetometer, and telemetry. */
        sleep_ms(10);
    }
}
#endif

#ifndef QCOPTER_HOST_TEST
int main(void) {
    stdio_init_all();
    multicore_launch_core1(slow_core);
    sleep_ms(1500);
    puts("QCopter FC safe scaffold: outputs disabled; no sensors or ESCs configured.");
    puts("Control and estimator core is not armed or connected to motor pins.");

    const sensor_sample_t no_sensors = {0};
    const pilot_command_t no_command = {0};
    for (;;) {
        const control_output_t status =
            flight_control_step(&no_sensors, &no_command, FC_LOOP_DT);
        (void)status;
        sleep_ms(1000);
    }
}
#endif
