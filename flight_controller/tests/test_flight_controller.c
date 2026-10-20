/* Host tests include the production C implementation without changing its math. */
#define QCOPTER_HOST_TEST
#include "../flight_controller.c"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        exit(1); \
    } \
} while (0)

static sensor_sample_t level_sensor(void) {
    sensor_sample_t s = {
        .accel_x_g = 0.0f, .accel_y_g = 0.0f, .accel_z_g = 1.0f,
        .gyro_x_dps = 0.0f, .gyro_y_dps = 0.0f, .gyro_z_dps = 0.0f,
        .vertical_acceleration_mps2 = 0.0f,
        .barometer_height_m = 0.0f,
        .imu_valid = true, .imu_calibrated = true, .barometer_valid = true, .barometer_updated = true
    };
    return s;
}

static pilot_command_t neutral_command(void) {
    pilot_command_t c = {
        .roll_deg = 0.0f, .pitch_deg = 0.0f, .yaw_rate_dps = 0.0f,
        .throttle = 0.5f, .valid = true, .arm_request = false
    };
    return c;
}

static void test_radio_frame_codec_sequence_and_watchdog(void) {
    qcopter_command_frame_t input = {
        .sequence = 65535u, .roll = 5000, .pitch = -10000,
        .yaw_rate = -10000, .throttle = 5000u, .lt = 0u, .rt = 1000u,
        .buttons = QCOPTER_BUTTON_RB | QCOPTER_BUTTON_NORTH
    };
    uint8_t bytes[QCOPTER_FRAME_SIZE];
    qcopter_command_frame_t decoded = {0};
    CHECK(qcopter_encode_frame(&input, bytes));
    CHECK(qcopter_decode_frame(bytes, sizeof(bytes), &decoded));
    CHECK(decoded.sequence == input.sequence && decoded.roll == input.roll
          && decoded.pitch == input.pitch && decoded.yaw_rate == input.yaw_rate
          && decoded.throttle == input.throttle && decoded.buttons == input.buttons);

    pilot_link_state_t link = {0};
    CHECK(pilot_link_accept(&link, bytes, sizeof(bytes), 100000u));
    pilot_command_t command = pilot_link_current(&link, 250000u);
    CHECK(command.valid && fabsf(command.roll_deg - 17.5f) < 1e-5f);
    CHECK(fabsf(command.pitch_deg + 35.0f) < 1e-5f);
    CHECK(fabsf(command.yaw_rate_dps + 180.0f) < 1e-5f);
    CHECK(fabsf(command.throttle - 0.5f) < 1e-5f);
    CHECK(fabsf(command.boost-1.0f)<1e-6f);
    CHECK(!command.arm_request);
    CHECK(!pilot_link_current(&link, 250001u).valid);
    CHECK(!pilot_link_accept(&link, bytes, sizeof(bytes), 260000u));

    input.sequence = 0u; /* valid 16-bit sequence wrap */
    CHECK(qcopter_encode_frame(&input, bytes));
    CHECK(pilot_link_accept(&link, bytes, sizeof(bytes), 270000u));
    bytes[9] ^= 0x40u;
    CHECK(!qcopter_decode_frame(bytes, sizeof(bytes), &decoded));
    input.buttons |= 0x8000u;
    CHECK(!qcopter_encode_frame(&input, bytes));
    CHECK(qcopter_sequence_is_newer(0u, 65535u));
}

static void test_centered_throttle_holds_and_stick_commands_vertical_rate(void) {
    sensor_sample_t s = level_sensor();
    pilot_command_t c = neutral_command();
    for (unsigned i = 0; i < 20; ++i)
        (void)flight_control_step(&s, &c, FC_LOOP_DT);
    const float neutral_collective = flight_control_step(&s, &c, FC_LOOP_DT).motor[0];
    CHECK(fabsf(neutral_collective - FC_HOVER_THROTTLE_PLACEHOLDER) < 1e-5f);

    c.throttle = 0.75f;
    control_output_t climb = {0};
    for (unsigned i = 0; i < 20; ++i) climb = flight_control_step(&s, &c, FC_LOOP_DT);
    CHECK(climb.motor[0] > neutral_collective);

    c.throttle = 0.25f;
    control_output_t descend = {0};
    for (unsigned i = 0; i < 20; ++i) descend = flight_control_step(&s, &c, FC_LOOP_DT);
    CHECK(descend.motor[0] < neutral_collective);
    CHECK(!climb.armed && !descend.armed);
}

static void test_neutral_output_stays_disarmed(void) {
    sensor_sample_t s = level_sensor();
    pilot_command_t c = neutral_command();
    const control_output_t out = flight_control_step(&s, &c, FC_LOOP_DT);
    CHECK(!out.sensor_fault);
    CHECK(!out.armed);
    for (unsigned i = 0; i < 4; ++i) {
        CHECK(out.motor[i] >= 0.0f && out.motor[i] <= 1.0f);
        CHECK(fabsf(out.motor[i] - c.throttle) < 1e-5f);
    }
}

static void test_bad_sensor_dt_and_command_fail_closed(void) {
    sensor_sample_t s = level_sensor();
    pilot_command_t c = neutral_command();
    s.imu_valid = false;
    altitude_reference_initialized = true;
    roll_rate_pid.integrator = 0.1f;
    control_output_t out = flight_control_step(&s, &c, FC_LOOP_DT);
    CHECK(out.sensor_fault && !out.armed);
    CHECK(!altitude_reference_initialized && roll_rate_pid.integrator == 0.0f);
    for (unsigned i = 0; i < 4; ++i) CHECK(out.motor[i] == 0.0f);

    s = level_sensor();
    out = flight_control_step(&s, &c, 0.02f);
    CHECK(out.sensor_fault && !out.armed);

    s = level_sensor();
    s.barometer_valid = false;
    out = flight_control_step(&s, &c, FC_LOOP_DT);
    CHECK(out.sensor_fault && !out.armed);
    for (unsigned i = 0; i < 4; ++i) CHECK(out.motor[i] == 0.0f);

    s = level_sensor();
    c.valid = false;
    out = flight_control_step(&s, &c, FC_LOOP_DT);
    CHECK(out.sensor_fault && !out.armed);
}

static void test_pid_limits_and_anti_windup(void) {
    pid_t p = { .kp = 1.0f, .ki = 1.0f,
                .integrator_limit = 1.0f, .output_limit = 0.2f };
    for (unsigned i = 0; i < 100; ++i) {
        const float out = pid_update(&p, 10.0f, 0.0f, FC_LOOP_DT);
        CHECK(out <= 0.2f && out >= -0.2f);
    }
    CHECK(p.integrator == 0.0f);

    const float rejected = pid_update(&p, 1.0f, 0.0f, 0.0f);
    CHECK(rejected == 0.0f && !p.initialized);
}

static void test_estimator_and_mixer_remain_finite(void) {
    sensor_sample_t s = level_sensor();
    pilot_command_t c = neutral_command();
    s.accel_y_g = 0.5f;
    s.accel_z_g = 0.8660254f;
    c.roll_deg = 30.0f;
    c.pitch_deg = -30.0f;
    c.yaw_rate_dps = 180.0f;
    for (unsigned step = 0; step < 500; ++step) {
        const control_output_t out = flight_control_step(&s, &c, FC_LOOP_DT);
        CHECK(!out.sensor_fault && !out.armed);
        for (unsigned i = 0; i < 4; ++i)
            CHECK(isfinite(out.motor[i]) && out.motor[i] >= 0.0f && out.motor[i] <= 1.0f);
    }
    CHECK(isfinite(roll_estimator.angle_deg));
    CHECK(isfinite(pitch_estimator.angle_deg));
    CHECK(isfinite(altitude_estimator.height_m));
}

static void test_battery_monitor_filter_soc_and_actions(void) {
    (void)battery_policy_reset(true);
    battery_status_t status;
    battery_monitor_init(&status, 100.0f);
    const battery_config_t cfg = {
        .capacity_ah = 4.2f, .initial_soc_percent = 100.0f,
        .low_cell_voltage_v = 3.6f, .critical_cell_voltage_v = 3.3f,
        .low_pack_voltage_v = 10.8f, .maximum_discharge_current_a = 80.0f,
        .voltage_measurement_variance_v2 = 0.04f,
        .voltage_process_variance_v2 = 0.001f,
        .initial_soc_variance_percent2 = 25.0f,
        .soc_process_variance_percent2_per_s = 0.01f,
        .soc_measurement_variance_percent2 = 4.0f,
        .maximum_sample_dt_s = 0.1f
    };
    battery_sample_t sample = {
        .pack_voltage_v = 12.0f, .pack_current_a = 4.2f,
        .minimum_cell_voltage_v = 4.0f, .pack_voltage_valid = true,
        .pack_current_valid = true, .cell_voltage_valid = true
    };
    CHECK(battery_monitor_update(&status, &sample, &cfg, 0.1f)
          == BATTERY_ACTION_NORMAL);
    CHECK(status.voltage_filter_initialized && status.soc_initialized);
    CHECK(status.filtered_pack_voltage_v == sample.pack_voltage_v);
    CHECK(status.estimated_soc_percent < 100.0f);
    const float sigma_before_correction = status.estimated_soc_sigma_percent;
    sample.rested_soc_observation_valid = true;
    sample.rested_soc_observation_percent = 80.0f;
    battery_monitor_update(&status, &sample, &cfg, 0.1f);
    CHECK(status.estimated_soc_percent < 90.0f);
    CHECK(status.estimated_soc_sigma_percent < sigma_before_correction);
    /* User-selected policy now disarms at 20%, regardless of landed state. */
    status.estimated_soc_percent = 20.0f;
    sample.rested_soc_observation_valid = false;
    CHECK(battery_monitor_update(&status, &sample, &cfg, 0.1f)
          == BATTERY_ACTION_DISARM);
    sample.aircraft_landed = true;
    CHECK(battery_monitor_update(&status, &sample, &cfg, 0.1f)
          == BATTERY_ACTION_DISARM);
    sample.aircraft_landed = false;
    sample.rested_soc_observation_valid = false;
    sample.pack_voltage_v = 10.0f;
    for (unsigned i = 0; i < 20; ++i)
        battery_monitor_update(&status, &sample, &cfg, 0.1f);
    CHECK(status.action == BATTERY_ACTION_DISARM);
    sample.pack_voltage_v = 12.0f;
    sample.minimum_cell_voltage_v = 3.2f;
    CHECK(battery_monitor_update(&status, &sample, &cfg, 0.1f)
          == BATTERY_ACTION_DISARM);
    sample.aircraft_landed = true;
    CHECK(battery_monitor_update(&status, &sample, &cfg, 0.1f)
          == BATTERY_ACTION_DISARM);
    sample.aircraft_landed = false;
    sample.minimum_cell_voltage_v = 4.0f;
    sample.bms_alert = true;
    CHECK(battery_monitor_update(&status, &sample, &cfg, 0.1f)
          == BATTERY_ACTION_DISARM);
    sample.bms_alert = false;
    sample.pack_current_valid = false;
    CHECK(battery_monitor_update(&status, &sample, &cfg, 0.1f)
          == BATTERY_ACTION_SENSOR_FAULT);
    (void)battery_policy_reset(true);
}

static void test_imu_calibration_known_bias_gain_and_motion_rejection(void) {
    imu_calibration_t calibration = {0};
    cal_stats3_t faces[6] = {0};
    const float bias[3] = {0.03f, -0.02f, 0.04f};
    const float gain[3] = {0.98f, 1.03f, 0.97f};
    const float gyro_bias[3] = {0.7f, -0.4f, 0.12f};
    for (unsigned face = 0; face < 6u; ++face) {
        for (unsigned n = 0; n < 100u; ++n) {
            float raw[3];
            const float noise = n % 2u ? 0.001f : -0.001f;
            for (unsigned axis = 0; axis < 3u; ++axis) {
                const float truth = axis == face/2u ? (face%2u ? -1.0f : 1.0f) : 0.0f;
                raw[axis] = truth/gain[axis] + bias[axis] + noise;
            }
            CHECK(imu_accel_face_add(faces, face, raw, true));
        }
    }
    CHECK(!imu_accel_cal_finish(faces, &calibration, false));
    CHECK(imu_accel_cal_finish(faces, &calibration, true));
    for (unsigned axis = 0; axis < 3u; ++axis) {
        CHECK(fabsf(calibration.accel_bias_g[axis] - bias[axis]) < 1e-6f);
        CHECK(fabsf(calibration.accel_gain[axis] - gain[axis]) < 1e-6f);
    }
    imu_stationary_cal_t stationary = {0};
    const float accel[3] = {bias[0], bias[1], 1.0f/gain[2] + bias[2]};
    CHECK(!imu_stationary_cal_add(&stationary, accel, gyro_bias, false));
    for (unsigned n = 0; n < 500u; ++n) {
        const float noise = n%2u ? 0.02f : -0.02f;
        const float gyro[3] = {gyro_bias[0]+noise, gyro_bias[1]+noise, gyro_bias[2]+noise};
        CHECK(imu_stationary_cal_add(&stationary, accel, gyro, true));
    }
    CHECK(imu_gyro_cal_finish(&stationary, &calibration, true));
    float corrected_accel[3], corrected_gyro[3];
    CHECK(imu_cal_apply(&calibration, accel, gyro_bias, corrected_accel, corrected_gyro));
    CHECK(fabsf(corrected_accel[2] - 1.0f) < 1e-6f);
    CHECK(fabsf(corrected_accel[0]) < 1e-6f && fabsf(corrected_accel[1]) < 1e-6f);
    for (unsigned axis = 0; axis < 3u; ++axis) {
        CHECK(fabsf(corrected_gyro[axis]) < 1e-6f);
        CHECK(calibration.gyro_variance_dps2[axis] > 0.0003f);
    }
    imu_stationary_cal_t moving = {0};
    for (unsigned n = 0; n < 500u; ++n) {
        const float gyro[3] = {n%2u ? 1.0f : -1.0f, 0.0f, 0.0f};
        CHECK(imu_stationary_cal_add(&moving, accel, gyro, true));
    }
    CHECK(!imu_gyro_cal_finish(&moving, &calibration, true));
    cal_stats3_t bad_faces[6] = {0};
    CHECK(!imu_accel_cal_finish(bad_faces, &calibration, true));
    sensor_sample_t sample = level_sensor();
    sample.imu_calibrated = false;
    sample.accel_x_g = accel[0]; sample.accel_y_g = accel[1]; sample.accel_z_g = accel[2];
    sample.gyro_x_dps = gyro_bias[0]; sample.gyro_y_dps = gyro_bias[1]; sample.gyro_z_dps = gyro_bias[2];
    CHECK(!sample_is_healthy(&sample));
    CHECK(sensor_sample_apply_imu_calibration(&sample, &calibration));
    CHECK(sample_is_healthy(&sample));
    CHECK(!sensor_sample_apply_imu_calibration(&sample, &calibration));
    printf("IMU calibration: injected accel bias/gain and gyro bias recovered; moving capture rejected. gyro sigma=%.6f deg/s\n",
           sqrtf(calibration.gyro_variance_dps2[0]));
}

static void test_magnetometer_3d_fit_and_planar_rejection(void) {
    mag_sweep_cal_t sweep = {0}, planar = {0};
    mag_calibration_t calibration = {0};
    for (unsigned lat = 0; lat <= 20u; ++lat) {
        const float phi = -1.57079632679f + 3.14159265359f*(float)lat/20.0f;
        for (unsigned lon = 0; lon < 24u; ++lon) {
            const float theta = 6.28318530718f*(float)lon/24.0f;
            const float field[3] = {8.0f + 48.0f*cosf(phi)*cosf(theta),
                                   -12.0f + 32.0f*cosf(phi)*sinf(theta),
                                   4.0f + 44.0f*sinf(phi)};
            CHECK(mag_cal_add(&sweep, field, true));
        }
    }
    CHECK(!mag_cal_finish(&sweep, &calibration, false, true));
    CHECK(mag_cal_finish(&sweep, &calibration, true, true));
    CHECK(fabsf(calibration.offset_ut[0] - 8.0f) < 0.001f);
    CHECK(fabsf(calibration.offset_ut[1] + 12.0f) < 0.001f);
    CHECK(fabsf(calibration.offset_ut[2] - 4.0f) < 0.001f);
    for (unsigned n = 0; n < 400u; ++n) {
        const float angle = 6.28318530718f*(float)n/400.0f;
        const float x = 40.0f*cosf(angle), y = 40.0f*sinf(angle);
        const float field[3] = {x + 8.0f, y - 12.0f, x*0.5f+y*0.3f+4.0f};
        CHECK(mag_cal_add(&planar, field, true));
    }
    CHECK(!mag_cal_finish(&planar, &calibration, true, true));
    const float raw[3] = {56.0f, -12.0f, 4.0f};
    float corrected[3];
    CHECK(mag_cal_apply(&calibration, raw, corrected));
    CHECK(fabsf(corrected[0] - 41.333333f) < 0.001f);
    printf("Magnetometer: known offsets recovered from 504 sphere samples; tilted planar sweep rejected.\n");
}

static void test_bmp280_factory_compensation_and_ground_reference(void) {
    const uint16_t coefficients[12] = {27504u, (uint16_t)26435, (uint16_t)-1000,
        36477u, (uint16_t)-10685, 3024u, 2855u, 140u, (uint16_t)-7,
        15500u, (uint16_t)-14600, 6000u};
    uint8_t bytes[24];
    for (unsigned i = 0; i < 12u; ++i) qcopter_put_u16(bytes + 2u*i, coefficients[i]);
    bmp280_trim_t trim;
    CHECK(bmp280_trim_decode(bytes, sizeof(bytes), &trim));
    double temperature, pressure;
    CHECK(bmp280_compensate(&trim, 519888u, 415148u, &temperature, &pressure));
    CHECK(fabs(temperature - 25.08247793) < 0.00001);
    CHECK(fabs(pressure - 100653.26678) < 0.01);
    CHECK(!bmp280_compensate(&trim, 0x80000u, 415148u, &temperature, &pressure));
    baro_reference_cal_t reference = {0};
    baro_calibration_t calibration = {0};
    CHECK(!baro_reference_add(&reference, 100000.0, false));
    for (unsigned i = 0; i < 100u; ++i)
        CHECK(baro_reference_add(&reference, 100000.0 + (i%2u ? 1.0 : -1.0), true));
    CHECK(!baro_reference_finish(&reference, &calibration, 0.1, true));
    CHECK(baro_reference_finish(&reference, &calibration, 2.0, true));
    double height, sigma;
    const double pressure_at_10m = 100000.0*pow(1.0-10.0/44330.0, 1.0/0.190294957);
    CHECK(baro_relative_height(&calibration, pressure_at_10m, &height, &sigma));
    CHECK(fabs(height - 10.0) < 1e-7 && sigma > 0.0 && sigma < 0.2);
    printf("BMP280 reference: T=%.6f C P=%.6f Pa; synthetic ground-reference height=%.6f m sigma=%.6f m\n",
           temperature, pressure, height, sigma);
}

static void test_lidar_voltage_current_reference_regression(void) {
    linear_cal_fit_t fit = {0}, no_span = {0};
    linear_cal_t calibration = {0};
    for (unsigned i = 0; i < 10u; ++i) {
        const double x = (double)i/3.0;
        const double reference = 4.0*x + 0.02 + (i%2u ? 0.001 : -0.001);
        CHECK(linear_cal_add(&fit, x, reference, true));
        CHECK(linear_cal_add(&no_span, 2.0, reference, true));
    }
    CHECK(!linear_cal_finish(&no_span, &calibration, 0.01, true));
    CHECK(!linear_cal_finish(&fit, &calibration, 0.0001, true));
    CHECK(linear_cal_finish(&fit, &calibration, 0.01, true));
    double corrected;
    CHECK(linear_cal_apply(&calibration, 2.0, &corrected));
    CHECK(fabs(corrected - 8.02) < 0.001);
    CHECK(!linear_cal_add(&fit, NAN, 1.0, true));
    printf("Reference regression: gain=%.8f offset=%.8f residual sigma=%.8f; zero-span and noisy fits rejected.\n",
           calibration.gain, calibration.offset, calibration.residual_sigma);
}

/* Deterministic 5-second sensor-input trace for plots; this is not a plant model. */
static int write_trace(const char *path) {
    FILE *file = fopen(path, "w");
    if (file == NULL) {
        perror("trace output");
        return 2;
    }
    fprintf(file, "time_s,setpoint_roll_deg,accel_roll_deg,estimated_roll_deg,gyro_x_dps,motor_fl,motor_fr,motor_rr,motor_rl,armed,sensor_fault\n");
    sensor_sample_t sensor = level_sensor();
    pilot_command_t command = neutral_command();
    command.throttle = 0.45f;
    const unsigned sample_count = 2500; /* 500 Hz for 5 seconds */
    for (unsigned i = 0; i < sample_count; ++i) {
        const float time_s = (float)i * FC_LOOP_DT;
        command.roll_deg = time_s < 0.5f ? 0.0f : 10.0f;
        float angle_deg = 0.0f;
        sensor.gyro_x_dps = 0.0f;
        if (time_s >= 1.0f && time_s < 1.7f) {
            angle_deg = (time_s - 1.0f) * (8.0f / 0.7f);
            sensor.gyro_x_dps = 8.0f / 0.7f;
        } else if (time_s >= 1.7f) {
            angle_deg = 8.0f;
        }
        const float radians = angle_deg * FC_DEG_TO_RAD;
        sensor.accel_y_g = sinf(radians);
        sensor.accel_z_g = cosf(radians);
        const control_output_t out = flight_control_step(&sensor, &command, FC_LOOP_DT);
        fprintf(file, "%.3f,%.3f,%.3f,%.5f,%.3f,%.5f,%.5f,%.5f,%.5f,%d,%d\n",
                time_s, command.roll_deg, angle_deg, roll_estimator.angle_deg,
                sensor.gyro_x_dps, out.motor[0], out.motor[1], out.motor[2],
                out.motor[3], out.armed ? 1 : 0, out.sensor_fault ? 1 : 0);
    }
    fclose(file);
    printf("Wrote %u deterministic 500 Hz math trace samples to %s (no plant model).\n",
           sample_count, path);
    return 0;
}

static float gaussian_noise(void) {
    const float u1 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 2.0f);
    const float u2 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 2.0f);
    return sqrtf(-2.0f * logf(u1)) * cosf(6.28318530718f * u2);
}

static float scenario_roll_deg(float time_s) {
    if (time_s < 1.0f) return 0.0f;
    if (time_s < 1.7f) return (time_s - 1.0f) * (8.0f / 0.7f);
    return 8.0f;
}

static float scenario_roll_rate_dps(float time_s) {
    return time_s >= 1.0f && time_s < 1.7f ? 8.0f / 0.7f : 0.0f;
}

/* Monte Carlo runs the production angle_kf_update() with documented test noise. */
static int write_kalman_mc(const char *path, unsigned trials) {
    enum { STEPS = 2500, STRIDE = 10, POINTS = STEPS / STRIDE + 1 };
    double mean_error[POINTS] = {0};
    double m2[POINTS] = {0};
    double squared_error[POINTS] = {0};
    unsigned within_1sigma[POINTS] = {0};
    unsigned within_2sigma[POINTS] = {0};
    float predicted_sigma[POINTS] = {0};
    float truth[POINTS] = {0};
    const angle_kf_t initial = {
        .p00 = 1.0f, .p11 = 0.01f,
        .gyro_noise = 0.04f, .bias_walk = 0.0001f, .accel_noise = 4.0f
    };
    FILE *file = fopen(path, "w");
    if (file == NULL) { perror("Kalman MC output"); return 2; }
    if (trials < 2) { fclose(file); fprintf(stderr, "Use at least 2 Monte Carlo trials.\n"); return 2; }
    srand(2350);
    for (unsigned trial = 0; trial < trials; ++trial) {
        roll_estimator = initial;
        const float initial_roll_deg = gaussian_noise() * sqrtf(initial.p00);
        float true_bias_dps = gaussian_noise() * sqrtf(initial.p11);
        for (unsigned step = 0; step <= STEPS; ++step) {
            const float time_s = (float)step * FC_LOOP_DT;
            const float true_angle = initial_roll_deg + scenario_roll_deg(time_s);
            true_bias_dps += gaussian_noise() * sqrtf(initial.bias_walk * FC_LOOP_DT);
            const float gyro = scenario_roll_rate_dps(time_s) + true_bias_dps
                             + gaussian_noise() * sqrtf(initial.gyro_noise);
            const float accel_angle = true_angle
                                    + gaussian_noise() * sqrtf(initial.accel_noise);
            angle_kf_update(&roll_estimator, gyro, accel_angle, FC_LOOP_DT);
            if (step % STRIDE == 0) {
                const unsigned p = step / STRIDE;
                const double estimate = roll_estimator.angle_deg;
                const double error = estimate - true_angle;
                const double delta = error - mean_error[p];
                mean_error[p] += delta / (double)(trial + 1);
                m2[p] += delta * (error - mean_error[p]);
                squared_error[p] += error * error;
                const float sigma = sqrtf(fmaxf(roll_estimator.p00, 0.0f));
                predicted_sigma[p] += sigma / (float)trials;
                truth[p] = scenario_roll_deg(time_s);
                if (fabs(error) <= sigma) within_1sigma[p]++;
                if (fabs(error) <= 2.0 * sigma) within_2sigma[p]++;
            }
        }
    }
    fprintf(file, "time_s,true_roll_deg,mean_error_deg,empirical_error_std_deg,kalman_std_deg,coverage_1sigma,coverage_2sigma,rmse_deg\n");
    for (unsigned p = 0; p < POINTS; ++p) {
        const double variance = m2[p] / (double)(trials - 1);
        fprintf(file, "%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                (float)(p * STRIDE) * FC_LOOP_DT, truth[p], mean_error[p],
                sqrt(variance), predicted_sigma[p],
                (float)within_1sigma[p] / (float)trials,
                (float)within_2sigma[p] / (float)trials,
                sqrt(squared_error[p] / (double)trials));
    }
    fclose(file);
    printf("Kalman Monte Carlo passed: %u trials x %u steps; production C filter; seed=2350.\n",
           trials, STEPS);
    printf("Assumed noise: gyro sigma=%.3f deg/s, accel-angle sigma=%.3f deg, bias walk sigma=%.4f deg/s/sqrt(s).\n",
           sqrtf(initial.gyro_noise), sqrtf(initial.accel_noise), sqrtf(initial.bias_walk));
    printf("Initial priors: roll sigma=%.3f deg, gyro-bias sigma=%.3f deg/s; Gaussian simulated input.\n",
           sqrtf(initial.p00), sqrtf(initial.p11));
    printf("Wrote %u-sample distribution trace to %s.\n", POINTS, path);
    return 0;
}

static void test_matrix_covariance_and_pitch_sign(void) {
    altitude_kf_t k={.p00=2.0f,.p01=.3f,.p10=.3f,.p11=1.0f,
                     .accel_noise=1.5f,.baro_noise=.0025f};
    altitude_kf_predict(&k,2.0f,.02f);
    const double p00=2+2*.02*.3+.02*.02+pow(.02,4)*2.25/4;
    const double p01=.3+.02+pow(.02,3)*2.25/2;
    const double p11=1+.02*.02*2.25;
    CHECK(fabs(k.p00-p00)<1e-6 && fabs(k.p01-p01)<1e-6);
    CHECK(fabs(k.p11-p11)<1e-6 && k.p01==k.p10);
    const double gain=p00/(p00+.0025);
    altitude_kf_baro_update(&k,.1f);
    CHECK(fabs(k.gain_height-gain)<1e-6);
    CHECK(k.p00>0 && k.p11>0 && k.p00*k.p11-k.p01*k.p10>=0);
    // Independent full Joseph product check for H=[1,0].
    float a=2,b=.3f,c=.3f,d=1;
    covariance_correct(&a,&b,&c,&d,.8f,.2f,.1f);
    CHECK(fabsf(a-.144f)<1e-6f && fabsf(b+.004f)<1e-6f);
    CHECK(fabsf(d-.964f)<1e-6f && b==c);
    const float gains[18]={3,0,0,.00118477f,.00789846f,0,3,0,0,
        .00118477f,.00789846f,0,.0076795f,.0170656f,0,.2205f,.09f,.175f};
    qc_sitl_reset(.49f,0,gains);
    sensor_sample_t s=level_sensor();pilot_command_t cmd=neutral_command();cmd.pitch_deg=5;
    const control_output_t o=flight_control_step(&s,&cmd,FC_LOOP_DT);
    CHECK(o.motor[2]+o.motor[3]>o.motor[0]+o.motor[1]);
    CHECK(!o.armed);
    // RT grants performance allowance, but at center stick it still holds height.
    qc_sitl_reset(.49f,0,gains);cmd=neutral_command();cmd.boost=1;
    control_output_t boosted={0};
    for(int i=0;i<125;i++)boosted=flight_control_step(&s,&cmd,FC_LOOP_DT);
    CHECK(fabsf(boost_state-1)<1e-5f && fabsf(boosted.motor[0]-.49f)<1e-5f);
    cmd.throttle=1;
    for(int i=0;i<125;i++)boosted=flight_control_step(&s,&cmd,FC_LOOP_DT);
    CHECK(boosted.motor[0]>.79f && boosted.motor[0]<=.80001f && !boosted.armed);
    qc_sitl_reset(.49f,0,gains);cmd.boost=0;
    for(int i=0;i<125;i++)boosted=flight_control_step(&s,&cmd,FC_LOOP_DT);
    CHECK(boosted.motor[0]>.64f && boosted.motor[0]<=.65001f);
    // Infeasible simultaneous torque requests: proportional allocation stays bounded.
    s.gyro_x_dps=s.gyro_y_dps=s.gyro_z_dps=-1000;
    cmd.roll_deg=cmd.pitch_deg=35;cmd.yaw_rate_dps=180;
    boosted=flight_control_step(&s,&cmd,FC_LOOP_DT);
    for(unsigned i=0;i<4;i++)CHECK(isfinite(boosted.motor[i])&&boosted.motor[i]>=0&&boosted.motor[i]<=1);
    puts("RT boost slew/neutral hold/collective reserve and bounded coupled allocation checks passed");
    puts("matrix prediction, Joseph product, PSD covariance and FLU pitch mixer checks passed");
}

static int write_altitude_mc(const char *path) {
    enum {TRIALS=500,STEPS=2500,STRIDE=10,POINTS=250};
    double sum[POINTS]={0},sq[POINTS]={0},sigma[POINTS]={0};
    unsigned covered[POINTS]={0};
    srand(2352);
    for(unsigned trial=0;trial<TRIALS;trial++) {
        altitude_kf_t k={.height_m=1.5f,.p00=.04f,.p11=.1f,.accel_noise=1.5f,.baro_noise=.0025f};
        float h=1.5f+.2f*gaussian_noise(),v=sqrtf(.1f)*gaussian_noise();
        for(unsigned step=0;step<STEPS;step++) {
            const float noise=1.5f*gaussian_noise();
            h+=v*FC_LOOP_DT+.5f*noise*FC_LOOP_DT*FC_LOOP_DT;v+=noise*FC_LOOP_DT;
            altitude_kf_predict(&k,0,FC_LOOP_DT);
            if(step%10==0) {
                altitude_kf_baro_update(&k,h+.05f*gaussian_noise());
                const unsigned i=step/STRIDE;const double e=k.height_m-h;
                sum[i]+=e;sq[i]+=e*e;sigma[i]+=sqrt(k.p00);
                if(fabs(e)<=2*sqrt(k.p00))covered[i]++;
            }
            CHECK(k.p00>0 && k.p11>0 && k.p00*k.p11-k.p01*k.p10>-1e-10f);
        }
    }
    FILE*f=fopen(path,"w");if(!f)return 2;
    fprintf(f,"time_s,mean_error_m,empirical_std_m,kalman_std_m,coverage_2sigma\n");
    for(unsigned i=0;i<POINTS;i++) {
        double mean=sum[i]/TRIALS;
        fprintf(f,"%.3f,%.8f,%.8f,%.8f,%.6f\n",(i*STRIDE+1)*FC_LOOP_DT,
            mean,sqrt(fmax(0,(sq[i]-TRIALS*mean*mean)/(TRIALS-1))),sigma[i]/TRIALS,(double)covered[i]/TRIALS);
    }
    fclose(f);puts("altitude C Monte Carlo passed: 500 trials x 2500 steps, seed=2352; matched Gaussian model, barometer 50Hz");
    return 0;
}

int main(int argc, char **argv) {
    if(argc==3 && strcmp(argv[1],"--altitude-mc")==0)return write_altitude_mc(argv[2]);
    if (argc == 3 && strcmp(argv[1], "--trace") == 0) {
        return write_trace(argv[2]);
    }
    if ((argc == 3 || argc == 4) && strcmp(argv[1], "--kalman-mc") == 0) {
        const unsigned trials = argc == 4 ? (unsigned)strtoul(argv[3], NULL, 10) : 500;
        return write_kalman_mc(argv[2], trials);
    }
    if (argc != 1) {
        fprintf(stderr, "Usage: %s [--trace output.csv | --kalman-mc output.csv [trials]]\n", argv[0]);
        return 2;
    }
    test_neutral_output_stays_disarmed();
    test_radio_frame_codec_sequence_and_watchdog();
    test_centered_throttle_holds_and_stick_commands_vertical_rate();
    test_bad_sensor_dt_and_command_fail_closed();
    test_pid_limits_and_anti_windup();
    test_estimator_and_mixer_remain_finite();
    test_battery_monitor_filter_soc_and_actions();
    test_imu_calibration_known_bias_gain_and_motion_rejection();
    test_magnetometer_3d_fit_and_planar_rejection();
    test_bmp280_factory_compensation_and_ground_reference();
    test_lidar_voltage_current_reference_regression();
    test_matrix_covariance_and_pitch_sign();
    puts("flight-controller host tests passed (math only; no hardware I/O)");
    return 0;
}
