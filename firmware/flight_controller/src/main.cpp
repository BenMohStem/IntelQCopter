/*
 * src/main.cpp
 * 
 * IntelQCopter Flight Controller - ESP32-S3 Main Entry Point
 * ============================================================================
 * 
 * Integrates:
 *   - flight_controller.c (core control logic)
 *   - wifi_pid_tuner.cpp (WiFi AP + WebSocket server for wireless PID tuning)
 *   - Sensor reading (placeholder - replace with real I2C drivers)
 * 
 * WiFi AP: "IntelQCopter_FC" / "qcopter123" at 192.168.4.1
 * Web GUI: http://192.168.4.1
 * WebSocket: ws://192.168.4.1:81
 * 
 * Build: pio run -t upload --upload-port COM3
 */

#include <Arduino.h>

extern "C" {
#include "flight_controller.h"
#include "wifi_pid_tuner.h"
}

/* ===========================================================================
   Sensor reading (placeholder)
   ===========================================================================
   Replace these stub functions with real I2C sensor drivers for:
   - MPU6050 (accel + gyro)
   - QMC5883P (magnetometer)
   - BMP280 (barometer)
   =========================================================================== */

static sensor_sample_t g_sensor;
static uint32_t g_last_loop_ms = 0;
static float g_loop_dt = 0.002f;   /* 500 Hz = 2 ms */

static void read_sensors(sensor_sample_t *s) {
    /* TODO: Replace with real I2C sensor reads */
    s->valid = true;
    s->fresh = true;
    s->timestamp_us = micros();
    
    /* Simulated sensor data (for testing without hardware) */
    s->accel_x_g = 0.0f;
    s->accel_y_g = 0.0f;
    s->accel_z_g = 9.80665f;
    s->gyro_x_dps = 0.0f;
    s->gyro_y_dps = 0.0f;
    s->gyro_z_dps = 0.0f;
    s->temperature_c = 25.0f;
    s->barometer_height_m = 0.0f;
    s->barometer_valid = true;
    s->barometer_updated = true;
    s->barometer_raw_pa = 101325.0f;
    s->barometer_temperature_c = 25.0f;
    s->magnetometer_x_uT = 0.0f;
    s->magnetometer_y_uT = 0.0f;
    s->magnetometer_z_uT = 45.0f;
    s->magnetometer_valid = true;
    s->magnetometer_fresh = true;
    s->vertical_acceleration_mps2 = 0.0f;
}

/* Pilot command stub (from UART - replace with real UART reader) */
static pilot_command_t g_pilot_cmd;
static bool g_pilot_cmd_valid = false;

static void read_pilot_command(void) {
    /* TODO: Replace with real UART command reading */
    g_pilot_cmd.valid = false;  /* No pilot command - drone stays disarmed */
    g_pilot_cmd.timestamp_ms = millis();
    g_pilot_cmd.roll = 0.0f;
    g_pilot_cmd.pitch = 0.0f;
    g_pilot_cmd.yaw = 0.0f;
    g_pilot_cmd.throttle = 0.0f;
    g_pilot_cmd.rt = 0.0f;
    g_pilot_cmd.lt = 0.0f;
    g_pilot_cmd.buttons = 0;
}

/* ===========================================================================
   Setup
   =========================================================================== */
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=== IntelQCopter Flight Controller ===");
    Serial.println("Target: ESP32-S3 DevKitC-1 N8");
    Serial.println("WiFi PID Tuner: Enabled");
    Serial.println("Motor Outputs: DISABLED (INPUT ONLY)");
    Serial.println();
    
    /* Initialize flight controller (also starts WiFi PID tuner) */
    fc_init();
    
    Serial.println("Flight controller initialized.");
    Serial.println("WiFi AP: IntelQCopter_FC / qcopter123");
    Serial.println("Web GUI: http://192.168.4.1");
    Serial.println("WebSocket: ws://192.168.4.1:81");
    Serial.println("========================================");
    
    g_last_loop_ms = millis();
}

/* ===========================================================================
   Main loop - 500 Hz control cycle
   =========================================================================== */
void loop() {
    uint32_t now = millis();
    float dt = (now - g_last_loop_ms) / 1000.0f;
    g_last_loop_ms = now;
    
    /* Clamp dt to prevent large jumps */
    if (dt > 0.010f) dt = 0.010f;
    if (dt <= 0.0f) dt = 0.002f;
    
    /* Read sensors */
    read_sensors(&g_sensor);
    
    /* Read pilot command (UART) */
    read_pilot_command();
    
    /* Run flight control */
    control_output_t output = fc_get_output(&g_sensor, 
                                             g_pilot_cmd_valid ? &g_pilot_cmd : NULL,
                                             dt);
    
    /* Run WiFi PID tuner loop (handles WebSocket clients, broadcasts telemetry) */
    wifi_pid_tuner_loop(dt);
    
    /* Motor outputs - currently disabled (safety) */
    /* The output.motor[] values are computed but not sent to any hardware */
    
    /* Debug output every 100ms */
    static uint32_t last_debug = 0;
    if (now - last_debug >= 100) {
        last_debug = now;
        Serial.printf("Armed: %d | Motors: %.3f %.3f %.3f %.3f\n",
                      output.armed ? 1 : 0,
                      output.motor[0], output.motor[1],
                      output.motor[2], output.motor[3]);
    }
    
    /* Yield for WiFi/ESP32 housekeeping */
    yield();
}
