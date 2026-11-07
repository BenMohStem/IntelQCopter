#ifndef WIFI_PID_TUNER_H
#define WIFI_PID_TUNER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===========================================================================
   WiFi PID Tuner - Wireless PID tuning via WebSocket web server
   ===========================================================================
   
   The ESP32-S3 creates a WiFi Access Point and hosts a WebSocket server.
   A PC browser connects to the AP, loads a GUI served from ESP32 flash,
   and can tune PID parameters in real time while receiving telemetry.
   
   WiFi AP config:
     SSID:    "IntelQCopter_FC"
     Password: "qcopter123"   (change for production)
     IP:      192.168.4.1
   
   WebSocket port: 81
   HTTP server port: 80
   
   Message format (JSON):
     Client -> Server: {"type":"pid_set","axis":"roll_angle","kp":0.65,"ki":0.02,"kd":0.03}
     Server -> Client: {"type":"telemetry","roll":12.3,"pitch":-2.1,...,"motor":[0.10,0.12,0.11,0.13],...}
   
   =========================================================================== */

/* WiFi configuration */
#define WIFI_AP_SSID      "IntelQCopter_FC"
#define WIFI_AP_PASSWORD  "qcopter123"
#define WIFI_AP_CHANNEL   1
#define WIFI_AP_MAX_CONN  4

/* WebSocket & HTTP ports */
#define WS_PORT 81
#define HTTP_PORT 80

/* Telemetry broadcast interval (ms) */
#define TELEMETRY_INTERVAL_MS 50   /* 20 Hz telemetry broadcast */

/* PID axis identifiers (match WebSocket JSON "axis" field) */
#define PID_AXIS_ROLL_ANGLE   "roll_angle"
#define PID_AXIS_PITCH_ANGLE  "pitch_angle"
#define PID_AXIS_ROLL_RATE    "roll_rate"
#define PID_AXIS_PITCH_RATE   "pitch_rate"
#define PID_AXIS_YAW_RATE     "yaw_rate"
#define PID_AXIS_ALTITUDE     "altitude"
#define PID_AXIS_VERT_SPEED   "vert_speed"

/* Initialize WiFi AP + WebSocket server (call once at boot) */
void wifi_pid_tuner_init(void);

/* Run WiFi PID tuner loop (call from main 500 Hz loop) */
void wifi_pid_tuner_loop(float dt);

/* Check if WiFi is connected and AP is active */
bool wifi_pid_tuner_is_active(void);

/* Get IP address string (for display/logging) */
const char *wifi_pid_tuner_get_ip(void);

/* Broadcast telemetry to all WebSocket clients */
void wifi_pid_tuner_broadcast_telemetry(void);

/* Apply PID update received from WebSocket client */
bool wifi_pid_tuner_apply_pid_update(const char *axis, float kp, float ki, float kd);

/* Get current PID values as formatted string (for debugging) */
void wifi_pid_tuner_get_pid_string(char *buf, size_t bufsz);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_PID_TUNER_H */