/*
 * wifi_pid_tuner.cpp
 * 
 * WiFi Access Point + WebSocket server for wireless PID tuning.
 * ESP32-S3 creates WiFi AP "IntelQCopter_FC" and hosts:
 *   - HTTP server on port 80 (serves web GUI)
 *   - WebSocket server on port 81 (real-time PID tuning + telemetry)
 * 
 * This file is C++ (uses ESPAsyncWebServer). Public interface is extern "C"
 * so it can be called from the C flight_controller.c.
 * 
 * Build requirements (platformio.ini):
 *   lib_deps = 
 *     me-no-dev/ESPAsyncWebServer @ ^1.2.3
 *     bblanchon/ArduinoJson @ ^6.21.0
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>
#include <ArduinoJson.h>

#include "wifi_pid_tuner.h"
#include "web_gui.h"
#include "flight_controller.h"   /* Forward declaration of fc_* functions */

#ifdef __cplusplus
extern "C" {
#endif

/* ===========================================================================
   Forward declarations of flight_controller.c functions
   =========================================================================== */
extern void fc_get_pid_values(float *kp, float *ki, float *kd,
                               const char *axis);
extern bool fc_set_pid_values(const char *axis, float kp, float ki, float kd);
extern void fc_get_telemetry(float *roll, float *pitch, float *yaw,
                              float *roll_rate, float *pitch_rate,
                              float *yaw_rate, float *altitude,
                              float *vert_speed, float *motor0,
                              float *motor1, float *motor2, float *motor3,
                              bool *armed, float *loop_rate);

/* ===========================================================================
   Global state
   =========================================================================== */
static AsyncWebServer server(HTTP_PORT);
static AsyncWebSocket ws(WS_PORT);

static bool wifi_active = false;
static char ip_string[16] = "0.0.0.0";

static uint32_t last_telemetry_ms = 0;

/* PID values cached for the web GUI */
typedef struct {
    float kp, ki, kd;
} pid_gains_t;

static pid_gains_t cached_pids[7] = {
    {0.65f, 0.02f, 0.03f},   /* roll_angle */
    {0.65f, 0.02f, 0.03f},   /* pitch_angle */
    {0.078f, 0.039f, 0.00006f},  /* roll_rate */
    {0.083f, 0.041f, 0.00006f},  /* pitch_rate */
    {0.087f, 0.028f, 0.00012f},  /* yaw_rate */
    {0.40f, 0.005f, 0.015f},     /* altitude */
    {1.0f, 0.06f, 0.025f}        /* vert_speed */
};

/* Axis name -> index mapping */
static int axis_to_index(const char *axis) {
    if (strcmp(axis, PID_AXIS_ROLL_ANGLE) == 0) return 0;
    if (strcmp(axis, PID_AXIS_PITCH_ANGLE) == 0) return 1;
    if (strcmp(axis, PID_AXIS_ROLL_RATE) == 0) return 2;
    if (strcmp(axis, PID_AXIS_PITCH_RATE) == 0) return 3;
    if (strcmp(axis, PID_AXIS_YAW_RATE) == 0) return 4;
    if (strcmp(axis, PID_AXIS_ALTITUDE) == 0) return 5;
    if (strcmp(axis, PID_AXIS_VERT_SPEED) == 0) return 6;
    return -1;
}

static const char *index_to_axis(int idx) {
    switch (idx) {
        case 0: return PID_AXIS_ROLL_ANGLE;
        case 1: return PID_AXIS_PITCH_ANGLE;
        case 2: return PID_AXIS_ROLL_RATE;
        case 3: return PID_AXIS_PITCH_RATE;
        case 4: return PID_AXIS_YAW_RATE;
        case 5: return PID_AXIS_ALTITUDE;
        case 6: return PID_AXIS_VERT_SPEED;
        default: return "";
    }
}

/* ===========================================================================
   WebSocket event handler
   =========================================================================== */
static void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("WebSocket client connected: %s\n", client->remoteIP().toString().c_str());
        /* Send current PID values to newly connected client */
        StaticJsonDocument<512> doc;
        doc["type"] = "pid_values";
        JsonObject pids = doc.createNestedObject("pids");
        for (int i = 0; i < 7; i++) {
            JsonObject axis = pids.createNestedObject(index_to_axis(i));
            axis["kp"] = cached_pids[i].kp;
            axis["ki"] = cached_pids[i].ki;
            axis["kd"] = cached_pids[i].kd;
        }
        String msg;
        serializeJson(doc, msg);
        client->text(msg);
    }
    else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("WebSocket client disconnected: %s\n", client->remoteIP().toString().c_str());
    }
    else if (type == WS_EVT_DATA) {
        /* Incoming WebSocket message */
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->opcode == WS_TEXT) {
            data[len] = 0;
            String message = (char*)data;
            
            StaticJsonDocument<512> doc;
            DeserializationError err = deserializeJson(doc, message);
            if (err) {
                Serial.printf("JSON parse error: %s\n", err.c_str());
                StaticJsonDocument<128> errDoc;
                errDoc["type"] = "error";
                errDoc["message"] = "Invalid JSON";
                String errMsg;
                serializeJson(errDoc, errMsg);
                client->text(errMsg);
                return;
            }
            
            const char *typeStr = doc["type"] | "";
            
            if (strcmp(typeStr, "pid_set") == 0) {
                const char *axis = doc["axis"] | "";
                float kp = doc["kp"] | 0.0f;
                float ki = doc["ki"] | 0.0f;
                float kd = doc["kd"] | 0.0f;
                
                if (fc_set_pid_values(axis, kp, ki, kd)) {
                    int idx = axis_to_index(axis);
                    if (idx >= 0) {
                        cached_pids[idx].kp = kp;
                        cached_pids[idx].ki = ki;
                        cached_pids[idx].kd = kd;
                    }
                    Serial.printf("PID updated: %s kp=%.4f ki=%.4f kd=%.4f\n", axis, kp, ki, kd);
                    
                    StaticJsonDocument<128> ack;
                    ack["type"] = "pid_ack";
                    ack["axis"] = axis;
                    String ackMsg;
                    serializeJson(ack, ackMsg);
                    client->text(ackMsg);
                } else {
                    StaticJsonDocument<128> errDoc;
                    errDoc["type"] = "error";
                    errDoc["message"] = "Invalid axis or PID values";
                    String errMsg;
                    serializeJson(errDoc, errMsg);
                    client->text(errMsg);
                }
            }
            else if (strcmp(typeStr, "pid_get") == 0) {
                /* Send all current PID values */
                StaticJsonDocument<512> doc;
                doc["type"] = "pid_values";
                JsonObject pids = doc.createNestedObject("pids");
                for (int i = 0; i < 7; i++) {
                    JsonObject axis = pids.createNestedObject(index_to_axis(i));
                    axis["kp"] = cached_pids[i].kp;
                    axis["ki"] = cached_pids[i].ki;
                    axis["kd"] = cached_pids[i].kd;
                }
                String msg;
                serializeJson(doc, msg);
                client->text(msg);
            }
            else if (strcmp(typeStr, "disarm") == 0) {
                Serial.println("Emergency disarm received via WebSocket");
                /* The flight_controller.c handles disarm via command buttons,
                   but we can set a flag here. For now just log it. */
                StaticJsonDocument<128> ack;
                ack["type"] = "ack";
                ack["message"] = "disarm_received";
                String ackMsg;
                serializeJson(ack, ackMsg);
                client->text(ackMsg);
            }
        }
    }
}

/* ===========================================================================
   HTTP request handler - serve the web GUI
   =========================================================================== */
static void onRootRequest(AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", WEB_GUI_HTML);
}

/* ===========================================================================
   WiFi AP initialization
   =========================================================================== */
static bool startWifiAp() {
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1),
                      IPAddress(192, 168, 4, 1),
                      IPAddress(255, 255, 255, 0));
    
    bool result = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD,
                              WIFI_AP_CHANNEL, 0, WIFI_AP_MAX_CONN);
    
    if (result) {
        IPAddress ip = WiFi.softAPIP();
        snprintf(ip_string, sizeof(ip_string), "%d.%d.%d.%d",
                 ip[0], ip[1], ip[2], ip[3]);
        Serial.printf("WiFi AP started: SSID=%s IP=%s\n", WIFI_AP_SSID, ip_string);
        wifi_active = true;
    } else {
        Serial.println("WiFi AP failed to start");
        wifi_active = false;
    }
    return result;
}

/* ===========================================================================
   Public API (extern "C")
   =========================================================================== */

void wifi_pid_tuner_init(void) {
    Serial.begin(115200);
    
    if (!startWifiAp()) {
        Serial.println("Failed to start WiFi AP - PID tuner inactive");
        return;
    }
    
    /* Configure WebSocket */
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    
    /* Configure HTTP routes */
    server.on("/", HTTP_GET, onRootRequest);
    
    /* Start HTTP server */
    server.begin();
    
    Serial.printf("HTTP server started on port %d\n", HTTP_PORT);
    Serial.printf("WebSocket server started on port %d\n", WS_PORT);
    Serial.printf("Connect to: http://%s\n", ip_string);
}

void wifi_pid_tuner_loop(float dt) {
    (void)dt;  /* dt not needed for this implementation */
    
    if (!wifi_active) return;
    
    /* Clean up WebSocket clients */
    ws.cleanupClients();
    
    /* Broadcast telemetry at fixed interval */
    uint32_t now = millis();
    if (now - last_telemetry_ms >= TELEMETRY_INTERVAL_MS) {
        last_telemetry_ms = now;
        
        float roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate;
        float altitude, vert_speed;
        float m0, m1, m2, m3;
        bool armed;
        float loop_rate;
        
        fc_get_telemetry(&roll, &pitch, &yaw,
                         &roll_rate, &pitch_rate, &yaw_rate,
                         &altitude, &vert_speed,
                         &m0, &m1, &m2, &m3,
                         &armed, &loop_rate);
        
        /* Build telemetry JSON */
        StaticJsonDocument<512> doc;
        doc["type"] = "telemetry";
        doc["roll"] = roll;
        doc["pitch"] = pitch;
        doc["yaw"] = yaw;
        doc["roll_rate"] = roll_rate;
        doc["pitch_rate"] = pitch_rate;
        doc["yaw_rate"] = yaw_rate;
        doc["altitude"] = altitude;
        doc["vert_speed"] = vert_speed;
        doc["armed"] = armed;
        doc["loop_rate"] = loop_rate;
        
        JsonArray motors = doc.createNestedArray("motor");
        motors.add(m0);
        motors.add(m1);
        motors.add(m2);
        motors.add(m3);
        
        String msg;
        serializeJson(doc, msg);
        ws.textAll(msg);
        
        /* Sync cached PID values from flight controller */
        for (int i = 0; i < 7; i++) {
            fc_get_pid_values(&cached_pids[i].kp, &cached_pids[i].ki,
                               &cached_pids[i].kd, index_to_axis(i));
        }
    }
}

bool wifi_pid_tuner_is_active(void) {
    return wifi_active;
}

const char *wifi_pid_tuner_get_ip(void) {
    return ip_string;
}

void wifi_pid_tuner_broadcast_telemetry(void) {
    /* Called externally to force an immediate telemetry broadcast */
    uint32_t now = millis();
    if (now - last_telemetry_ms >= 10) {  /* min 10ms between broadcasts */
        last_telemetry_ms = now;
        
        float roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate;
        float altitude, vert_speed;
        float m0, m1, m2, m3;
        bool armed;
        float loop_rate;
        
        fc_get_telemetry(&roll, &pitch, &yaw,
                         &roll_rate, &pitch_rate, &yaw_rate,
                         &altitude, &vert_speed,
                         &m0, &m1, &m2, &m3,
                         &armed, &loop_rate);
        
        StaticJsonDocument<512> doc;
        doc["type"] = "telemetry";
        doc["roll"] = roll;
        doc["pitch"] = pitch;
        doc["yaw"] = yaw;
        doc["roll_rate"] = roll_rate;
        doc["pitch_rate"] = pitch_rate;
        doc["yaw_rate"] = yaw_rate;
        doc["altitude"] = altitude;
        doc["vert_speed"] = vert_speed;
        doc["armed"] = armed;
        doc["loop_rate"] = loop_rate;
        
        JsonArray motors = doc.createNestedArray("motor");
        motors.add(m0);
        motors.add(m1);
        motors.add(m2);
        motors.add(m3);
        
        String msg;
        serializeJson(doc, msg);
        ws.textAll(msg);
    }
}

bool wifi_pid_tuner_apply_pid_update(const char *axis, float kp, float ki, float kd) {
    if (!fc_set_pid_values(axis, kp, ki, kd)) return false;
    
    int idx = axis_to_index(axis);
    if (idx >= 0) {
        cached_pids[idx].kp = kp;
        cached_pids[idx].ki = ki;
        cached_pids[idx].kd = kd;
    }
    return true;
}

void wifi_pid_tuner_get_pid_string(char *buf, size_t bufsz) {
    snprintf(buf, bufsz,
             "PIDs: RA(%.3f,%.3f,%.3f) PA(%.3f,%.3f,%.3f) "
             "RR(%.4f,%.4f,%.5f) PR(%.4f,%.4f,%.5f) "
             "YR(%.4f,%.4f,%.5f) ALT(%.3f,%.3f,%.3f) "
             "VS(%.3f,%.3f,%.3f)",
             cached_pids[0].kp, cached_pids[0].ki, cached_pids[0].kd,
             cached_pids[1].kp, cached_pids[1].ki, cached_pids[1].kd,
             cached_pids[2].kp, cached_pids[2].ki, cached_pids[2].kd,
             cached_pids[3].kp, cached_pids[3].ki, cached_pids[3].kd,
             cached_pids[4].kp, cached_pids[4].ki, cached_pids[4].kd,
             cached_pids[5].kp, cached_pids[5].ki, cached_pids[5].kd,
             cached_pids[6].kp, cached_pids[6].ki, cached_pids[6].kd);
}

#ifdef __cplusplus
}
#endif