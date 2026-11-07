/*
 * wifi_pid_tuner.cpp
 * 
 * WiFi Access Point + WebSocket server for wireless PID tuning.
 * Uses ESP-IDF WiFi API directly (avoids Arduino WiFi library dependency issues).
 * ESP32-S3 creates WiFi AP "IntelQCopter_FC" and hosts:
 *   - HTTP server on port 80 (serves web GUI)
 *   - WebSocket server on port 81 (real-time PID tuning + telemetry)
 * 
 * Public interface is extern "C" so it can be called from flight_controller.c.
 */

#include <Arduino.h>

/* ESP-IDF WiFi and networking headers */
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include <ArduinoJson.h>

#include "wifi_pid_tuner.h"
#include "web_gui.h"
#include "flight_controller.h"

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
static bool wifi_active = false;
static char ip_string[16] = "0.0.0.0";

static int httpServerSock = -1;
static int wsServerSock = -1;
static int wsClientSock = -1;

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
   Socket helpers
   =========================================================================== */

static int createServerSocket(uint16_t port) {
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) return -1;
    
    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(sock);
        return -1;
    }
    
    if (listen(sock, 5) < 0) {
        close(sock);
        return -1;
    }
    
    return sock;
}

static bool sendAll(int sock, const uint8_t *data, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = send(sock, data + sent, len - sent, 0);
        if (n <= 0) return false;
        sent += n;
    }
    return true;
}

/* Send a WebSocket text frame */
static bool wsSendText(int sock, const char *payload) {
    size_t len = strlen(payload);
    uint8_t header[10];
    int hlen = 0;
    
    header[hlen++] = 0x81;  /* FIN + text opcode */
    
    if (len <= 125) {
        header[hlen++] = (uint8_t)len;
    } else if (len <= 65535) {
        header[hlen++] = 126;
        header[hlen++] = (uint8_t)(len >> 8);
        header[hlen++] = (uint8_t)(len & 0xFF);
    } else {
        header[hlen++] = 127;
        header[hlen++] = 0;
        header[hlen++] = 0;
        header[hlen++] = 0;
        header[hlen++] = 0;
        header[hlen++] = (uint8_t)(len >> 24);
        header[hlen++] = (uint8_t)(len >> 16);
        header[hlen++] = (uint8_t)(len >> 8);
        header[hlen++] = (uint8_t)len;
    }
    
    if (!sendAll(sock, header, hlen)) return false;
    if (!sendAll(sock, (const uint8_t *)payload, len)) return false;
    return true;
}

/* Receive a WebSocket text frame (non-blocking, returns bytes or -1) */
static int wsRecvText(int sock, char *buf, size_t bufsz) {
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 1000;  /* 1 ms timeout */
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    uint8_t header[2];
    if (recv(sock, header, 2, MSG_WAITALL) != 2) return -1;
    
    uint8_t opcode = header[0] & 0x0F;
    if (opcode != 0x01 && opcode != 0x08) return -1;
    if (opcode == 0x08) return -2;  /* close frame */
    
    uint8_t lenByte = header[1];
    size_t payloadLen = lenByte & 0x7F;
    bool masked = (lenByte & 0x80) != 0;
    
    if (payloadLen == 126) {
        uint8_t ext[2];
        if (recv(sock, ext, 2, MSG_WAITALL) != 2) return -1;
        payloadLen = ((size_t)ext[0] << 8) | ext[1];
    } else if (payloadLen == 127) {
        return -1;  /* too large */
    }
    
    uint8_t mask[4] = {0, 0, 0, 0};
    if (masked) {
        if (recv(sock, mask, 4, MSG_WAITALL) != 4) return -1;
    }
    
    if (payloadLen >= bufsz) payloadLen = bufsz - 1;
    if (payloadLen > 0) {
        if (recv(sock, (uint8_t *)buf, payloadLen, MSG_WAITALL) != (int)payloadLen) return -1;
        if (masked) {
            for (size_t i = 0; i < payloadLen; i++) {
                buf[i] ^= mask[i % 4];
            }
        }
    }
    buf[payloadLen] = 0;
    return (int)payloadLen;
}

/* ===========================================================================
   HTTP request handler
   =========================================================================== */
static void handleHttpClient(int clientSock) {
    char req[256];
    int len = recv(clientSock, req, sizeof(req) - 1, 0);
    if (len <= 0) { close(clientSock); return; }
    req[len] = 0;
    
    /* Skip headers */
    char *body = strstr(req, "\r\n\r\n");
    
    const char *response =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html\r\n"
        "Connection: close\r\n"
        "\r\n";
    
    send(clientSock, response, strlen(response), 0);
    send(clientSock, WEB_GUI_HTML, strlen(WEB_GUI_HTML), 0);
    close(clientSock);
}

/* ===========================================================================
   WiFi AP initialization (ESP-IDF)
   =========================================================================== */
static bool startWifiAp() {
    esp_netif_t *netif = esp_netif_create_default_wifi_ap();
    if (!netif) return false;
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        Serial.printf("WiFi init failed: %s\n", esp_err_to_name(ret));
        return false;
    }
    
    wifi_config_t wifi_config = {};
    memcpy(wifi_config.ap.ssid, WIFI_AP_SSID, strlen(WIFI_AP_SSID));
    memcpy(wifi_config.ap.password, WIFI_AP_PASSWORD, strlen(WIFI_AP_PASSWORD));
    wifi_config.ap.ssid_len = strlen(WIFI_AP_SSID);
    wifi_config.ap.channel = WIFI_AP_CHANNEL;
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    wifi_config.ap.max_connection = WIFI_AP_MAX_CONN;
    
    ret = esp_wifi_set_mode(WIFI_MODE_AP);
    if (ret != ESP_OK) {
        Serial.printf("WiFi set mode failed: %s\n", esp_err_to_name(ret));
        return false;
    }
    
    ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    if (ret != ESP_OK) {
        Serial.printf("WiFi set config failed: %s\n", esp_err_to_name(ret));
        return false;
    }
    
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        Serial.printf("WiFi start failed: %s\n", esp_err_to_name(ret));
        return false;
    }
    
    esp_netif_ip_info_t ipInfo;
    if (esp_netif_get_ip_info(netif, &ipInfo) == ESP_OK) {
        snprintf(ip_string, sizeof(ip_string), IPSTR, IP2STR(&ipInfo.ip));
    }
    
    Serial.printf("WiFi AP started: SSID=%s IP=%s\n", WIFI_AP_SSID, ip_string);
    wifi_active = true;
    return true;
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
    
    httpServerSock = createServerSocket(HTTP_PORT);
    wsServerSock = createServerSocket(WS_PORT);
    
    if (httpServerSock < 0) Serial.println("HTTP server socket failed");
    if (wsServerSock < 0) Serial.println("WebSocket server socket failed");
    
    Serial.printf("HTTP server started on port %d\n", HTTP_PORT);
    Serial.printf("WebSocket server started on port %d\n", WS_PORT);
    Serial.printf("Connect to: http://%s\n", ip_string);
}

void wifi_pid_tuner_loop(float dt) {
    (void)dt;
    
    if (!wifi_active) return;
    
    /* Set sockets to non-blocking */
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    
    /* Handle HTTP connections */
    if (httpServerSock >= 0) {
        setsockopt(httpServerSock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        int clientSock = accept(httpServerSock, NULL, NULL);
        if (clientSock >= 0) {
            handleHttpClient(clientSock);
        }
    }
    
    /* Handle WebSocket connections */
    if (wsServerSock >= 0) {
        setsockopt(wsServerSock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        if (wsClientSock < 0) {
            int newClient = accept(wsServerSock, NULL, NULL);
            if (newClient >= 0) {
                /* Set client to non-blocking */
                setsockopt(newClient, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
                wsClientSock = newClient;
                
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
                wsSendText(wsClientSock, msg.c_str());
            }
        }
    }
    
    /* Handle WebSocket client data */
    if (wsClientSock >= 0) {
        char buf[512];
        int len = wsRecvText(wsClientSock, buf, sizeof(buf));
        if (len > 0) {
            buf[len] = 0;
            
            StaticJsonDocument<512> doc;
            DeserializationError err = deserializeJson(doc, buf);
            if (!err) {
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
                        Serial.printf("PID updated: %s kp=%.4f ki=%.4f kd=%.4f\n",
                                     axis, kp, ki, kd);
                        
                        StaticJsonDocument<128> ack;
                        ack["type"] = "pid_ack";
                        ack["axis"] = axis;
                        String ackMsg;
                        serializeJson(ack, ackMsg);
                        wsSendText(wsClientSock, ackMsg.c_str());
                    } else {
                        StaticJsonDocument<128> errDoc;
                        errDoc["type"] = "error";
                        errDoc["message"] = "Invalid axis or PID values";
                        String errMsg;
                        serializeJson(errDoc, errMsg);
                        wsSendText(wsClientSock, errMsg.c_str());
                    }
                }
                else if (strcmp(typeStr, "pid_get") == 0) {
                    StaticJsonDocument<512> docOut;
                    docOut["type"] = "pid_values";
                    JsonObject pids = docOut.createNestedObject("pids");
                    for (int i = 0; i < 7; i++) {
                        JsonObject axis = pids.createNestedObject(index_to_axis(i));
                        axis["kp"] = cached_pids[i].kp;
                        axis["ki"] = cached_pids[i].ki;
                        axis["kd"] = cached_pids[i].kd;
                    }
                    String msg;
                    serializeJson(docOut, msg);
                    wsSendText(wsClientSock, msg.c_str());
                }
                else if (strcmp(typeStr, "disarm") == 0) {
                    Serial.println("Emergency disarm received via WebSocket");
                    StaticJsonDocument<128> ack;
                    ack["type"] = "ack";
                    ack["message"] = "disarm_received";
                    String ackMsg;
                    serializeJson(ack, ackMsg);
                    wsSendText(wsClientSock, ackMsg.c_str());
                }
            }
        } else if (len == -2) {
            /* Close frame */
            close(wsClientSock);
            wsClientSock = -1;
        } else if (len < 0) {
            /* Connection error or closed */
            close(wsClientSock);
            wsClientSock = -1;
        }
    }
    
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
        
        if (wsClientSock >= 0) {
            wsSendText(wsClientSock, msg.c_str());
        }
        
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
    uint32_t now = millis();
    if (now - last_telemetry_ms >= 10) {
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
        
        if (wsClientSock >= 0) {
            wsSendText(wsClientSock, msg.c_str());
        }
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