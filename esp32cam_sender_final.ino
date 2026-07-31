#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>

// --- wifi config ---
// running off the phone hotspot for testing
const char* ssid = "***";
const char* password = "***";

// static ip config (commented out in setup below for now)
IPAddress local_IP(10, 34, 124, 32);
IPAddress gateway(10, 34, 124, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiUDP udp;
const int udpPort = 8888;
char packetBuffer[255]; 

// standard AI Thinker cam pinout - don't touch these
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// --- stream & adaptive quality hack ---
// trying to keep the feed from choking when signal drops
#define HEADER_MAX 96
#define MAX_JPEG_SIZE 32768 // 32kb should be plenty for QVGA
static uint8_t* sendBuf = nullptr; 

sensor_t* camSensor = nullptr;

const int BASE_QUALITY = 15; // lower is better quality for ESP32 cams
const int MAX_QUALITY_NUM = 26; // max compression before it looks like literal potato
int currentQuality = BASE_QUALITY;
int slowStreak = 0;
int fastStreak = 0;
const unsigned long SLOW_SEND_MS = 60;   
const int SLOW_STREAK_TO_STEP_UP = 5;    
const int FAST_STREAK_TO_STEP_DOWN = 90; 

// background task to pump frames over HTTP multipart
void camStreamTask(void *pvParameters) {
    WiFiServer server(81); 
    server.begin();
    
    for (;;) {
        WiFiClient client = server.available();
        if (client) {
            client.setNoDelay(true); // force packets out instantly
            
            // MJPEG header
            client.print("HTTP/1.1 200 OK\r\n");
            client.print("Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n");
            
            while (client.connected()) {
                camera_fb_t * fb = esp_camera_fb_get();
                if (fb) {
                    // only send if it fits in our buffer
                    if (fb->len <= MAX_JPEG_SIZE) {
                        size_t headerLen = snprintf((char*)sendBuf, HEADER_MAX,
                            "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);

                        // pack header, image data, and boundary into one buffer to prevent tcp fragmentation overhead
                        memcpy(sendBuf + headerLen, fb->buf, fb->len);
                        memcpy(sendBuf + headerLen + fb->len, "\r\n", 2);

                        size_t totalLen = headerLen + fb->len + 2;

                        unsigned long sendStart = millis();
                        size_t written = client.write(sendBuf, totalLen);
                        unsigned long sendDuration = millis() - sendStart;

                        // client disconnected mid-send
                        if (written != totalLen) {
                            esp_camera_fb_return(fb);
                            break;
                        }

                        // adaptive quality logic:
                        // if sending takes too long, downgrade quality. 
                        // if we get a long streak of fast sends, slowly bump quality back up.
                        if (sendDuration > SLOW_SEND_MS) {
                            slowStreak++; fastStreak = 0;
                            if (slowStreak >= SLOW_STREAK_TO_STEP_UP && currentQuality < MAX_QUALITY_NUM) {
                                currentQuality = min(currentQuality + 2, MAX_QUALITY_NUM);
                                if (camSensor) camSensor->set_quality(camSensor, currentQuality);
                                slowStreak = 0;
                            }
                        } else {
                            fastStreak++; slowStreak = 0;
                            if (fastStreak >= FAST_STREAK_TO_STEP_DOWN && currentQuality > BASE_QUALITY) {
                                currentQuality = max(currentQuality - 1, BASE_QUALITY);
                                if (camSensor) camSensor->set_quality(camSensor, currentQuality);
                                fastStreak = 0;
                            }
                        }
                    }
                    esp_camera_fb_return(fb); // always return the frame buffer!
                }
                vTaskDelay(pdMS_TO_TICKS(1)); // yield to watchdog
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    // DO NOT USE standard Serial.begin() HERE.
    // GPIO 1 (TX) is wired straight to the Nano. If the ESP spits out boot info
    // or debug logs, it absolutely nukes the Nano's packet parser.
    // Using Serial1 instead on specific pins.
    Serial1.begin(38400, SERIAL_8N1, 13, 1); 

    // try to stuff the send buffer in PSRAM, fallback to regular heap if it fails
    sendBuf = (uint8_t*)ps_malloc(HEADER_MAX + MAX_JPEG_SIZE + 2);
    if (!sendBuf) sendBuf = (uint8_t*)malloc(HEADER_MAX + MAX_JPEG_SIZE + 2);

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000; // dropping this from 20MHz to 10MHz sometimes helps with green lines, but 20 is fine for now
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_QVGA; // keeping it small for latency
    config.jpeg_quality = BASE_QUALITY;
    config.fb_count = 3;                       
    config.fb_location = CAMERA_FB_IN_PSRAM; // must use psram for multiple buffers
    config.grab_mode = CAMERA_GRAB_LATEST; // always grab the newest frame so we don't stream delayed garbage     

    if (esp_camera_init(&config) == ESP_OK) {
        camSensor = esp_camera_sensor_get();
        if (camSensor) camSensor->set_quality(camSensor, BASE_QUALITY);
    }

    //WiFi.config(local_IP, gateway, subnet); // skipping static ip for now
    
    // absolutely necessary for streaming, otherwise wifi power saving ruins the framerate
    WiFi.setSleep(false); 
    WiFi.begin(ssid, password);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }
    
    // max out the wifi transmitter 
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    udp.begin(udpPort);
    
    // pin the camera task to core 1 (core 0 usually handles wifi)
    xTaskCreatePinnedToCore(camStreamTask, "CamStream", 8192, NULL, 2, NULL, 1); 
}

void loop() {
    // listen for incoming flight control UDP packets
    int packetSize = udp.parsePacket();
    if (packetSize) {
        int len = udp.read(packetBuffer, 254);
        if (len > 0) {
            packetBuffer[len] = '\0'; 
            
            // pipe it directly to the Nano over Serial
            // format strictness matters here: <mode,joyX,joyY>\n
            Serial1.printf("<%s>\n", packetBuffer); 
        }
    }
    delay(1);
}