#include <WiFi.h>
#include <WiFiUdp.h>
#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>
#include <lvgl.h>
#include <math.h>

#ifndef RGB_BUILTIN
#define RGB_BUILTIN 48
#endif

// --- Hardware & Settings ---
const int MOTOR_PIN = 42;       // Haptic motor pin
const int MODE_SWITCH_PIN = 18; // Toggle switch for flight modes
const int JOY_X_PIN = 10;       // Joystick X axis
const int JOY_Y_PIN = 11;       // Joystick Y axis

bool INVERT_X = true;  // X-axis inverted based on my joystick orientation
bool INVERT_Y = false;

const char* ssid = "***";
const char* password = "***";

// network fix: force the controller to .33 so it shares the subnet with the drone camera on .32
IPAddress local_IP(10, 34, 124, 33);
IPAddress gateway(10, 34, 124, 1);
IPAddress subnet(255, 255, 255, 0);

const char* host = "***"; // drone IP
const int streamPort = 81;
const char* streamPath = "/stream";
const int controlPort = 8888;

#define MAX_JPEG_SIZE 32768
#define CAM_FRAME_WIDTH  320
#define CAM_FRAME_HEIGHT 240
#define SCREEN_W 320
#define SCREEN_H 240

// scaling math for the video stream
#define SCALE_NUM 3
#define SCALE_DEN 4
#define ROT_CANVAS_W CAM_FRAME_HEIGHT                      
#define ROT_CANVAS_H CAM_FRAME_WIDTH                       
#define SCALED_W ((ROT_CANVAS_W * SCALE_NUM) / SCALE_DEN)  
#define SCALED_H ((ROT_CANVAS_H * SCALE_NUM) / SCALE_DEN)  
#define CAM_OFFSET_X ((SCREEN_W - SCALED_W) / 2)           
#define CAM_OFFSET_Y ((SCREEN_H - SCALED_H) / 2)           

// UI colors
#define COL_BG        0x0B0B10
#define COL_BG_ALT    0x08080B
#define COL_CARD      0x171720
#define COL_BORDER    0x2A2A38
#define COL_CYAN      0x00E5FF
#define COL_YELLOW    0xFFD400
#define COL_MAGENTA   0xFF2FD0
#define COL_ORANGE    0xFF9500
#define COL_GREEN     0x30E070
#define COL_RED       0xFF4455
#define COL_MUTED     0x8A8AA0
#define COL_TEXT      0xE8E8F0

// --- Globals ---
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite joySprite = TFT_eSprite(&tft);
TFT_eSprite droneSprite = TFT_eSprite(&tft);

WiFiClient streamClient;
WiFiUDP udpClient;
static uint8_t* jpegBuf = nullptr;

enum SystemState { STATE_BOOT_CHECK, STATE_OFFLINE, STATE_CONNECTING, STATE_ONLINE };
enum OnlineView { VIEW_DASHBOARD, VIEW_CAMSTREAM };
enum FlightMode { MODE_XY_PLANE, MODE_Z_PLANE };
enum HapticPattern { HAPTIC_OFF, HAPTIC_BUMP, HAPTIC_SUCCESS, HAPTIC_ERROR, HAPTIC_HOLDING };

volatile SystemState currentState = STATE_BOOT_CHECK;
volatile OnlineView currentOnlineView = VIEW_DASHBOARD;
volatile FlightMode currentFlightMode = MODE_XY_PLANE;
volatile bool camLandscape = false; // toggle for camera view mode

volatile bool modeTransitioning = false;
volatile unsigned long modeTransitionStart = 0;
const int MODE_TRANSITION_MS = 260;
volatile FlightMode transitionFromMode = MODE_XY_PLANE;

volatile int lastSwitchState = -1;
volatile unsigned long lastControlSendTime = 0;
unsigned long lastUiUpdateTime = 0; 
unsigned long camViewEnteredAt = 0; 

// Boot calibration tracking
volatile int bootCheckPhase = 0;
volatile unsigned long phaseStartTime = 0;
volatile long centerSumX = 0, centerSumY = 0;
volatile int centerCount = 0;
volatile int joyX_center = 2048, joyY_center = 2048;
volatile int joyX_min = 4095, joyX_max = 0;
volatile int joyY_min = 4095, joyY_max = 0;

volatile float uiCurrentX = 0.0f;
volatile float uiCurrentY = 0.0f;

// Joystick button handling
volatile bool isButtonHolding = false;
volatile bool buttonCooldown = false;
volatile unsigned long buttonHoldStartTime = 0;
const int HOLD_DURATION = 1500;      
const int LONG_HOLD_DURATION = 3500; 
volatile bool shortHoldFired = false;
volatile bool longHoldFired = false;

volatile long raw_joyX = 2048;
volatile long raw_joyY = 2048;

volatile bool reqTftClear = false;
volatile bool reqLoadOffline = false;
volatile bool reqLoadOnline = false;

TaskHandle_t ControlTaskHandle;

// LVGL buffer setup
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[SCREEN_W * 20]; 

static lv_img_dsc_t joy_img_dsc;
static lv_img_dsc_t drone_img_dsc;

// LVGL Objects
lv_obj_t * scr_boot, * boot_label, * boot_bar, * boot_step_label;
lv_obj_t * scr_offline, * offline_led, * network_status_label, * mode_status_label, * action_bar_label, * action_bar, * joy_img_obj, * offline_card;
lv_obj_t * scr_online, * online_led, * online_mode_label, * drone_img_obj, * online_action_bar_label, * online_action_bar;
lv_obj_t * scr_connecting, * connecting_label;

// display flushing for LVGL
void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t *)&color_p->full, w * h, true);
    tft.endWrite();
    lv_disp_flush_ready(disp_drv);
}

// --- Haptics ---
volatile HapticPattern currentHaptic = HAPTIC_OFF;
volatile int hapticStep = 0;
volatile unsigned long nextHapticTime = 0;

void setHaptic(HapticPattern pattern) {
    currentHaptic = pattern;
    hapticStep = 0;
    nextHapticTime = 0;
}

void updateHaptics() {
    if (currentHaptic == HAPTIC_OFF) return;
    unsigned long now = millis();
    if (now < nextHapticTime) return;

    // simple state machines for vibration patterns
    switch (currentHaptic) {
        case HAPTIC_BUMP:
            if (hapticStep == 0) { analogWrite(MOTOR_PIN, 200); nextHapticTime = now + 60; hapticStep++; }
            else { analogWrite(MOTOR_PIN, 0); currentHaptic = HAPTIC_OFF; }
            break;
        case HAPTIC_SUCCESS:
            if (hapticStep == 0) { analogWrite(MOTOR_PIN, 255); nextHapticTime = now + 150; hapticStep++; }
            else { analogWrite(MOTOR_PIN, 0); currentHaptic = HAPTIC_OFF; }
            break;
        case HAPTIC_ERROR:
            if (hapticStep == 0) { analogWrite(MOTOR_PIN, 180); nextHapticTime = now + 80; hapticStep++; }
            else if (hapticStep == 1) { analogWrite(MOTOR_PIN, 0); nextHapticTime = now + 60; hapticStep++; }
            else if (hapticStep == 2) { analogWrite(MOTOR_PIN, 180); nextHapticTime = now + 80; hapticStep++; }
            else { analogWrite(MOTOR_PIN, 0); currentHaptic = HAPTIC_OFF; }
            break;
        case HAPTIC_HOLDING:
            if (hapticStep == 0) { analogWrite(MOTOR_PIN, 180); nextHapticTime = now + 40; hapticStep++; }
            else { analogWrite(MOTOR_PIN, 0); currentHaptic = HAPTIC_OFF; }
            break;
    }
}

// --- Cam Rendering Utilities ---
static uint16_t rotBuf[16 * 16];   
static uint16_t scaleBuf[16 * 16]; 

// takes a decoded jpeg block and scales/rotates it before drawing
bool tft_output_cam(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
    if (camLandscape) { 
        tft.pushImage(x, y, w, h, bitmap); 
        return true; 
    }
    
    // rotation logic for portrait view
    if ((uint32_t)w * h > (sizeof(rotBuf) / sizeof(rotBuf[0]))) return true;
    int rotW = h, rotH = w;
    for (int j = 0; j < h; j++) {
        for (int i = 0; i < w; i++) rotBuf[(w - 1 - i) * h + j] = bitmap[j * w + i];
    }
    int canvasX0 = y; 
    int canvasY0 = ROT_CANVAS_H - x - w;
    int dstW = (rotW * SCALE_NUM) / SCALE_DEN;
    int dstH = (rotH * SCALE_NUM) / SCALE_DEN;
    if (dstW < 1) dstW = 1; if (dstH < 1) dstH = 1;
    if ((uint32_t)dstW * dstH > (sizeof(scaleBuf) / sizeof(scaleBuf[0]))) return true;

    // scaling loop
    for (int dy = 0; dy < dstH; dy++) {
        int sy = (dy * SCALE_DEN) / SCALE_NUM;
        if (sy >= rotH) sy = rotH - 1;
        for (int dx = 0; dx < dstW; dx++) {
            int sx = (dx * SCALE_DEN) / SCALE_NUM;
            if (sx >= rotW) sx = rotW - 1;
            scaleBuf[dy * dstW + dx] = rotBuf[sy * rotW + sx];
        }
    }
    int destX = CAM_OFFSET_X + (canvasX0 * SCALE_NUM) / SCALE_DEN;
    int destY = CAM_OFFSET_Y + (canvasY0 * SCALE_NUM) / SCALE_DEN;
    tft.pushImage(destX, destY, dstW, dstH, scaleBuf);
    return true;
}

// parses raw analog values into a -1 to +1 float 
float getJoystickAxis(int raw, int minVal, int centerVal, int maxVal) {
    if (raw == centerVal) return 0.0f;
    float normalized = 0.0f;
    if (raw < centerVal) {
        int range = centerVal - minVal;
        if (range < 10) range = 2048;
        normalized = (float)(raw - centerVal) / range;
    } else {
        int range = maxVal - centerVal;
        if (range < 10) range = 2047;
        normalized = (float)(raw - centerVal) / range;
    }
    return constrain(normalized, -1.0f, 1.0f);
}

float applyDeadzone(float value, float deadzone) {
    if (abs(value) < deadzone) return 0.0f;
    if (value > 0) return (value - deadzone) / (1.0f - deadzone);
    return (value + deadzone) / (1.0f - deadzone);
}

// network stream parsing tools
bool readUntil(WiFiClient& client, const char* boundary, uint32_t timeoutMs = 3000) {
    size_t bLen = strlen(boundary);
    char ring[32] = {0};
    size_t ringLen = 0;
    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        if (client.available() > 0) {
            char c = client.read();
            if (ringLen < sizeof(ring) - 1) ring[ringLen++] = c;
            else { memmove(ring, ring + 1, sizeof(ring) - 2); ring[sizeof(ring) - 2] = c; }
            if (ringLen >= bLen && memcmp(ring + ringLen - bLen, boundary, bLen) == 0) return true;
        } else { yield(); } // yield to watchdog
    }
    return false;
}

int readContentLength(WiFiClient& client, uint32_t timeoutMs = 2000) {
    int contentLength = -1;
    unsigned long start = millis();
    char line[64]; size_t lineLen = 0;
    while (millis() - start < timeoutMs) {
        if (client.available()) {
            char c = client.read();
            if (c == '\n') {
                line[lineLen] = '\0';
                if (lineLen > 0 && line[lineLen - 1] == '\r') line[lineLen - 1] = '\0';
                if (strncasecmp(line, "Content-Length:", 15) == 0) contentLength = atoi(line + 15);
                if (lineLen == 0 && contentLength != -1) return contentLength;
                lineLen = 0;
            } else if (c != '\r') {
                if (lineLen < sizeof(line) - 1) line[lineLen++] = c;
            }
        } else { yield(); }
    }
    return -1;
}

// dims a color
uint16_t neonShade(uint8_t r, uint8_t g, uint8_t b, float brightness) {
    brightness = constrain(brightness, 0.0f, 1.0f);
    return tft.color565((uint8_t)(r * brightness), (uint8_t)(g * brightness), (uint8_t)(b * brightness));
}

// computes bounce animation for joystick release
void computeReleaseBounce(float rt, int &sinkY, float &squashScale) {
    rt = constrain(rt, 0.0f, 1.0f);
    float baseSink   = 6.0f * (1.0f - rt);
    float overshoot  = sinf(rt * PI) * (1.0f - rt) * 6.0f;
    sinkY = (int)(baseSink - overshoot);
    float squashBase = 0.68f + 0.32f * rt;
    float squashOver = sinf(rt * PI) * (1.0f - rt) * 0.3f;
    squashScale = squashBase + squashOver;
}

// draws over the live video feed
void drawCamOverlay() {
    const char* label = camLandscape ? "WIDE" : "FIT";
    uint16_t badgeColor = camLandscape ? TFT_ORANGE : TFT_CYAN;
    tft.fillRoundRect(4, 4, 46, 16, 4, TFT_BLACK);
    tft.drawRoundRect(4, 4, 46, 16, 4, badgeColor);
    tft.setTextColor(badgeColor, TFT_BLACK);
    tft.setTextSize(1); tft.setCursor(10, 8); tft.print(label);

    // fade out the instruction hint after 3 seconds
    unsigned long elapsed = millis() - camViewEnteredAt;
    if (elapsed < 3000) {
        float remain = 1.0f - (float)elapsed / 3000.0f;
        uint16_t col = neonShade(255, 255, 255, 0.35f + 0.65f * remain);
        tft.fillRect(56, 4, 210, 16, TFT_BLACK);
        tft.setTextColor(col, TFT_BLACK); tft.setCursor(60, 8);
        tft.print("TAP=WIDE/FIT   HOLD=DASHBOARD");
    }
}

// --- LVGL Setup Functions ---
lv_obj_t* build_header(lv_obj_t* parent, const char* title, lv_obj_t** outLed) {
    lv_obj_t* header = lv_obj_create(parent);
    lv_obj_set_size(header, 320, 36);
    lv_obj_align(header, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(header, lv_color_hex(COL_BG_ALT), 0);
    lv_obj_set_style_border_width(header, 1, 0);
    lv_obj_set_style_border_side(header, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_border_color(header, lv_color_hex(COL_BORDER), 0);
    lv_obj_set_style_radius(header, 0, 0);
    lv_obj_clear_flag(header, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* title_lbl = lv_label_create(header);
    lv_label_set_text(title_lbl, title);
    lv_obj_set_style_text_color(title_lbl, lv_color_hex(COL_CYAN), 0);
    lv_obj_align(title_lbl, LV_ALIGN_LEFT_MID, 10, 0);

    lv_obj_t* led = lv_obj_create(header);
    lv_obj_set_size(led, 10, 10);
    lv_obj_set_style_radius(led, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(led, 0, 0);
    lv_obj_align(led, LV_ALIGN_RIGHT_MID, -12, 0);
    lv_obj_clear_flag(led, LV_OBJ_FLAG_SCROLLABLE);

    if (outLed) *outLed = led;
    return header;
}

lv_obj_t* build_hintbar(lv_obj_t* parent, lv_obj_t** outLabel) {
    lv_obj_t* bar = lv_obj_create(parent);
    lv_obj_set_size(bar, 320, 30);
    lv_obj_align(bar, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(bar, lv_color_hex(COL_BG_ALT), 0);
    lv_obj_set_style_border_width(bar, 1, 0);
    lv_obj_set_style_border_side(bar, LV_BORDER_SIDE_TOP, 0);
    lv_obj_set_style_border_color(bar, lv_color_hex(COL_BORDER), 0);
    lv_obj_set_style_radius(bar, 0, 0);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* lbl = lv_label_create(bar);
    lv_obj_center(lbl);
    if (outLabel) *outLabel = lbl;
    return bar;
}

void led_anim_cb(void* var, int32_t v) { lv_obj_set_style_bg_opa((lv_obj_t*)var, (lv_opa_t)v, 0); }

void setLedState(lv_obj_t* led, bool connected) {
    if (connected) {
        if (lv_anim_get(led, led_anim_cb)) lv_anim_del(led, led_anim_cb);
        lv_obj_set_style_bg_color(led, lv_color_hex(COL_GREEN), 0);
        lv_obj_set_style_bg_opa(led, 255, 0);
    } else {
        lv_obj_set_style_bg_color(led, lv_color_hex(COL_RED), 0);
        // start pulse animation if disconnected
        if (!lv_anim_get(led, led_anim_cb)) {
            lv_anim_t a; lv_anim_init(&a); lv_anim_set_var(&a, led);
            lv_anim_set_exec_cb(&a, led_anim_cb); lv_anim_set_values(&a, 80, 255);
            lv_anim_set_time(&a, 500); lv_anim_set_playback_time(&a, 500);
            lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE); lv_anim_start(&a);
        }
    }
}

void build_boot_ui() {
    scr_boot = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_boot, lv_color_hex(COL_BG), 0);
    boot_step_label = lv_label_create(scr_boot);
    lv_obj_set_style_text_color(boot_step_label, lv_color_hex(COL_MUTED), 0);
    lv_label_set_text(boot_step_label, "STEP 1 / 3");
    lv_obj_align(boot_step_label, LV_ALIGN_CENTER, 0, -50);
    boot_label = lv_label_create(scr_boot);
    lv_obj_set_style_text_color(boot_label, lv_color_hex(COL_CYAN), 0);
    lv_label_set_text(boot_label, "HARDWARE TEST\nCalibrating Neutral Center");
    lv_obj_align(boot_label, LV_ALIGN_CENTER, 0, -20);
    boot_bar = lv_bar_create(scr_boot);
    lv_obj_set_size(boot_bar, 220, 12);
    lv_obj_align(boot_bar, LV_ALIGN_CENTER, 0, 30);
    lv_bar_set_range(boot_bar, 0, 100);
    lv_obj_set_style_bg_color(boot_bar, lv_color_hex(COL_CARD), LV_PART_MAIN);
    lv_obj_set_style_bg_color(boot_bar, lv_color_hex(COL_CYAN), LV_PART_INDICATOR);
}

void build_offline_ui() {
    scr_offline = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_offline, lv_color_hex(COL_BG), 0);
    build_header(scr_offline, "CONTROLLER", &offline_led);

    offline_card = lv_obj_create(scr_offline);
    lv_obj_set_size(offline_card, 150, 130);
    lv_obj_align(offline_card, LV_ALIGN_TOP_LEFT, 10, 44);
    lv_obj_set_style_bg_color(offline_card, lv_color_hex(COL_CARD), 0);
    lv_obj_set_style_border_width(offline_card, 1, 0);
    lv_obj_set_style_border_color(offline_card, lv_color_hex(COL_BORDER), 0);
    lv_obj_set_style_radius(offline_card, 12, 0);
    lv_obj_set_style_pad_all(offline_card, 10, 0);
    lv_obj_clear_flag(offline_card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t * title1 = lv_label_create(offline_card);
    lv_obj_set_style_text_color(title1, lv_color_hex(COL_MUTED), 0);
    lv_label_set_text(title1, "UPLINK"); lv_obj_align(title1, LV_ALIGN_TOP_LEFT, 0, 0);
    network_status_label = lv_label_create(offline_card); lv_obj_align(network_status_label, LV_ALIGN_TOP_LEFT, 0, 20);

    lv_obj_t * title2 = lv_label_create(offline_card);
    lv_obj_set_style_text_color(title2, lv_color_hex(COL_MUTED), 0);
    lv_label_set_text(title2, "MODE"); lv_obj_align(title2, LV_ALIGN_TOP_LEFT, 0, 65);
    mode_status_label = lv_label_create(offline_card); lv_obj_align(mode_status_label, LV_ALIGN_TOP_LEFT, 0, 85);

    joy_img_obj = lv_img_create(scr_offline);
    lv_img_set_src(joy_img_obj, &joy_img_dsc);
    lv_obj_align(joy_img_obj, LV_ALIGN_TOP_RIGHT, -15, 44);

    build_hintbar(scr_offline, &action_bar_label);
    action_bar = lv_obj_get_parent(action_bar_label);
}

void build_online_ui() {
    scr_online = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_online, lv_color_hex(COL_BG_ALT), 0);
    lv_obj_t* header = build_header(scr_online, "DRONE DASHBOARD", &online_led);
    online_mode_label = lv_label_create(header);
    lv_obj_align(online_mode_label, LV_ALIGN_RIGHT_MID, -30, 0);
    drone_img_obj = lv_img_create(scr_online);
    lv_img_set_src(drone_img_obj, &drone_img_dsc);
    lv_obj_align(drone_img_obj, LV_ALIGN_TOP_MID, 0, 44);
    build_hintbar(scr_online, &online_action_bar_label);
    online_action_bar = lv_obj_get_parent(online_action_bar_label);
}

void build_connecting_ui() {
    scr_connecting = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_connecting, lv_color_hex(COL_BG), 0);
    connecting_label = lv_label_create(scr_connecting);
    lv_obj_set_style_text_color(connecting_label, lv_color_hex(COL_CYAN), 0);
    lv_obj_align(connecting_label, LV_ALIGN_CENTER, 0, 0);
}

// --- Background Task (Core 0) ---
// Handles reading analog sticks and sending UDP packets continuously
void processInputs() {
    int switchState = digitalRead(MODE_SWITCH_PIN);
    if (switchState != lastSwitchState) {
        FlightMode newMode = (switchState == LOW) ? MODE_Z_PLANE : MODE_XY_PLANE;
        if (lastSwitchState != -1) {
            setHaptic(HAPTIC_BUMP);
            if (newMode != currentFlightMode) {
                transitionFromMode = currentFlightMode;
                modeTransitioning = true;
                modeTransitionStart = millis();
            }
        }
        lastSwitchState = switchState;
        currentFlightMode = newMode;
    }

    static int lastJoyX_raw = raw_joyX;
    int joyX = raw_joyX; int joyY = raw_joyY;
    int deltaX = joyX - lastJoyX_raw; lastJoyX_raw = joyX;

    float rawNx = getJoystickAxis(joyX, joyX_min, joyX_center, joyX_max);
    float rawNy = getJoystickAxis(joyY, joyY_min, joyY_center, joyY_max);

    // apply hardware deadzones to prevent drift
    float targetX = applyDeadzone(rawNx, 0.15f);
    float targetY = applyDeadzone(rawNy, 0.15f);

    if (INVERT_X) targetX = -targetX;
    if (INVERT_Y) targetY = -targetY;

    // Detect hard press (joystick button is integrated into the X-axis)
    // The potentiometer reads >4070 when pressed hard right quickly
    if (joyX > 4070 && deltaX > 300 && !isButtonHolding && !buttonCooldown) {
        isButtonHolding = true; buttonHoldStartTime = millis();
        shortHoldFired = false; longHoldFired = false; setHaptic(HAPTIC_BUMP);
    }
    else if (isButtonHolding) {
        unsigned long heldTime = millis() - buttonHoldStartTime;
        if (joyX < 3950) { // button released
            isButtonHolding = false; buttonCooldown = true;
            
            // if released quickly in cam view, toggle landscape
            if (!shortHoldFired && currentState == STATE_ONLINE && currentOnlineView == VIEW_CAMSTREAM) {
                camLandscape = !camLandscape; reqTftClear = true; setHaptic(HAPTIC_BUMP);
            } 
            // if released quickly in dashboard, switch to cam view
            else if (currentState == STATE_ONLINE && currentOnlineView == VIEW_DASHBOARD && shortHoldFired && !longHoldFired) {
                currentOnlineView = VIEW_CAMSTREAM; currentState = STATE_CONNECTING;
            }
        } else {
            // handle long holds
            if (!shortHoldFired && heldTime > HOLD_DURATION) {
                shortHoldFired = true; setHaptic(HAPTIC_SUCCESS);
                if (currentState == STATE_OFFLINE) {
                    currentState = STATE_CONNECTING; currentOnlineView = VIEW_DASHBOARD;
                    isButtonHolding = false; buttonCooldown = true;
                } else if (currentState == STATE_ONLINE && currentOnlineView == VIEW_CAMSTREAM) {
                    streamClient.stop(); currentOnlineView = VIEW_DASHBOARD;
                    isButtonHolding = false; buttonCooldown = true; reqLoadOnline = true;
                }
            }
            if (currentState == STATE_ONLINE && currentOnlineView == VIEW_DASHBOARD && !longHoldFired && heldTime > LONG_HOLD_DURATION) {
                longHoldFired = true; setHaptic(HAPTIC_ERROR); streamClient.stop();
                currentState = STATE_OFFLINE; isButtonHolding = false; buttonCooldown = true; reqLoadOffline = true;
            }
        }
    } else if (joyX < 3950) { buttonCooldown = false; }

    // freeze UI updates if holding button to avoid jitters
    if (isButtonHolding || (joyX > 4070 && deltaX > 300)) targetX = 0.0f;
    
    // smooth out the UI joystick visual
    uiCurrentX = uiCurrentX * 0.85f + targetX * 0.15f;
    uiCurrentY = uiCurrentY * 0.85f + targetY * 0.15f;

    // Send control packets to drone
    if (currentState == STATE_ONLINE && (millis() - lastControlSendTime > 20)) {
        lastControlSendTime = millis();
        
        int sendX = constrain((int)((targetX + 1.0f) * 2047.5f), 0, 4095);
        int sendY = constrain((int)((targetY + 1.0f) * 2047.5f), 0, 4095);

        // simplifying logic for flight mode transmission
        if (currentFlightMode == MODE_Z_PLANE) {
            // Force neutral X when in thrust mode to prevent yaw/roll issues on the drone
            sendX = 2048; 
        } 
        
        udpClient.beginPacket(host, controlPort);
        udpClient.printf("%d,%d,%d", (int)currentFlightMode, sendX, sendY);
        udpClient.endPacket();
    }
}

void controlLoopTask(void *parameter) {
    for (;;) {
        raw_joyX = analogRead(JOY_X_PIN); raw_joyY = analogRead(JOY_Y_PIN);
        if (currentState != STATE_BOOT_CHECK) processInputs();
        updateHaptics();
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

// --- Sprite Drawing ---
// renders the animated joystick on the offline screen
void drawJoystickHighlight() {
    joySprite.fillSprite(TFT_BLACK);
    static unsigned long initTime = 0; if (initTime == 0) initTime = millis();
    unsigned long runTime = millis() - initTime;
    static bool prevHolding = false; static unsigned long releaseStart = 0; static bool releasing = false;

    if (prevHolding && !isButtonHolding) { releasing = true; releaseStart = millis(); }
    prevHolding = isButtonHolding;
    if (releasing && millis() - releaseStart > 220) releasing = false;

    int sinkY = 0; float squashScale = 1.0f;
    bool isActive = false; bool orangeColor = false; float ringProgress = 0.0f;

    // calculate press animations
    if (isButtonHolding) {
        orangeColor = true; isActive = true;
        unsigned long holdTime = millis() - buttonHoldStartTime;
        ringProgress = (float)constrain(holdTime, 0UL, (unsigned long)HOLD_DURATION) / HOLD_DURATION;
        float pressPhase = constrain((float)holdTime / 150.0f, 0.0f, 1.0f);
        float pressEase = 1.0f - powf(1.0f - pressPhase, 3);
        sinkY = (int)(6 * pressEase); squashScale = 1.0f - 0.32f * pressEase;
    } else if (releasing) {
        float rt = (float)(millis() - releaseStart) / 220.0f;
        computeReleaseBounce(rt, sinkY, squashScale);
        isActive = (rt < 0.4f);
    }

    int sprCx = 60, sprCy = 60, trackX = 0, trackY = 0;
    
    // startup swirl animation
    if (runTime < 2500) {
        float phase = runTime / 2500.0f; float angle = runTime * 0.008f;
        float orbitRadius = 20.0f * (1.0f - powf(phase, 2));
        trackX = (int)(cosf(angle) * orbitRadius); trackY = (int)(sinf(angle) * orbitRadius);
        joySprite.fillCircle(sprCx + trackX * 1.5, sprCy + trackY * 1.5, 3, neonShade(0, 255, 255, 1.0f - phase));
        isActive = true;
    } else {
        trackX = (int)(uiCurrentX * 15.0f); trackY = (int)(uiCurrentY * 15.0f);
        if (abs(uiCurrentX) > 0.05f || abs(uiCurrentY) > 0.05f) isActive = true;
    }

    int knobCx = sprCx + trackX, knobCy = sprCy + sinkY + trackY;
    int radX = 24, radY = (int)(24 * squashScale);
    uint16_t primaryColor = orangeColor ? TFT_ORANGE : TFT_CYAN;
    uint16_t glowColor = orangeColor ? neonShade(255, 140, 0, 0.3f) : neonShade(0, 255, 255, 0.3f);

    // base rings
    joySprite.drawCircle(sprCx, sprCy, 45, tft.color565(30, 30, 30));
    joySprite.drawCircle(sprCx, sprCy, 46, tft.color565(20, 20, 20));

    if (isActive) {
        joySprite.fillEllipse(knobCx, knobCy, radX + 8, radY + 8, glowColor);
        joySprite.drawEllipse(knobCx, knobCy, radX + 12, radY + 12, primaryColor);
    }

    // knob body
    joySprite.fillEllipse(knobCx, knobCy, radX, radY, tft.color565(20, 20, 20));
    joySprite.drawEllipse(knobCx, knobCy, radX, radY, primaryColor);
    joySprite.fillCircle(knobCx, knobCy, 3, primaryColor);

    // hold progress ring
    if (isButtonHolding) {
        joySprite.drawCircle(sprCx, sprCy, 45, TFT_DARKGREY);
        int endAngle = (int)(ringProgress * 360.0f);
        for(int a = 0; a < endAngle; a+= 5) {
             float angle = (a - 90.0f) * DEG_TO_RAD;
             joySprite.fillCircle(sprCx + (int)(cosf(angle) * 45), sprCy + (int)(sinf(angle) * 45), 2, TFT_ORANGE);
        }
    }
}

// renders the XY view of the drone
void drawDroneTopView(TFT_eSprite &spr, int cx, int cy, float squashX) {
    static float spinPhase = 0.0f;
    auto sqx = [&](int px) { return cx + (int)((px - cx) * squashX); };

    // background grid
    spr.drawCircle(cx, cy, 60, tft.color565(24, 24, 28)); 
    spr.drawCircle(cx, cy, 40, tft.color565(20, 20, 24));
    spr.drawLine(sqx(cx - 60), cy, sqx(cx + 60), cy, tft.color565(20, 20, 24));
    spr.drawLine(sqx(cx), cy - 60, sqx(cx), cy + 60, tft.color565(20, 20, 24));

    float offsetX = uiCurrentX * 35.0f, offsetY = uiCurrentY * 35.0f;
    float bankAngle = uiCurrentX * 0.45f; // tilt based on stick pos
    float stickMag = sqrtf(uiCurrentX * uiCurrentX + uiCurrentY * uiCurrentY);
    spinPhase += 0.25f + stickMag * 1.5f;

    int dcx = cx + (int)offsetX, dcy = cy + (int)offsetY;
    
    // motion trail
    if (stickMag > 0.15f) {
        for (int t = 1; t <= 4; t++) {
            int tx = cx + (int)(offsetX * (1.0f - t * 0.2f)), ty = cy + (int)(offsetY * (1.0f - t * 0.2f));
            spr.fillCircle(sqx(tx), ty, 12 - t*2, neonShade(0, 200, 255, 0.4f - t*0.08f));
        }
    }

    // shadow
    int shadowX = dcx - (int)(uiCurrentX * 10), shadowY = dcy - (int)(uiCurrentY * 10) + 15;
    spr.drawLine(sqx(cx), cy, sqx(shadowX), shadowY, tft.color565(30, 30, 35));
    spr.fillCircle(sqx(shadowX), shadowY, 15, tft.color565(20, 20, 25));

    // drone arms and props
    float baseAngles[4] = { -0.785f, 0.785f, 2.356f, -2.356f };
    for (int i = 0; i < 4; i++) {
        float a = baseAngles[i] + bankAngle;
        int ax = dcx + (int)(cosf(a) * 52.0f), ay = dcy + (int)(sinf(a) * 52.0f * 0.55f);
        spr.drawLine(sqx(dcx), dcy, sqx(ax), ay, tft.color565(90, 90, 100));
        spr.fillCircle(sqx(ax), ay, 16, neonShade(0, 255, 255, 0.3f + stickMag * 0.4f));
        spr.fillCircle(sqx(ax), ay, 10, tft.color565(14, 14, 17));
        spr.drawCircle(sqx(ax), ay, 10, neonShade(0, 255, 255, 1.0f));
        
        // spinning blades
        float bladeA = spinPhase + i * 1.57f;
        int bx1 = ax + (int)(cosf(bladeA) * 12), by1 = ay + (int)(sinf(bladeA) * 12);
        int bx2 = ax - (int)(cosf(bladeA) * 12), by2 = ay - (int)(sinf(bladeA) * 12);
        spr.drawLine(sqx(bx1), by1, sqx(bx2), by2, TFT_WHITE);
    }
    
    // drone body
    spr.fillCircle(sqx(dcx), dcy, 16, neonShade(0, 200, 255, 0.3f));
    spr.fillCircle(sqx(dcx), dcy, 12, tft.color565(18, 18, 22));
    spr.drawCircle(sqx(dcx), dcy, 12, TFT_CYAN);
}

// renders the Z view of the drone
void drawDroneSideView(TFT_eSprite &spr, int cx, int cy, float squashX) {
    static float spinPhase = 0.0f;
    auto sqx = [&](int px) { return cx + (int)((px - cx) * squashX); };
    int groundY = cy + 65; 

    // background grid
    spr.drawLine(0, groundY, spr.width(), groundY, tft.color565(45, 45, 45));
    for (int r = 1; r <= 3; r++) spr.drawLine(sqx(cx - 120), groundY - r * 35, sqx(cx + 120), groundY - r * 35, tft.color565(22, 22, 25));

    int dcy = cy + 15 + (int)(-uiCurrentY * 65.0f); 
    float stickMag = fabsf(uiCurrentY) + fabsf(uiCurrentX) * 0.5f;
    spinPhase += 0.25f + stickMag * 1.5f;

    // shadow shrinks as altitude increases
    float shadowScale = constrain(1.0f - (float)(groundY - dcy) / 120.0f, 0.2f, 1.0f);
    spr.fillEllipse(sqx(cx), groundY, (int)(30 * shadowScale), (int)(8 * shadowScale), tft.color565(28, 28, 30));

    // vertical motion trail
    if (fabsf(uiCurrentY) > 0.15f) {
        for (int t = 1; t <= 3; t++) {
            spr.fillCircle(sqx(cx), dcy + (int)(uiCurrentY * t * 15), 14 - t*2, neonShade(255, 0, 200, 0.3f - t*0.08f));
        }
    }

    // draw sides
    for (int side = -1; side <= 1; side += 2) {
        int ax = cx + (int)(side * 45 * cosf(uiCurrentX * 0.45f));
        int ay = dcy - (int)(side * 45 * sinf(uiCurrentX * 0.45f));
        spr.drawLine(sqx(cx), dcy, sqx(ax), ay, tft.color565(90, 90, 100));
        
        // thrust exhaust visuals
        float thrust = constrain(-uiCurrentY, 0.0f, 1.0f); 
        if (thrust > 0.05f) {
            int glowLen = (int)(thrust * 25);
            for(int g=0; g<glowLen; g+=3) {
                spr.drawLine(sqx(ax - 8 + g/2), ay + 2 + g, sqx(ax + 8 - g/2), ay + 2 + g, neonShade(255, 0, 200, 0.4f * (1.0f - (float)g/glowLen)));
            }
        }
        
        // props
        float bladeOffset = sinf(spinPhase + side) * 5.0f;
        spr.drawLine(sqx(ax - 16), ay - (int)bladeOffset, sqx(ax + 16), ay + (int)bladeOffset, neonShade(255, 0, 200, 1.0f));
        spr.drawLine(sqx(ax - 16), ay, sqx(ax + 16), ay, tft.color565(65, 65, 70));
    }
    
    // drone body
    spr.fillCircle(sqx(cx), dcy, 16, neonShade(255, 0, 200, 0.25f));
    spr.fillCircle(sqx(cx), dcy, 12, tft.color565(18, 18, 22));
    spr.drawCircle(sqx(cx), dcy, 12, TFT_MAGENTA);
}

// --- Main State Machine Updaters ---
// runs once on startup to calibrate joystick center
void runBootCheck() {
    unsigned long now = millis();
    long jx = raw_joyX, jy = raw_joyY;

    if (bootCheckPhase == 0) {
        if (phaseStartTime == 0) { phaseStartTime = now; centerSumX = 0; centerSumY = 0; centerCount = 0; }
        
        // ignore wild readings
        if (jx < 4000) { centerSumX += jx; centerSumY += jy; centerCount++; }
        lv_bar_set_value(boot_bar, (now - phaseStartTime) / 20, LV_ANIM_OFF);

        if (now - phaseStartTime > 2000) {
            if (centerCount > 0) { joyX_center = centerSumX / centerCount; joyY_center = centerSumY / centerCount; }
            joyX_min = joyX_center; joyX_max = joyX_center; joyY_min = joyY_center; joyY_max = joyY_center;
            bootCheckPhase = 1; phaseStartTime = 0;
            lv_label_set_text(boot_step_label, "STEP 2 / 3");
            lv_label_set_text(boot_label, "HARDWARE INIT\nRotate Stick Fully");
            lv_obj_set_style_text_color(boot_label, lv_color_hex(COL_YELLOW), 0);
        }
    } else if (bootCheckPhase == 1) {
        if (phaseStartTime == 0) phaseStartTime = now;
        
        // track max/min extents
        if (jx < 4080) { if (jx < joyX_min) joyX_min = jx; if (jx > joyX_max) joyX_max = jx; }
        if (jy < 4080) { if (jy < joyY_min) joyY_min = jy; if (jy > joyY_max) joyY_max = jy; }
        lv_bar_set_value(boot_bar, (now - phaseStartTime) / 50, LV_ANIM_OFF);
        
        if (now - phaseStartTime > 5000) {
            bootCheckPhase = 2; phaseStartTime = 0;
            lv_label_set_text(boot_step_label, "STEP 3 / 3");
            lv_label_set_text(boot_label, "HARDWARE INIT\nPress Joystick Button");
            lv_obj_set_style_text_color(boot_label, lv_color_hex(COL_MAGENTA), 0);
        }
    } else if (bootCheckPhase == 2) {
        lv_bar_set_value(boot_bar, 100, LV_ANIM_OFF);
        if (jx > 4070) {
            // fallback if user didn't move stick enough
            if (joyX_center - joyX_min < 100) joyX_min = 0; if (joyX_max - joyX_center < 100) joyX_max = 4095;
            if (joyY_center - joyY_min < 100) joyY_min = 0; if (joyY_max - joyY_center < 100) joyY_max = 4095;
            setHaptic(HAPTIC_SUCCESS); currentState = STATE_OFFLINE; lv_scr_load(scr_offline);
        }
    }
}

void update_offline_ui() {
    bool connected = (WiFi.status() == WL_CONNECTED);
    setLedState(offline_led, connected);
    if (connected) {
        lv_label_set_text(network_status_label, "CONNECTED"); lv_obj_set_style_text_color(network_status_label, lv_color_hex(COL_GREEN), 0);
    } else {
        lv_label_set_text(network_status_label, "SEARCHING..."); lv_obj_set_style_text_color(network_status_label, lv_color_hex(COL_RED), 0);
    }
    lv_obj_set_style_border_color(offline_card, connected ? lv_color_hex(COL_GREEN) : lv_color_hex(COL_BORDER), 0);

    if (currentFlightMode == MODE_XY_PLANE) {
        lv_label_set_text(mode_status_label, "XY [HORZ]"); lv_obj_set_style_text_color(mode_status_label, lv_color_hex(COL_YELLOW), 0);
    } else {
        lv_label_set_text(mode_status_label, "Z  [VERT]"); lv_obj_set_style_text_color(mode_status_label, lv_color_hex(COL_MAGENTA), 0);
    }

    drawJoystickHighlight(); lv_obj_invalidate(joy_img_obj);

    if (isButtonHolding) {
        lv_obj_set_style_bg_color(action_bar, lv_color_hex(COL_ORANGE), 0);
        lv_label_set_text(action_bar_label, "INITIATING LINK..."); lv_obj_set_style_text_color(action_bar_label, lv_color_hex(0x000000), 0);
    } else {
        lv_obj_set_style_bg_color(action_bar, lv_color_hex(COL_BG_ALT), 0);
        lv_label_set_text(action_bar_label, "HOLD JOYSTICK TO LINK"); lv_obj_set_style_text_color(action_bar_label, lv_color_hex(COL_TEXT), 0);
    }
}

void update_online_dashboard() {
    setLedState(online_led, WiFi.status() == WL_CONNECTED);
    if (currentFlightMode == MODE_XY_PLANE) {
        lv_label_set_text(online_mode_label, "XY TOP VIEW"); lv_obj_set_style_text_color(online_mode_label, lv_color_hex(COL_YELLOW), 0);
    } else {
        lv_label_set_text(online_mode_label, "Z SIDE VIEW"); lv_obj_set_style_text_color(online_mode_label, lv_color_hex(COL_MAGENTA), 0);
    }

    FlightMode drawMode = currentFlightMode; float squashX = 1.0f;
    
    // handles 3D flip animation between drone modes
    if (modeTransitioning) {
        unsigned long elapsed = millis() - modeTransitionStart;
        if (elapsed >= MODE_TRANSITION_MS) { modeTransitioning = false; } 
        else {
            float t = (float)elapsed / MODE_TRANSITION_MS; 
            if (t < 0.5f) { drawMode = transitionFromMode; squashX = 1.0f - (t / 0.5f); } 
            else { drawMode = currentFlightMode; squashX = (t - 0.5f) / 0.5f; }
            if (squashX < 0.06f) squashX = 0.06f;  
        }
    }

    droneSprite.fillSprite(TFT_BLACK);
    int scx = droneSprite.width() / 2, scy = droneSprite.height() / 2;
    if (drawMode == MODE_XY_PLANE) drawDroneTopView(droneSprite, scx, scy, squashX);
    else drawDroneSideView(droneSprite, scx, scy, squashX);
    lv_obj_invalidate(drone_img_obj);

    if (isButtonHolding) {
        lv_obj_set_style_bg_color(online_action_bar, lv_color_hex(COL_ORANGE), 0);
        lv_label_set_text(online_action_bar_label, "PREPARING..."); lv_obj_set_style_text_color(online_action_bar_label, lv_color_hex(0x000000), 0);
    } else {
        lv_obj_set_style_bg_color(online_action_bar, lv_color_hex(COL_BG_ALT), 0);
        lv_label_set_text(online_action_bar_label, "HOLD: CAM | LONG HOLD: DISCONNECT"); lv_obj_set_style_text_color(online_action_bar_label, lv_color_hex(COL_TEXT), 0);
    }
}

// --- Connections ---
void connectToDashboard() {
    lv_label_set_text(connecting_label, "Linking Dashboard..."); lv_scr_load(scr_connecting); lv_timer_handler(); 
    delay(400); setHaptic(HAPTIC_SUCCESS); currentState = STATE_ONLINE; currentOnlineView = VIEW_DASHBOARD; lv_scr_load(scr_online);
}

void connectToStream() {
    lv_label_set_text(connecting_label, "Linking Video Stream..."); lv_scr_load(scr_connecting); lv_timer_handler(); 
    if (!streamClient.connect(host, streamPort)) {
        lv_label_set_text(connecting_label, "Connection Failed!"); lv_obj_set_style_text_color(connecting_label, lv_color_hex(COL_RED), 0);
        setHaptic(HAPTIC_ERROR); delay(1000); currentState = STATE_ONLINE; currentOnlineView = VIEW_DASHBOARD; lv_scr_load(scr_online); return;
    }
    
    streamClient.setNoDelay(true); streamClient.setTimeout(1000);
    streamClient.printf("GET %s HTTP/1.1\r\nHost: %s\r\nConnection: keep-alive\r\n\r\n", streamPath, host);

    // skip HTTP headers
    unsigned long start = millis(); char line[64]; size_t lineLen = 0;
    while (millis() - start < 5000) {
        if (streamClient.available()) {
            char c = streamClient.read();
            if (c == '\n') { if (lineLen == 0 || (lineLen == 1 && line[0] == '\r')) break; lineLen = 0; } 
            else if (c != '\r') { if (lineLen < sizeof(line) - 1) line[lineLen++] = c; }
        }
    }
    tft.fillScreen(TFT_BLACK); currentState = STATE_ONLINE; currentOnlineView = VIEW_CAMSTREAM; camViewEnteredAt = millis();
}

// parses the multipart MJPEG stream from the drone
void processVideoStream() {
    if (!streamClient.connected()) { setHaptic(HAPTIC_ERROR); currentOnlineView = VIEW_DASHBOARD; lv_scr_load(scr_online); return; }
    if (!readUntil(streamClient, "\r\n--", 3000)) return;
    int jpegSize = readContentLength(streamClient, 2000);
    if (jpegSize <= 0 || jpegSize > MAX_JPEG_SIZE) return;

    int bytesRead = 0; unsigned long start = millis();
    while (bytesRead < jpegSize && millis() - start < 2000) {
        int avail = streamClient.available();
        if (avail > 0) {
            int toRead = min(avail, jpegSize - bytesRead);
            int n = streamClient.read(jpegBuf + bytesRead, toRead);
            if (n > 0) bytesRead += n;
        } else { yield(); }
    }
    if (bytesRead == jpegSize) { TJpgDec.drawJpg(0, 0, jpegBuf, jpegSize); drawCamOverlay(); }
}

// --- Main Setup & Loop (Core 1) ---
void setup() {
    Serial.begin(115200);
    pinMode(MOTOR_PIN, OUTPUT); digitalWrite(MOTOR_PIN, LOW);
    pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
    analogReadResolution(12);

    if (psramFound()) jpegBuf = (uint8_t*)ps_malloc(MAX_JPEG_SIZE);
    else jpegBuf = (uint8_t*)malloc(MAX_JPEG_SIZE);

    tft.begin(); tft.invertDisplay(false); tft.setRotation(3); // landscape
    
    // Init sprites for fast drawing
    joySprite.setColorDepth(16); joySprite.createSprite(120, 120);
    droneSprite.setColorDepth(16); droneSprite.createSprite(300, 160);
    joySprite.setSwapBytes(false); droneSprite.setSwapBytes(false);

    // Map sprites to LVGL image descriptors so they can be drawn within LVGL screens
    joy_img_dsc.header.always_zero = 0; joy_img_dsc.header.w = 120; joy_img_dsc.header.h = 120;
    joy_img_dsc.data_size = 120 * 120 * 2; joy_img_dsc.header.cf = LV_IMG_CF_TRUE_COLOR; joy_img_dsc.data = (const uint8_t *)joySprite.getPointer();

    drone_img_dsc.header.always_zero = 0; drone_img_dsc.header.w = 300; drone_img_dsc.header.h = 160;
    drone_img_dsc.data_size = 300 * 160 * 2; drone_img_dsc.header.cf = LV_IMG_CF_TRUE_COLOR; drone_img_dsc.data = (const uint8_t *)droneSprite.getPointer();

    lv_init(); lv_disp_draw_buf_init(&draw_buf, buf, NULL, SCREEN_W * 20);
    static lv_disp_drv_t disp_drv; lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = SCREEN_W; disp_drv.ver_res = SCREEN_H;
    disp_drv.flush_cb = my_disp_flush; disp_drv.draw_buf = &draw_buf; lv_disp_drv_register(&disp_drv);

    build_boot_ui(); build_offline_ui(); build_online_ui(); build_connecting_ui();
    lv_scr_load(scr_boot);

    // apply network static IP fix
    WiFi.config(local_IP, gateway, subnet);
    WiFi.begin(ssid, password); WiFi.setSleep(false);

    // hook JPEG decoder to TFT for direct hardware rendering
    TJpgDec.setJpgScale(1); TJpgDec.setSwapBytes(true); TJpgDec.setCallback(tft_output_cam); 

    // offload input polling and UDP to background task
    xTaskCreatePinnedToCore(controlLoopTask, "ControlTask", 8192, NULL, 2, &ControlTaskHandle, 0);
}

void loop() {
    // process deferred screen change requests
    if (reqTftClear) { tft.fillScreen(TFT_BLACK); reqTftClear = false; }
    if (reqLoadOffline) { lv_scr_load(scr_offline); reqLoadOffline = false; }
    if (reqLoadOnline) { lv_scr_load(scr_online); reqLoadOnline = false; }

    // state machine router
    switch (currentState) {
        case STATE_BOOT_CHECK: runBootCheck(); break;
        case STATE_OFFLINE: if (millis() - lastUiUpdateTime > 40) { lastUiUpdateTime = millis(); update_offline_ui(); } break;
        case STATE_CONNECTING: if (currentOnlineView == VIEW_CAMSTREAM) { connectToStream(); return; } else { connectToDashboard(); } break;
        case STATE_ONLINE:
            if (currentOnlineView == VIEW_DASHBOARD) { if (millis() - lastUiUpdateTime > 33) { lastUiUpdateTime = millis(); update_online_dashboard(); } } 
            else { processVideoStream(); return; } break;
    }
    
    // tick LVGL engine
    static uint32_t last_tick = 0; uint32_t current_tick = millis();
    lv_tick_inc(current_tick - last_tick); last_tick = current_tick;
    lv_timer_handler(); delay(5);
}