// =============================================================================
//  CYD_Display.ino  —  ESP32-2432S028
//  LVGL 9.x + TFT_eSPI + XPT2046 + ESP-NOW
//  Rotazione 1, 320x240 landscape
// =============================================================================

#include <lvgl.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <Preferences.h>
#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>

#define TFT_BL_PIN   21
#define TFT_ROTATION  1
#define SCREEN_W    320
#define SCREEN_H    240

#define TOUCH_CS    33
#define TOUCH_CLK   25
#define TOUCH_MOSI  32
#define TOUCH_MISO  39
#define TOUCH_Z_MIN 200
#define HOLD_SAMPLES 30

// ── Colori ────────────────────────────────────────────────────────────────────
#define COL_BG        lv_color_hex(0x1A1A2E)
#define COL_CARD      lv_color_hex(0x16213E)
#define COL_HEADER    lv_color_hex(0x2D1B69)   // viola scuro
#define COL_TEMP      lv_color_hex(0xFF6B6B)
#define COL_HUM       lv_color_hex(0x4ECDC4)
#define COL_PRESS     lv_color_hex(0xA29BFE)
#define COL_BATT      lv_color_hex(0x6BCB77)
#define COL_WHITE     lv_color_hex(0xECECEC)
#define COL_GRAY      lv_color_hex(0x888888)

// =============================================================================
//  Struttura dati ESP-NOW
// =============================================================================
struct ESPNowData {
    float    temperatura;
    float    umidita;
    float    pressione;
    int16_t  batteria_mv;
    uint8_t  nodeId;
    int8_t   rssi;
    uint32_t packetCount;
    uint8_t  errorRate;
} __attribute__((packed));

static volatile bool  newData  = false;
static ESPNowData     rxData   = {};
static ESPNowData     dispData = {};

// ── LVGL widget handles ───────────────────────────────────────────────────────
static lv_obj_t *label_node, *label_rssi_txt, *bar_rssi;
static lv_obj_t *label_temp, *label_hum, *label_press;
static lv_obj_t *label_batt, *bar_batt;
static lv_obj_t *label_pkt, *label_err, *label_status, *label_uptime;

// ── Hardware ──────────────────────────────────────────────────────────────────
static TFT_eSPI            tft;
static SPIClass            touchSPI(VSPI);
static XPT2046_Touchscreen touch(TOUCH_CS, -1);
static lv_color_t          lvBuf[SCREEN_W * 10];
static lv_display_t       *lvDisp;
static lv_indev_t         *lvTouch;

// =============================================================================
//  Calibrazione touch 9 punti
// =============================================================================
#define CAL_POINTS 9
struct CalPoint { int sx, sy; int32_t rx, ry; };
struct Cal { CalPoint pts[CAL_POINTS]; bool valid; } cal;

void saveCalibration() {
    Preferences p; p.begin("tcal9", false);
    p.putBytes("cal", &cal, sizeof(cal)); p.end();
}
bool loadCalibration() {
    Preferences p; p.begin("tcal9", true);
    size_t n = p.getBytes("cal", &cal, sizeof(cal)); p.end();
    return n == sizeof(cal) && cal.valid;
}

void rawToScreen(int32_t rx, int32_t ry, int &sx, int &sy) {
    float rx_col[3], ry_row[3];
    for (int c=0; c<3; c++)
        rx_col[c] = (cal.pts[c].rx + cal.pts[c+3].rx + cal.pts[c+6].rx) / 3.0f;
    for (int r=0; r<3; r++)
        ry_row[r] = (cal.pts[r*3].ry + cal.pts[r*3+1].ry + cal.pts[r*3+2].ry) / 3.0f;

    int col = (rx >= rx_col[1]) ? 1 : 0;
    if (col > 1) col = 1;
    int row = (ry >= ry_row[1]) ? 1 : 0;
    if (row > 1) row = 1;

    CalPoint &p00=cal.pts[row*3+col],   &p10=cal.pts[row*3+col+1];
    CalPoint &p01=cal.pts[(row+1)*3+col],&p11=cal.pts[(row+1)*3+col+1];

    float tx = (rx_col[col+1]-rx_col[col])>0
               ? constrain((float)(rx-rx_col[col])/(rx_col[col+1]-rx_col[col]),0.0f,1.0f) : 0.5f;
    float ty = (ry_row[row+1]-ry_row[row])>0
               ? constrain((float)(ry-ry_row[row])/(ry_row[row+1]-ry_row[row]),0.0f,1.0f) : 0.5f;

    sx = constrain((int)((p00.sx*(1-tx)+p10.sx*tx)*(1-ty)+(p01.sx*(1-tx)+p11.sx*tx)*ty),0,SCREEN_W-1);
    sy = constrain((int)((p00.sy*(1-ty)+p01.sy*ty)*(1-tx)+(p10.sy*(1-ty)+p11.sy*ty)*tx),0,SCREEN_H-1);
}

void drawTarget(int x, int y, uint16_t col) {
    tft.fillCircle(x, y, 3, col);
    tft.drawCircle(x, y, 10, col);
    tft.drawLine(x-16,y,x-12,y,col); tft.drawLine(x+12,y,x+16,y,col);
    tft.drawLine(x,y-16,x,y-12,col); tft.drawLine(x,y+12,x,y+16,col);
}

void waitAndSampleTouch(int32_t &rx, int32_t &ry, int tsx, int tsy) {
    while (!touch.touched()) delay(10);
    delay(60);
    int64_t sumX=0, sumY=0; int count=0;
    while (count < HOLD_SAMPLES) {
        if (touch.touched()) {
            TS_Point p = touch.getPoint();
            if (p.z > TOUCH_Z_MIN) {
                sumX+=p.x; sumY+=p.y; count++;
                int r = map(count, 0, HOLD_SAMPLES, 0, 8);
                tft.fillCircle(tsx, tsy, r, count>=HOLD_SAMPLES ? TFT_GREEN : TFT_ORANGE);
            } else { sumX=0; sumY=0; count=0; drawTarget(tsx, tsy, TFT_RED); }
        } else { sumX=0; sumY=0; count=0; drawTarget(tsx, tsy, TFT_RED); }
        delay(25);
    }
    while (touch.touched()) delay(10);
    rx = sumX/HOLD_SAMPLES; ry = sumY/HOLD_SAMPLES;
}

void runCalibration() {
    const int M=20, CX=SCREEN_W/2, CY=SCREEN_H/2;
    int grid[CAL_POINTS][2] = {
        {M,M},{CX,M},{SCREEN_W-M,M},
        {M,CY},{CX,CY},{SCREEN_W-M,CY},
        {M,SCREEN_H-M},{CX,SCREEN_H-M},{SCREEN_W-M,SCREEN_H-M}
    };
    const char* names[CAL_POINTS] = {
        "TOP-LEFT","TOP-CENTER","TOP-RIGHT",
        "MID-LEFT","CENTER","MID-RIGHT",
        "BOT-LEFT","BOT-CENTER","BOT-RIGHT"
    };
    for (int i=0; i<CAL_POINTS; i++) { cal.pts[i].sx=grid[i][0]; cal.pts[i].sy=grid[i][1]; }

    for (int i=0; i<CAL_POINTS; i++) {
        tft.fillScreen(TFT_BLACK);
        int textY = M + (CY-M)/2 - 8;
        char buf[48];
        snprintf(buf, sizeof(buf), "Passo %d/%d — Tieni premuto", i+1, CAL_POINTS);
        tft.setTextColor(TFT_WHITE); tft.setTextSize(1);
        tft.setCursor((SCREEN_W - strlen(buf)*6)/2, textY); tft.print(buf);
        tft.setCursor((SCREEN_W - strlen(names[i])*6)/2, textY+14); tft.print(names[i]);

        for (int j=0; j<CAL_POINTS; j++) {
            if      (j < i) drawTarget(grid[j][0], grid[j][1], TFT_GREEN);
            else if (j > i) drawTarget(grid[j][0], grid[j][1], TFT_DARKGREY);
        }
        drawTarget(grid[i][0], grid[i][1], TFT_RED);

        int32_t rx, ry;
        waitAndSampleTouch(rx, ry, grid[i][0], grid[i][1]);
        cal.pts[i].rx=rx; cal.pts[i].ry=ry;
        drawTarget(grid[i][0], grid[i][1], TFT_GREEN);
        delay(400);
    }
    cal.valid = true;
    saveCalibration();

    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN); tft.setTextSize(2);
    tft.setCursor(50, 100); tft.print("Calibrazione OK!");
    delay(1500);
}

// =============================================================================
//  LVGL flush + touch
// =============================================================================
void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    uint32_t w = area->x2 - area->x1 + 1;
    uint32_t h = area->y2 - area->y1 + 1;
    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t*)px_map, w * h, true);
    tft.endWrite();
    lv_display_flush_ready(disp);
}

void lvgl_touch_cb(lv_indev_t *indev, lv_indev_data_t *data) {
    if (touch.touched()) {
        TS_Point p = touch.getPoint();
        if (p.z > TOUCH_Z_MIN) {
            int sx, sy; rawToScreen(p.x, p.y, sx, sy);
            data->point.x = sx; data->point.y = sy;
            data->state = LV_INDEV_STATE_PRESSED;
            return;
        }
    }
    data->state = LV_INDEV_STATE_RELEASED;
}

// =============================================================================
//  ESP-NOW
// =============================================================================
void espnow_recv_cb(const esp_now_recv_info_t *info,
                    const uint8_t *data, int len) {
    if (len != sizeof(ESPNowData)) return;
    memcpy(&rxData, data, sizeof(ESPNowData));
    newData = true;
}

// =============================================================================
//  UI helpers
// =============================================================================
lv_obj_t* makeCard(lv_obj_t *p, int x, int y, int w, int h, lv_color_t border) {
    lv_obj_t *c = lv_obj_create(p);
    lv_obj_set_pos(c, x, y); lv_obj_set_size(c, w, h);
    lv_obj_set_style_bg_color(c, COL_CARD, 0);
    lv_obj_set_style_border_color(c, border, 0);
    lv_obj_set_style_border_width(c, 2, 0);
    lv_obj_set_style_border_opa(c, LV_OPA_60, 0);
    lv_obj_set_style_radius(c, 6, 0);
    lv_obj_set_style_pad_all(c, 5, 0);
    lv_obj_clear_flag(c, LV_OBJ_FLAG_SCROLLABLE);
    return c;
}

lv_obj_t* makeLbl(lv_obj_t *p, const char *txt, const lv_font_t *f,
                  lv_color_t col, lv_align_t align, int ox, int oy) {
    lv_obj_t *l = lv_label_create(p);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, f, 0);
    lv_obj_set_style_text_color(l, col, 0);
    lv_obj_align(l, align, ox, oy);
    return l;
}

lv_color_t rssiCol(int8_t r) {
    if (r > -70)  return lv_color_hex(0x00E676);
    if (r > -85)  return lv_color_hex(0xFFD600);
    if (r > -100) return lv_color_hex(0xFF6D00);
    return        lv_color_hex(0xFF1744);
}

// =============================================================================
//  Build UI — 320x240 landscape, 2 colonne
// =============================================================================
void buildUI() {
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, COL_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    // Header h=36
    lv_obj_t *hdr = lv_obj_create(scr);
    lv_obj_set_pos(hdr, 0, 0); lv_obj_set_size(hdr, SCREEN_W, 36);
    lv_obj_set_style_bg_color(hdr, COL_HEADER, 0);
    lv_obj_set_style_radius(hdr, 0, 0);
    lv_obj_set_style_border_width(hdr, 0, 0);
    lv_obj_set_style_pad_all(hdr, 4, 0);
    lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

    makeLbl(hdr, "LoRa Monitor", &lv_font_montserrat_14, COL_WHITE, LV_ALIGN_LEFT_MID, 0, 0);
    label_node     = makeLbl(hdr, "Node: --", &lv_font_montserrat_14, COL_GRAY,  LV_ALIGN_CENTER,   0,   0);
    label_rssi_txt = makeLbl(hdr, "-- dBm",  &lv_font_montserrat_14, COL_GRAY,  LV_ALIGN_RIGHT_MID,-68, 0);

    bar_rssi = lv_bar_create(hdr);
    lv_obj_set_size(bar_rssi, 60, 6);
    lv_obj_align(bar_rssi, LV_ALIGN_RIGHT_MID, -4, 0);
    lv_bar_set_range(bar_rssi, 0, 100); lv_bar_set_value(bar_rssi, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(bar_rssi, lv_color_hex(0x334466), 0);
    lv_obj_set_style_bg_color(bar_rssi, lv_color_hex(0xFF1744), LV_PART_INDICATOR);

    // Cards 2x2 — y=38 h1=82 y2=122 h2=82 footer=206
    const int CX1=3, CX2=163, CW=154, RY1=38, CH1=82, RY2=122, CH2=82;

    lv_obj_t *cT = makeCard(scr, CX1, RY1, CW, CH1, COL_TEMP);
    makeLbl(cT, "TEMPERATURA", &lv_font_montserrat_14, COL_TEMP, LV_ALIGN_TOP_MID, 0, 0);
    label_temp = makeLbl(cT, "--", &lv_font_montserrat_24, COL_WHITE, LV_ALIGN_LEFT_MID, 0, 4);
    makeLbl(cT, "C", &lv_font_montserrat_14, COL_GRAY, LV_ALIGN_BOTTOM_RIGHT, 0, 0);

    lv_obj_t *cH = makeCard(scr, CX2, RY1, CW, CH1, COL_HUM);
    makeLbl(cH, "UMIDITA'", &lv_font_montserrat_14, COL_HUM, LV_ALIGN_TOP_MID, 0, 0);
    label_hum = makeLbl(cH, "--", &lv_font_montserrat_24, COL_WHITE, LV_ALIGN_LEFT_MID, 0, 4);
    makeLbl(cH, "%", &lv_font_montserrat_14, COL_GRAY, LV_ALIGN_BOTTOM_RIGHT, 0, 0);

    lv_obj_t *cP = makeCard(scr, CX1, RY2, CW, CH2, COL_PRESS);
    makeLbl(cP, "PRESSIONE", &lv_font_montserrat_14, COL_PRESS, LV_ALIGN_TOP_MID, 0, 0);
    label_press = makeLbl(cP, "--", &lv_font_montserrat_24, COL_WHITE, LV_ALIGN_LEFT_MID, 0, 4);
    makeLbl(cP, "hPa", &lv_font_montserrat_14, COL_GRAY, LV_ALIGN_BOTTOM_RIGHT, 0, 0);

    lv_obj_t *cB = makeCard(scr, CX2, RY2, CW, CH2, COL_BATT);
    makeLbl(cB, "BATTERIA", &lv_font_montserrat_14, COL_BATT, LV_ALIGN_TOP_MID, 0, 0);
    label_batt = makeLbl(cB, "---- mV", &lv_font_montserrat_24, COL_WHITE, LV_ALIGN_LEFT_MID, 0, 4);
    bar_batt = lv_bar_create(cB);
    lv_obj_set_size(bar_batt, CW-12, 6);
    lv_obj_align(bar_batt, LV_ALIGN_BOTTOM_MID, 0, -2);
    lv_bar_set_range(bar_batt, 3000, 4200); lv_bar_set_value(bar_batt, 3000, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(bar_batt, lv_color_hex(0x334466), 0);
    lv_obj_set_style_bg_color(bar_batt, COL_BATT, LV_PART_INDICATOR);

    // Footer h=34
    lv_obj_t *foot = lv_obj_create(scr);
    lv_obj_set_pos(foot, 0, 206); lv_obj_set_size(foot, SCREEN_W, 34);
    lv_obj_set_style_bg_color(foot, COL_HEADER, 0);
    lv_obj_set_style_radius(foot, 0, 0);
    lv_obj_set_style_border_width(foot, 0, 0);
    lv_obj_set_style_pad_hor(foot, 6, 0);
    lv_obj_set_style_pad_ver(foot, 4, 0);
    lv_obj_clear_flag(foot, LV_OBJ_FLAG_SCROLLABLE);

    label_pkt    = makeLbl(foot, "PKT:--",       &lv_font_montserrat_14, COL_WHITE, LV_ALIGN_LEFT_MID,  0,  0);
    label_err    = makeLbl(foot, "ERR:--%",      &lv_font_montserrat_14, COL_GRAY,  LV_ALIGN_LEFT_MID,  70, 0);
    label_status = makeLbl(foot, "In attesa...", &lv_font_montserrat_14, COL_GRAY,  LV_ALIGN_CENTER,    0,  0);
    label_uptime = makeLbl(foot, "00:00:00",     &lv_font_montserrat_14, COL_GRAY,  LV_ALIGN_RIGHT_MID, 0,  0);
}

// =============================================================================
void updateUI(const ESPNowData &d) {
    char buf[16];

    snprintf(buf, sizeof(buf), "Node: %d", d.nodeId);
    lv_label_set_text(label_node, buf);

    snprintf(buf, sizeof(buf), "%d dBm", (int)d.rssi);
    lv_label_set_text(label_rssi_txt, buf);
    lv_obj_set_style_text_color(label_rssi_txt, rssiCol(d.rssi), 0);
    lv_bar_set_value(bar_rssi, map(constrain((int)d.rssi,-110,-30),-110,-30,0,100), LV_ANIM_ON);
    lv_obj_set_style_bg_color(bar_rssi, rssiCol(d.rssi), LV_PART_INDICATOR);

    int t_int=(int)d.temperatura, t_dec=abs((int)(d.temperatura*10)%10);
    int h_int=(int)d.umidita,     h_dec=abs((int)(d.umidita*10)%10);
    snprintf(buf, sizeof(buf), "%d.%d", t_int, t_dec); lv_label_set_text(label_temp,  buf);
    snprintf(buf, sizeof(buf), "%d.%d", h_int, h_dec); lv_label_set_text(label_hum,   buf);
    snprintf(buf, sizeof(buf), "%d",    (int)d.pressione); lv_label_set_text(label_press, buf);
    snprintf(buf, sizeof(buf), "%d mV", d.batteria_mv);    lv_label_set_text(label_batt,  buf);

    lv_bar_set_value(bar_batt, d.batteria_mv, LV_ANIM_ON);
    lv_color_t bc = (d.batteria_mv>3700)?COL_BATT:(d.batteria_mv>3400)?lv_color_hex(0xFFD600):lv_color_hex(0xFF1744);
    lv_obj_set_style_bg_color(bar_batt, bc, LV_PART_INDICATOR);

    snprintf(buf, sizeof(buf), "PKT:%lu", (unsigned long)d.packetCount);
    lv_label_set_text(label_pkt, buf);
    snprintf(buf, sizeof(buf), "ERR:%d%%", (int)d.errorRate);
    lv_label_set_text(label_err, buf);
    lv_label_set_text(label_status, "RX OK");
    lv_obj_set_style_text_color(label_status, lv_color_hex(0x00E676), 0);
}

// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== CYD Display ===");

    pinMode(0, INPUT_PULLUP);
    if (digitalRead(0) == LOW) {
        Preferences p; p.begin("tcal9", false); p.clear(); p.end();
        Serial.println("[INFO] NVS calibrazione cancellata");
        delay(2000);
    }

    pinMode(TFT_BL_PIN, OUTPUT);
    digitalWrite(TFT_BL_PIN, HIGH);
    tft.begin();
    tft.setRotation(TFT_ROTATION);
    tft.fillScreen(TFT_BLACK);

    touchSPI.begin(TOUCH_CLK, TOUCH_MISO, TOUCH_MOSI, TOUCH_CS);
    touch.begin(touchSPI);

    if (loadCalibration()) {
        Serial.println("[OK] Calibrazione touch caricata");
    } else {
        runCalibration();
    }

    lv_init();
    lvDisp = lv_display_create(SCREEN_W, SCREEN_H);
    lv_display_set_flush_cb(lvDisp, lvgl_flush_cb);
    lv_display_set_buffers(lvDisp, lvBuf, NULL, sizeof(lvBuf), LV_DISPLAY_RENDER_MODE_PARTIAL);

    lvTouch = lv_indev_create();
    lv_indev_set_type(lvTouch, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(lvTouch, lvgl_touch_cb);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    Serial.printf("[OK] MAC CYD: %s\n", WiFi.macAddress().c_str());
    Serial.println("     <- Inserisci in CYD_MAC nel ricevitore ESP32-S3");

    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb(espnow_recv_cb);
        Serial.println("[OK] ESP-NOW in ascolto");
    } else {
        Serial.println("[ERRORE] ESP-NOW init");
    }

    buildUI();
    lv_timer_handler();
    Serial.println("[OK] Display pronto");
}

// =============================================================================
void loop() {
    if (newData) {
        newData = false;
        memcpy(&dispData, &rxData, sizeof(ESPNowData));
        updateUI(dispData);
    }

    static uint32_t lastSec = 0;
    if (millis() - lastSec >= 1000) {
        lastSec = millis();
        uint32_t s = millis()/1000;
        char buf[16];
        snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", s/3600, (s%3600)/60, s%60);
        lv_label_set_text(label_uptime, buf);
    }

    lv_timer_handler();
    delay(5);
}
