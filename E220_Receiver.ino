// =============================================================================
//  E220_Receiver.ino  —  ESP32-S3
//  EBYTE E220-900T22D  —  comunicazione raw senza libreria
//
//  Mappa registri (da datasheet v1.2, pag.14-16):
//    00H = ADDH
//    01H = ADDL
//    02H = REG0: UART_BPS[7:5] | PARITY[4:3] | AIR_RATE[2:0]
//    03H = REG1: SUBPACKET[7:6] | RSSI_NOISE[5] | rsvd[4:2] | TX_PWR[1:0]
//    04H = REG2: CHANNEL (0-80, byte intero)
//    05H = REG3: RSSI_BYTE[7] | TX_MODE[6] | rsvd[5] | LBT[4] | rsvd[3] | WOR_CYCLE[2:0]
//    06H = CRYPT_H (write-only, lettura restituisce sempre 0)
//    07H = CRYPT_L (write-only, lettura restituisce sempre 0)
//
//  Connessioni:
//    ESP32-S3 GPIO17 → E220 RXD (pin 3)  [TX2]
//    ESP32-S3 GPIO18 → E220 TXD (pin 4)  [RX2]
//    ESP32-S3 GPIO4  → E220 M0  (pin 1)
//    ESP32-S3 GPIO5  → E220 M1  (pin 2)
//    ESP32-S3 GPIO6  → E220 AUX (pin 5)
//    ESP32-S3 3.3V   → E220 VCC (pin 6)
//    ESP32-S3 GND    → E220 GND (pin 7)
// =============================================================================


struct SensorPayload {
    uint8_t  nodeId;
    uint16_t batteria_mv;        // tensione cella in mV (MAX17048)
    uint8_t  batteria_soc;       // 0-100 % (MAX17048)
    int8_t   batteria_crate;     // %/h, +carica / -scarica (MAX17048)
    uint32_t timestamp_ms;
} __attribute__((packed));


// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                    ESP-NOW → CYD                                         ║
// ║  Inserisci la MAC del CYD (stampata sul suo monitor seriale al boot).    ║
// ╚═══════════════════════════════════════════════════════════════════════════╝
#include <esp_now.h>
#include <WiFi.h>

// MAC address del CYD — leggila dal monitor seriale del CYD al primo avvio
static const uint8_t CYD_MAC[6] = { 0x88, 0x13, 0xBF, 0x24, 0xB4, 0x14 };

// Struttura dati ESP-NOW — identica nel CYD
struct ESPNowData {
    uint16_t batteria_mv;
    uint8_t  batteria_soc;
    int8_t   batteria_crate;
    uint8_t  nodeId;
    int8_t   rssi;
    uint32_t packetCount;
    uint8_t  errorRate;
} __attribute__((packed));

static bool espnowReady = false;

void espnow_send_cb(const wifi_tx_info_t *info, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) Serial.println("[ESP-NOW] Invio OK");
    else Serial.println("[ESP-NOW] Invio FALLITO");
}

void initEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    Serial.print("[ESP-NOW] MAC: "); Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ERRORE] ESP-NOW init fallito");
        return;
    }
    esp_now_register_send_cb(espnow_send_cb);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, CYD_MAC, 6);
    peer.channel = 0;  // 0 = usa canale corrente
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("[ERRORE] ESP-NOW add peer fallito");
        return;
    }
    espnowReady = true;
    Serial.println("[OK] ESP-NOW pronto");
}

struct RxStats {
    uint32_t ok = 0, errori = 0, saltati = 0;
    int      last_rssi  = 0;
    uint32_t last_rx_ms = 0;
};
static RxStats stats;

void sendToDisplay(const SensorPayload &p, int rssi) {
    if (!espnowReady) return;
    ESPNowData d;
    d.batteria_mv    = p.batteria_mv;
    d.batteria_soc   = p.batteria_soc;
    d.batteria_crate = p.batteria_crate;
    d.nodeId         = p.nodeId;
    d.rssi           = (int8_t)rssi;
    d.packetCount    = stats.ok;
    uint32_t tot     = stats.ok + stats.errori;
    d.errorRate      = tot > 0 ? (uint8_t)((float)stats.errori / tot * 100.0f) : 0;
    esp_err_t result = esp_now_send(CYD_MAC, (uint8_t*)&d, sizeof(ESPNowData));
    if (result != ESP_OK) { Serial.print("[ESP-NOW] Send error: "); Serial.println(result); }
}


// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                      PIN — ESP32-S3                                      ║
// ╚═══════════════════════════════════════════════════════════════════════════╝
#define PIN_M0          4
#define PIN_M1          5
#define PIN_AUX         6
#define PIN_RX2         18
#define PIN_TX2         17

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                    PARAMETRI MODULO LORA                                 ║
// ║  Devono essere identici al trasmettitore per comunicare.                 ║
// ╚═══════════════════════════════════════════════════════════════════════════╝

// ── Indirizzo di questo nodo (0x0000–0xFFFF) ──────────────────────────────────
#define LORA_ADDR_HIGH      0x00
#define LORA_ADDR_LOW       0x02    // indirizzo RX

// ── Canale (addr 04H) — 0 fino a 80 ──────────────────────────────────────────
//    Frequenza effettiva = 850.125 + CHANNEL MHz
#define LORA_CHANNEL        23      // 873.125 MHz (factory default)

// ── UART baud rate (addr 02H bit[7:5]) ───────────────────────────────────────
//    DEVONO corrispondere tra loro
//    0x00=1200  0x01=2400  0x02=4800  0x03=9600 (default)
//    0x04=19200  0x05=38400  0x06=57600  0x07=115200
#define LORA_UART_BAUD      9600
#define LORA_UART_BPS       0x03

// ── UART parità (addr 02H bit[4:3]) ──────────────────────────────────────────
//    0x00=8N1 (default)  0x01=8O1  0x02=8E1
#define LORA_UART_PARITY    0x00

// ── Air data rate (addr 02H bit[2:0]) ────────────────────────────────────────
//    0x00=2.4k (default)  0x01=2.4k  0x02=2.4k  0x03=4.8k
//    0x04=9.6k  0x05=19.2k  0x06=38.4k  0x07=62.5k
#define LORA_AIR_DATA_RATE  0x02

// ── Sub-packet size (addr 03H bit[7:6]) ──────────────────────────────────────
//    0x00=200B (default)  0x01=128B  0x02=64B  0x03=32B
#define LORA_SUBPACKET_SIZE 0x00

// ── RSSI ambient noise (addr 03H bit[5]) ─────────────────────────────────────
//    0x00=disabilitato (default)  0x01=abilitato
#define LORA_RSSI_NOISE     0x00

// ── Potenza TX (addr 03H bit[1:0]) ───────────────────────────────────────────
//    0x00=22dBm (default)  0x01=17dBm  0x02=13dBm  0x03=10dBm
#define LORA_TX_POWER       0x00

// ── RSSI byte in coda al payload ricevuto (addr 05H bit[7]) ──────────────────
//    Se abilitato: RX vede [payload][1 byte RSSI]
//    PACKET_SIZE viene aggiornato automaticamente.
//    0x00=disabilitato (default)  0x01=abilitato
#define LORA_RSSI_BYTE      0x01

// ── Modalità trasmissione (addr 05H bit[6]) ───────────────────────────────────
//    0x00=trasparente (default)  0x01=fixed
#define LORA_TX_MODE        0x01    // fixed mode

// ── LBT Listen Before Talk (addr 05H bit[4]) ─────────────────────────────────
//    0x00=disabilitato (default)  0x01=abilitato
#define LORA_LBT            0x00

// ── WOR cycle (addr 05H bit[2:0]) ────────────────────────────────────────────
//    0x00=500ms (default)  0x01=1s  0x02=1.5s  ...  0x07=4s
#define LORA_WOR_CYCLE      0x00

// ── Chiave cifratura (addr 06H-07H, write-only) ───────────────────────────────
#define LORA_CRYPT_HIGH     0xA3    // stesso valore del trasmettitore
#define LORA_CRYPT_LOW      0x7F    // stesso valore del trasmettitore

// ── Timing ────────────────────────────────────────────────────────────────────
#define AUX_TIMEOUT_MS      5000
#define RX_PACKET_WAIT_MS   300
#define STATS_INTERVAL_MS   30000


// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                    CIFRATURA SOFTWARE AES-128 CTR                        ║
// ╚═══════════════════════════════════════════════════════════════════════════╝
#include "mbedtls/aes.h"

static const uint8_t AES_KEY[16] = {
    0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
    0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
};

void aesCtr(uint8_t* data, size_t len, uint32_t nonce) {
    mbedtls_aes_context ctx;
    mbedtls_aes_init(&ctx);
    mbedtls_aes_setkey_enc(&ctx, AES_KEY, 128);
    uint8_t nonceBlock[16] = {0};
    memcpy(nonceBlock, &nonce, 4);
    uint8_t streamBlock[16] = {0};
    size_t ncOff = 0;
    mbedtls_aes_crypt_ctr(&ctx, len, &ncOff, nonceBlock, streamBlock, data, data);
    mbedtls_aes_free(&ctx);
}

// =============================================================================
//  Struttura dati — DEVE essere identica al trasmettitore.
// =============================================================================

// Struttura pacchetto ricevuto:
// [nonce 4B] + [payload AES cifrato 19B] + [RSSI 1B se abilitato]
#define NONCE_SIZE      4
#define PAYLOAD_SIZE    sizeof(SensorPayload)
#define PACKET_SIZE     (NONCE_SIZE + PAYLOAD_SIZE + LORA_RSSI_BYTE)

// ── Build registri ────────────────────────────────────────────────────────────
#define BUILD_REG0  (((LORA_UART_BPS       & 0x07) << 5) | \
                     ((LORA_UART_PARITY    & 0x03) << 3) | \
                      (LORA_AIR_DATA_RATE  & 0x07))
#define BUILD_REG1  (((LORA_SUBPACKET_SIZE & 0x03) << 6) | \
                     ((LORA_RSSI_NOISE     & 0x01) << 5) | \
                      (LORA_TX_POWER       & 0x03))
#define BUILD_REG3  (((LORA_RSSI_BYTE  & 0x01) << 7) | \
                     ((LORA_TX_MODE    & 0x01) << 6) | \
                     ((LORA_LBT        & 0x01) << 4) | \
                      (LORA_WOR_CYCLE  & 0x07))

// ── Statistiche ───────────────────────────────────────────────────────────────

bool configureModule();
void exitConfigMode();
bool waitAux(uint32_t timeoutMs = AUX_TIMEOUT_MS);
void uartFlushRx();
void printPayload(const SensorPayload &p, int rssi);
void printStats();

// =============================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial.println("\n=== E220 Ricevitore ===");
    Serial.printf("Payload: %d byte  |  CHAN=%d (%.3fMHz)\n",
                  (int)PAYLOAD_SIZE, LORA_CHANNEL, 850.125f + LORA_CHANNEL);
    Serial.printf("REG0=0x%02X  REG1=0x%02X  REG3=0x%02X  mode=%s\n",
                  BUILD_REG0, BUILD_REG1, BUILD_REG3,
                  LORA_TX_MODE ? "FIXED" : "TRANSPARENT");
    Serial.printf("Indirizzo nodo: %02X%02X\n", LORA_ADDR_HIGH, LORA_ADDR_LOW);

    pinMode(PIN_M0, OUTPUT); pinMode(PIN_M1, OUTPUT); pinMode(PIN_AUX, INPUT);
    digitalWrite(PIN_M0, LOW); digitalWrite(PIN_M1, LOW);

    Serial2.begin(LORA_UART_BAUD, SERIAL_8N1, PIN_RX2, PIN_TX2);
    delay(500);

    if (!waitAux(3000)) {
        Serial.println("[ERRORE] AUX non HIGH!"); while (true) delay(1000);
    }
    Serial.println("[OK] Modulo pronto");
    initEspNow();

    if (!configureModule())
        Serial.println("[WARN] Configurazione fallita — uso parametri attuali");

    Serial.println("[OK] In ascolto...\n");
}

// =============================================================================
void loop() {
    static uint32_t lastStats = 0;
    if (millis() - lastStats >= STATS_INTERVAL_MS) {
        lastStats = millis(); printStats();
    }

    if (Serial2.available() <= 0) return;

    uint32_t ws = millis();
    while (Serial2.available() < (int)PACKET_SIZE && millis()-ws < RX_PACKET_WAIT_MS);

    int avail = Serial2.available();

    // Scarta eccesso — tieni solo l'ultimo pacchetto
    while (avail >= (int)(PACKET_SIZE * 2)) {
        uint8_t d[PACKET_SIZE]; Serial2.readBytes(d, PACKET_SIZE);
        stats.saltati++; avail = Serial2.available();
    }

    if (avail < (int)PACKET_SIZE) {
        Serial.printf("[WARN] %d byte (attesi %d)\n", avail, (int)PACKET_SIZE);
        while (Serial2.available()) Serial2.read();
        stats.errori++; return;
    }

    // Leggi nonce (4 byte) + payload cifrato (19 byte)
    uint32_t nonce = 0;
    Serial2.readBytes((uint8_t*)&nonce, NONCE_SIZE);

    uint8_t encrypted[PAYLOAD_SIZE];
    int n = Serial2.readBytes(encrypted, PAYLOAD_SIZE);

    // Leggi byte RSSI in coda se abilitato
    int rssi = 0;
    if (LORA_RSSI_BYTE && Serial2.available() >= 1) {
        uint8_t r = Serial2.read();
        rssi = -(int)(256 - r);
    }

    if (n != (int)PAYLOAD_SIZE) {
        Serial.printf("[ERRORE] Letti %d/%d byte\n", n, (int)PAYLOAD_SIZE);
        stats.errori++; return;
    }

    // Decifra con AES-128 CTR usando il nonce ricevuto
    aesCtr(encrypted, PAYLOAD_SIZE, nonce);
    SensorPayload payload;
    memcpy(&payload, encrypted, PAYLOAD_SIZE);

    // Validazione
    bool valido = payload.nodeId > 0
               && payload.batteria_mv > 0 && payload.batteria_mv < 5000
               && payload.batteria_soc <= 100;

    stats.ok++;
    stats.last_rssi  = rssi;
    stats.last_rx_ms = millis();

    printPayload(payload, rssi);
    if (!valido) Serial.println("         ↑ dati anomali");
    sendToDisplay(payload, rssi);
}

// =============================================================================
bool configureModule() {
    Serial.println("--- Configurazione E220 ---");

    digitalWrite(PIN_M0, HIGH); digitalWrite(PIN_M1, HIGH);
    delay(1000); waitAux(3000); delay(300);

    Serial2.end(); delay(100);
    Serial2.begin(LORA_UART_BAUD, SERIAL_8N1, PIN_RX2, PIN_TX2);
    delay(200); uartFlushRx();

    // Leggi 8 registri (0x00..0x07) — risposta 11 byte
    Serial2.write(0xC1); Serial2.write(0x00); Serial2.write(0x08); Serial2.flush();
    uint32_t t = millis();
    while (Serial2.available() < 11 && millis()-t < 3000); delay(100);

    if (Serial2.available() < 11) {
        Serial.printf("[ERRORE] Lettura: %d byte\n", Serial2.available());
        uartFlushRx(); exitConfigMode(); return false;
    }
    uint8_t resp[11]; Serial2.readBytes(resp, 11);
    if (resp[0]!=0xC1 || resp[1]!=0x00 || resp[2]!=0x08) {
        Serial.printf("[ERRORE] Header: %02X %02X %02X\n", resp[0], resp[1], resp[2]);
        exitConfigMode(); return false;
    }

    uint8_t reg[8]; memcpy(reg, &resp[3], 8);
    Serial.printf("[OK] Letto — ADDR=%02X%02X  REG0=%02X  REG1=%02X  CHAN=%d  REG3=%02X  CRYPT=%02X%02X\n",
                  reg[0], reg[1], reg[2], reg[3], reg[4], reg[5], reg[6], reg[7]);

    reg[0] = LORA_ADDR_HIGH;
    reg[1] = LORA_ADDR_LOW;
    reg[2] = BUILD_REG0;
    reg[3] = BUILD_REG1;
    reg[4] = LORA_CHANNEL;     // ← canale, byte intero 0-80
    reg[5] = BUILD_REG3;       // ← tx_mode, rssi_byte, lbt, wor
    reg[6] = LORA_CRYPT_HIGH;
    reg[7] = LORA_CRYPT_LOW;

    uartFlushRx();
    Serial2.write(0xC0); Serial2.write(0x00); Serial2.write(0x08);
    Serial2.write(reg, 8); Serial2.flush();
    t = millis();
    while (Serial2.available() < 11 && millis()-t < 3000); delay(100);
    uartFlushRx();

    // Verifica
    Serial2.write(0xC1); Serial2.write(0x00); Serial2.write(0x08); Serial2.flush();
    t = millis();
    while (Serial2.available() < 11 && millis()-t < 3000); delay(100);

    uint8_t v[11] = {0}; int n = Serial2.readBytes(v, 11);
    bool ok = (n==11)
           && (v[3]==LORA_ADDR_HIGH) && (v[4]==LORA_ADDR_LOW)
           && (v[5]==BUILD_REG0)
           && (v[7]==LORA_CHANNEL)
           && (v[8]==BUILD_REG3);

    if (ok)
        Serial.printf("[OK] Scritto — ADDR=%02X%02X  REG0=%02X  REG1=%02X  CHAN=%d (%.3fMHz)  REG3=%02X\n",
                      v[3], v[4], v[5], v[6], v[7], 850.125f+v[7], v[8]);
    else {
        Serial.println("[ERRORE] Verifica fallita:");
        Serial.printf("  Atteso   ADDR=%02X%02X REG0=%02X CHAN=%d REG3=%02X\n",
                      LORA_ADDR_HIGH, LORA_ADDR_LOW, BUILD_REG0, LORA_CHANNEL, BUILD_REG3);
        Serial.printf("  Ricevuto ADDR=%02X%02X REG0=%02X CHAN=%d REG3=%02X\n",
                      v[3], v[4], v[5], v[7], v[8]);
    }

    exitConfigMode(); return ok;
}

// =============================================================================
void exitConfigMode() {
    digitalWrite(PIN_M0, LOW); digitalWrite(PIN_M1, LOW); delay(200);
    Serial2.end(); delay(100);
    Serial2.begin(LORA_UART_BAUD, SERIAL_8N1, PIN_RX2, PIN_TX2);
    delay(200); waitAux(2000);
}

bool waitAux(uint32_t timeoutMs) {
    uint32_t start = millis();
    while (digitalRead(PIN_AUX)==LOW) {
        if (millis()-start > timeoutMs) return false; delay(5);
    }
    return true;
}

void uartFlushRx() { delay(10); while (Serial2.available()) Serial2.read(); }

void printPayload(const SensorPayload &p, int rssi) {
    Serial.printf("[RX #%lu] Node=%d | Batt=%umV (%u%%, %+d%%/h) | t=%lums",
                  stats.ok, p.nodeId, p.batteria_mv, p.batteria_soc,
                  p.batteria_crate, (unsigned long)p.timestamp_ms);
    if (LORA_RSSI_BYTE) Serial.printf(" | RSSI=%ddBm", rssi);
    Serial.println();
}

void printStats() {
    uint32_t tot = stats.ok + stats.errori;
    Serial.println("────────── STATISTICHE ──────────");
    Serial.printf("  OK: %lu  Errori: %lu (%.1f%%)  Saltati: %lu\n",
                  stats.ok, stats.errori,
                  tot > 0 ? (float)stats.errori/tot*100.0f : 0.0f, stats.saltati);
    if (LORA_RSSI_BYTE) Serial.printf("  Ultimo RSSI: %d dBm\n", stats.last_rssi);
    Serial.printf("  Silenzio: %lus\n",
                  stats.last_rx_ms > 0 ? (millis()-stats.last_rx_ms)/1000 : 0);
    Serial.println("─────────────────────────────────\n");
}
