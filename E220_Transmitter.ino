// =============================================================================
//  E220_Transmitter.ino  —  ESP32 classico
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
//    ESP32 GPIO17 TX2 → E220 RXD (pin 3)
//    ESP32 GPIO16 RX2 → E220 TXD (pin 4)
//    ESP32 GPIO4      → E220 M0  (pin 1)
//    ESP32 GPIO5      → E220 M1  (pin 2)
//    ESP32 GPIO2      → E220 AUX (pin 5)
//    ESP32 3.3V       → E220 VCC (pin 6)
//    ESP32 GND        → E220 GND (pin 7)
// =============================================================================

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                          PIN                                             ║
// ╚═══════════════════════════════════════════════════════════════════════════╝
#define PIN_M0          4
#define PIN_M1          5
#define PIN_AUX         2
#define PIN_RX2         16
#define PIN_TX2         17

// I2C per MAX17048 (fuel gauge batteria)
#define PIN_SDA         21
#define PIN_SCL         22

// UART1 + RS485 verso fototrappola (MAX485-class, DE+RE# legati)
#define PIN_RS485_RX    27
#define PIN_RS485_TX    26
#define PIN_RS485_DE    25
#define RS485_BAUD      9600

// OLED 128x64 SSD1306 — diagnostica locale (stesso bus I2C del MAX17048)
#define OLED_W          128
#define OLED_H          64
#define OLED_ADDR       0x3C

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                    PARAMETRI MODULO LORA                                 ║
// ║  Modificare solo questi valori. TX e RX devono condividere               ║
// ║  CHANNEL, AIR_DATA_RATE, UART_BAUD, TX_MODE, CRYPT.                     ║
// ╚═══════════════════════════════════════════════════════════════════════════╝

// ── Indirizzo di questo nodo (0x0000–0xFFFF) ──────────────────────────────────
//    0xFFFF = indirizzo broadcast/monitor
#define LORA_ADDR_HIGH      0x00
#define LORA_ADDR_LOW       0x01

// ── Indirizzo destinazione (nodo ricevitore) ──────────────────────────────────
#define DEST_ADDR_HIGH      0x00
#define DEST_ADDR_LOW       0x02

// ── Canale (addr 04H) — 0 fino a 80 ──────────────────────────────────────────
//    Frequenza effettiva = 850.125 + CHANNEL MHz
//    Es: 0=850.125  3=853.125  23=873.125 (factory default)  80=930.125
#define LORA_CHANNEL        23

// ── UART baud rate (addr 02H bit[7:5]) ───────────────────────────────────────
//    LORA_UART_BAUD = valore reale per Serial2.begin()
//    LORA_UART_BPS  = codice registro — DEVONO corrispondere
//    0x00=1200  0x01=2400  0x02=4800  0x03=9600 (default)
//    0x04=19200  0x05=38400  0x06=57600  0x07=115200
#define LORA_UART_BAUD      9600
#define LORA_UART_BPS       0x03

// ── UART parità (addr 02H bit[4:3]) ──────────────────────────────────────────
//    0x00=8N1 (default)  0x01=8O1  0x02=8E1
#define LORA_UART_PARITY    0x00

// ── Air data rate (addr 02H bit[2:0]) ────────────────────────────────────────
//    Più basso = più portata. Identico su TX e RX.
//    0x00=2.4k (default)  0x01=2.4k  0x02=2.4k  0x03=4.8k
//    0x04=9.6k  0x05=19.2k  0x06=38.4k  0x07=62.5k
#define LORA_AIR_DATA_RATE  0x02    // 2.4 kbps

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
//    0x00=disabilitato (default)  0x01=abilitato
#define LORA_RSSI_BYTE      0x01

// ── Modalità trasmissione (addr 05H bit[6]) ───────────────────────────────────
//    0x00=trasparente (default)  0x01=fixed
//    Fixed: TX antepone [DEST_H][DEST_L][CHAN] — il modulo instrada per addr
#define LORA_TX_MODE        0x01    // fixed mode

// ── LBT Listen Before Talk (addr 05H bit[4]) ─────────────────────────────────
//    0x00=disabilitato (default)  0x01=abilitato
#define LORA_LBT            0x00

// ── WOR cycle (addr 05H bit[2:0]) ────────────────────────────────────────────
//    0x00=500ms (default)  0x01=1s  0x02=1.5s  0x03=2s  ...  0x07=4s
#define LORA_WOR_CYCLE      0x00

// ── Chiave cifratura (addr 06H-07H, write-only) ───────────────────────────────
//    0x0000 = disabilitata
#define LORA_CRYPT_HIGH     0xA3    // primo byte chiave hardware modulo
#define LORA_CRYPT_LOW      0x7F    // secondo byte chiave hardware modulo

// ── Timing ────────────────────────────────────────────────────────────────────
#define TX_INTERVAL_MS      5000
#define AUX_TIMEOUT_MS      5000

// =============================================================================
//  Costruzione byte registri dai #define
//  REG0 (addr 02H): UART_BPS[7:5] | PARITY[4:3] | AIR_RATE[2:0]
//  REG1 (addr 03H): SUBPACKET[7:6] | RSSI_NOISE[5] | rsvd[4:2] | TX_PWR[1:0]
//  REG3 (addr 05H): RSSI_BYTE[7] | TX_MODE[6] | rsvd[5] | LBT[4] | rsvd[3] | WOR[2:0]
// =============================================================================
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


// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                    CIFRATURA SOFTWARE AES-128 CTR                        ║
// ║  Secondo livello su payload — combinato con CRYPT hardware del modulo.   ║
// ║  Chiave a 128 bit condivisa tra TX e RX (cambiare in produzione).        ║
// ╚═══════════════════════════════════════════════════════════════════════════╝
#include "mbedtls/aes.h"

// Chiave AES-128: 16 byte, DEVE essere identica su TX e RX
static const uint8_t AES_KEY[16] = {
    0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
    0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
};

// Cifra/decifra in-place con AES-128 CTR.
// nonce: contatore di pacchetto (unico per ogni trasmissione).
// CTR mode: encrypt == decrypt, nessun padding necessario.
void aesCtr(uint8_t* data, size_t len, uint32_t nonce) {
    mbedtls_aes_context ctx;
    mbedtls_aes_init(&ctx);
    mbedtls_aes_setkey_enc(&ctx, AES_KEY, 128);

    uint8_t nonceBlock[16] = {0};
    memcpy(nonceBlock, &nonce, 4);  // nonce nei primi 4 byte

    uint8_t streamBlock[16] = {0};
    size_t ncOff = 0;
    mbedtls_aes_crypt_ctr(&ctx, len, &ncOff, nonceBlock, streamBlock, data, data);
    mbedtls_aes_free(&ctx);
}

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                    FUEL GAUGE MAX17048                                   ║
// ║  Misura tensione, SOC% e tasso carica/scarica della batteria             ║
// ║  che alimenta TX + fototrappola. I2C 0x36.                               ║
// ╚═══════════════════════════════════════════════════════════════════════════╝
#include <Wire.h>
#include "Adafruit_MAX1704X.h"

static Adafruit_MAX17048 maxlipo;
static bool maxReady = false;

// I2C scanner — diagnostica al boot
void scanI2C() {
    Serial.println("[I2C] scan bus:");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  0x%02X\n", addr);
            found++;
        }
    }
    Serial.printf("[I2C] %d device trovati\n", found);
}

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                    OLED 0.96" 128x64 — STATO LOCALE                      ║
// ║  SSD1306 sullo stesso bus I2C del MAX17048.                              ║
// ╚═══════════════════════════════════════════════════════════════════════════╝
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

static Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire);
static bool oledReady = false;

void oled_init() {
    if (oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        oledReady = true;
        oled.clearDisplay();
        oled.setTextColor(SSD1306_WHITE);
        oled.setTextSize(1);
        oled.setCursor(0, 0);
        oled.println("LoRa TX");
        oled.println("avvio...");
        oled.display();
    }
}

void oled_render(uint16_t mv, uint8_t soc, int8_t crate,
                 uint8_t nodeId, uint32_t ok, uint32_t err) {
    if (!oledReady) return;
    char buf[24];
    oled.clearDisplay();

    snprintf(buf, sizeof(buf), "LoRa TX   N:%u", nodeId);
    oled.setCursor(0, 0);  oled.print(buf);
    oled.drawFastHLine(0, 10, OLED_W, SSD1306_WHITE);

    snprintf(buf, sizeof(buf), "Batt %u.%03u V", mv/1000, mv%1000);
    oled.setCursor(0, 14); oled.print(buf);

    snprintf(buf, sizeof(buf), "SOC  %u %%", soc);
    oled.setCursor(0, 24); oled.print(buf);

    snprintf(buf, sizeof(buf), "Rate %+d %%/h", crate);
    oled.setCursor(0, 34); oled.print(buf);

    oled.drawFastHLine(0, 44, OLED_W, SSD1306_WHITE);

    snprintf(buf, sizeof(buf), "TX:%lu Er:%lu",
             (unsigned long)ok, (unsigned long)err);
    oled.setCursor(0, 48); oled.print(buf);

    uint32_t s = millis()/1000;
    snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu",
             s/3600, (s%3600)/60, s%60);
    oled.setCursor(74, 56); oled.print(buf);

    oled.display();
}

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║                    RS485 (UART1) → FOTOTRAPPOLA                          ║
// ║  Half-duplex con controllo DE/RE manuale. Solo trasporto: il framing     ║
// ║  (start byte, CRC, protocollo) si aggiunge quando definito.              ║
// ╚═══════════════════════════════════════════════════════════════════════════╝
static HardwareSerial &RS485 = Serial1;

void rs485_init() {
    pinMode(PIN_RS485_DE, OUTPUT);
    digitalWrite(PIN_RS485_DE, LOW);   // default in ricezione
    RS485.begin(RS485_BAUD, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);
}

void rs485_send(const uint8_t *data, size_t len) {
    digitalWrite(PIN_RS485_DE, HIGH);  // abilita driver TX
    delayMicroseconds(50);             // settle linea
    RS485.write(data, len);
    RS485.flush();                     // attende fine shift-out ultimo bit
    delayMicroseconds(50);
    digitalWrite(PIN_RS485_DE, LOW);   // ritorno in RX
}

// =============================================================================
//  Struttura dati — __attribute__((packed)) = nessun padding
//  DEVE essere identica nel ricevitore.
// =============================================================================
struct SensorPayload {
    uint8_t  nodeId;
    uint16_t batteria_mv;        // tensione cella in mV (MAX17048)
    uint8_t  batteria_soc;       // 0-100 % (MAX17048)
    int8_t   batteria_crate;     // %/h, +carica / -scarica (MAX17048)
    uint32_t timestamp_ms;
} __attribute__((packed));

static uint32_t txCount  = 0;
static uint32_t txErrors = 0;

bool configureModule();
void exitConfigMode();
bool waitAux(uint32_t timeoutMs = AUX_TIMEOUT_MS);
bool sendPayload(const SensorPayload &p);
void printPayload(const SensorPayload &p);
void uartFlushRx();

// =============================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial.println("\n=== E220 Trasmettitore ===");
    Serial.printf("Payload: %d byte  |  CHAN=%d (%.3fMHz)\n",
                  (int)sizeof(SensorPayload), LORA_CHANNEL, 850.125f + LORA_CHANNEL);
    Serial.printf("REG0=0x%02X  REG1=0x%02X  REG3=0x%02X\n",
                  BUILD_REG0, BUILD_REG1, BUILD_REG3);
    Serial.printf("Nodo src=%02X%02X  dest=%02X%02X  mode=%s\n",
                  LORA_ADDR_HIGH, LORA_ADDR_LOW, DEST_ADDR_HIGH, DEST_ADDR_LOW,
                  LORA_TX_MODE ? "FIXED" : "TRANSPARENT");

    pinMode(PIN_M0, OUTPUT); pinMode(PIN_M1, OUTPUT); pinMode(PIN_AUX, INPUT);
    digitalWrite(PIN_M0, LOW); digitalWrite(PIN_M1, LOW);

    Wire.begin(PIN_SDA, PIN_SCL);
    delay(100);
    scanI2C();

    if (maxlipo.begin(&Wire)) {
        maxReady = true;
        Serial.printf("[OK] MAX17048 chip=0x%04X ver=0x%04X\n",
                      maxlipo.getChipID(), maxlipo.getICVersion());
    } else {
        Serial.println("[WARN] MAX17048 non trovato (I2C 0x36)");
    }

    oled_init();
    Serial.println(oledReady ? "[OK] OLED SSD1306"
                             : "[WARN] OLED non trovato (I2C 0x3C)");

    rs485_init();
    Serial.printf("[OK] RS485 UART1 @ %d baud (TX=%d RX=%d DE=%d)\n",
                  RS485_BAUD, PIN_RS485_TX, PIN_RS485_RX, PIN_RS485_DE);

    Serial2.begin(LORA_UART_BAUD, SERIAL_8N1, PIN_RX2, PIN_TX2);
    delay(500);

    if (!waitAux(3000)) {
        Serial.println("[ERRORE] AUX non HIGH!"); while (true) delay(1000);
    }
    Serial.println("[OK] Modulo pronto");

    if (!configureModule())
        Serial.println("[WARN] Configurazione fallita — uso parametri attuali");

    Serial.println("[OK] Inizio trasmissione\n");
}

// =============================================================================
void loop() {
    static uint32_t lastTx = 0;
    if (millis() - lastTx < TX_INTERVAL_MS) return;

    if (!waitAux(3000)) {
        Serial.println("[WARN] AUX non pronto, ciclo saltato");
        lastTx = millis(); return;
    }

    SensorPayload payload = {};
    payload.nodeId = 1;
    if (maxReady) {
        payload.batteria_mv    = (uint16_t)(maxlipo.cellVoltage() * 1000.0f);
        int soc   = (int)(maxlipo.cellPercent() + 0.5f);
        int crate = (int)maxlipo.chargeRate();
        payload.batteria_soc   = (uint8_t)constrain(soc, 0, 100);
        payload.batteria_crate = (int8_t)constrain(crate, -128, 127);
    }
    payload.timestamp_ms = millis();

    if (sendPayload(payload)) {
        txCount++;
        Serial.printf("[TX #%lu] ", txCount);
        printPayload(payload);
    } else {
        txErrors++;
        Serial.printf("[TX ERRORE] totale: %lu\n", txErrors);
    }

    oled_render(payload.batteria_mv, payload.batteria_soc,
                payload.batteria_crate, payload.nodeId,
                txCount, txErrors);

    lastTx = millis();
}

// =============================================================================
//  Fixed mode: invia [DEST_H][DEST_L][CHAN] + payload.
//  Il modulo instrada il pacchetto verso il nodo con quell'indirizzo e canale.
//  Il ricevitore in fixed mode vede solo il payload (header consumato dal modulo).
// =============================================================================
bool sendPayload(const SensorPayload &p) {
    // Cifra una copia del payload con AES-128 CTR
    // Il nonce è txCount: unico per ogni pacchetto.
    uint8_t encrypted[sizeof(SensorPayload)];
    memcpy(encrypted, &p, sizeof(SensorPayload));
    aesCtr(encrypted, sizeof(SensorPayload), txCount);

#if LORA_TX_MODE == 1
    Serial2.write((uint8_t)DEST_ADDR_HIGH);
    Serial2.write((uint8_t)DEST_ADDR_LOW);
    Serial2.write((uint8_t)LORA_CHANNEL);
#endif
    // Invia: [nonce 4 byte] + [payload cifrato]
    Serial2.write((const uint8_t*)&txCount, 4);
    Serial2.write(encrypted, sizeof(SensorPayload));
    Serial2.flush();
    return waitAux(AUX_TIMEOUT_MS);
}

// =============================================================================
bool configureModule() {
    Serial.println("--- Configurazione E220 ---");

    // Entra in config mode: M0=HIGH, M1=HIGH
    digitalWrite(PIN_M0, HIGH); digitalWrite(PIN_M1, HIGH);
    delay(1000); waitAux(3000); delay(300);

    Serial2.end(); delay(100);
    Serial2.begin(LORA_UART_BAUD, SERIAL_8N1, PIN_RX2, PIN_TX2);
    delay(200); uartFlushRx();

    // Leggi 8 registri: 0x00..0x07 (risposta = 3 header + 8 dati = 11 byte)
    Serial2.write(0xC1); Serial2.write(0x00); Serial2.write(0x08); Serial2.flush();
    uint32_t t = millis();
    while (Serial2.available() < 11 && millis()-t < 3000); delay(100);

    if (Serial2.available() < 11) {
        Serial.printf("[ERRORE] Lettura: %d byte (attesi 11)\n", Serial2.available());
        uartFlushRx(); exitConfigMode(); return false;
    }
    uint8_t resp[11]; Serial2.readBytes(resp, 11);
    if (resp[0]!=0xC1 || resp[1]!=0x00 || resp[2]!=0x08) {
        Serial.printf("[ERRORE] Header: %02X %02X %02X\n", resp[0], resp[1], resp[2]);
        exitConfigMode(); return false;
    }

    // resp[3..10] = registri 00H..07H
    uint8_t reg[8]; memcpy(reg, &resp[3], 8);
    Serial.printf("[OK] Letto — ADDR=%02X%02X  REG0=%02X  REG1=%02X  CHAN=%d  REG3=%02X  CRYPT=%02X%02X\n",
                  reg[0], reg[1], reg[2], reg[3], reg[4], reg[5], reg[6], reg[7]);
    // Nota: CRYPT (reg[6], reg[7]) è write-only → legge sempre 0x00

    // Imposta i nuovi valori
    reg[0] = LORA_ADDR_HIGH;   // 00H ADDH
    reg[1] = LORA_ADDR_LOW;    // 01H ADDL
    reg[2] = BUILD_REG0;       // 02H REG0: UART + parity + air rate
    reg[3] = BUILD_REG1;       // 03H REG1: subpacket + rssi_noise + tx_power
    reg[4] = LORA_CHANNEL;     // 04H REG2: canale (byte intero 0-80)
    reg[5] = BUILD_REG3;       // 05H REG3: rssi_byte | tx_mode | lbt | wor_cycle
    reg[6] = LORA_CRYPT_HIGH;  // 06H CRYPT_H (write-only)
    reg[7] = LORA_CRYPT_LOW;   // 07H CRYPT_L (write-only)

    // Scrivi e salva in flash (0xC0)
    uartFlushRx();
    Serial2.write(0xC0); Serial2.write(0x00); Serial2.write(0x08);
    Serial2.write(reg, 8); Serial2.flush();
    t = millis();
    while (Serial2.available() < 11 && millis()-t < 3000); delay(100);
    uartFlushRx();

    // Verifica rileggendo
    Serial2.write(0xC1); Serial2.write(0x00); Serial2.write(0x08); Serial2.flush();
    t = millis();
    while (Serial2.available() < 11 && millis()-t < 3000); delay(100);

    uint8_t v[11] = {0}; int n = Serial2.readBytes(v, 11);
    // v[3..10] = registri letti
    bool ok = (n==11)
           && (v[3]==LORA_ADDR_HIGH)
           && (v[4]==LORA_ADDR_LOW)
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

void printPayload(const SensorPayload &p) {
    Serial.printf("Node=%d  Batt=%umV (%u%%, %+d%%/h)  t=%lums\n",
                  p.nodeId, p.batteria_mv, p.batteria_soc,
                  p.batteria_crate, (unsigned long)p.timestamp_ms);
}
