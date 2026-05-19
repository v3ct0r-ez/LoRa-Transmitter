// =============================================================================
//  E220_FactoryRestore.ino
//  Ripristina i valori di fabbrica esatti dal datasheet:
//  ADDH=0x00 ADDL=0x00 REG0=0x62 REG1=0x00 CHAN=0x17 REG3=0x00 CRYPT=0x0000
//  → 9600 baud, 8N1, 2.4kbps, 22dBm, canale 23 (873.125MHz), trasparente
//
//  Carica su un modulo alla volta. Per ESP32-S3: cambia AUX=6, RX2=18.
// =============================================================================
#define PIN_M0   4
#define PIN_M1   5
#define PIN_AUX  2    // ESP32-S3: cambia in 6
#define PIN_RX2  16   // ESP32-S3: cambia in 18
#define PIN_TX2  17

// Valori factory dal datasheet (tabella 7-3, pag.16)
// 00H  01H  02H  03H  04H   05H  06H  07H
// ADDH ADDL REG0 REG1 CHAN  REG3 CRH  CRL
const uint8_t FACTORY[8] = { 0x00, 0x00, 0x62, 0x00, 0x17, 0x00, 0x00, 0x00 };

bool waitAux(uint32_t ms = 3000) {
    uint32_t s = millis();
    while (digitalRead(PIN_AUX)==LOW) { if(millis()-s>ms) return false; delay(5); }
    return true;
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial.println("\n=== E220 Factory Restore ===");
    Serial.println("Ripristino: ADDH=00 ADDL=00 REG0=62 REG1=00 CHAN=23(873MHz) REG3=00");

    pinMode(PIN_M0, OUTPUT); pinMode(PIN_M1, OUTPUT); pinMode(PIN_AUX, INPUT);
    digitalWrite(PIN_M0, HIGH); digitalWrite(PIN_M1, HIGH);
    delay(1000); waitAux(3000); delay(300);

    Serial2.begin(9600, SERIAL_8N1, PIN_RX2, PIN_TX2);
    delay(200);
    while (Serial2.available()) Serial2.read();

    // Leggi stato attuale
    Serial2.write(0xC1); Serial2.write(0x00); Serial2.write(0x08); Serial2.flush();
    uint32_t t = millis();
    while (Serial2.available() < 11 && millis()-t < 3000); delay(100);
    if (Serial2.available() >= 11) {
        uint8_t r[11]; Serial2.readBytes(r, 11);
        Serial.printf("Attuale: ADDR=%02X%02X REG0=%02X REG1=%02X CHAN=%d REG3=%02X CRYPT=%02X%02X\n",
                      r[3], r[4], r[5], r[6], r[7], r[8], r[9], r[10]);
    }
    while (Serial2.available()) Serial2.read();

    // Scrivi factory
    Serial2.write(0xC0); Serial2.write(0x00); Serial2.write(0x08);
    Serial2.write(FACTORY, 8); Serial2.flush();
    t = millis();
    while (Serial2.available() < 11 && millis()-t < 3000); delay(100);
    while (Serial2.available()) Serial2.read();

    // Verifica
    Serial2.write(0xC1); Serial2.write(0x00); Serial2.write(0x08); Serial2.flush();
    t = millis();
    while (Serial2.available() < 11 && millis()-t < 3000); delay(100);

    if (Serial2.available() >= 11) {
        uint8_t v[11]; Serial2.readBytes(v, 11);
        bool ok = (v[3]==0x00 && v[4]==0x00 && v[5]==0x62 && v[7]==0x17 && v[8]==0x00);
        Serial.printf("Verificato: ADDR=%02X%02X REG0=%02X REG1=%02X CHAN=%d REG3=%02X\n",
                      v[3], v[4], v[5], v[6], v[7], v[8]);
        Serial.println(ok ? "[OK] Factory restore completato!" : "[ERRORE] Verifica fallita");
    } else {
        Serial.println("[ERRORE] Nessuna risposta");
    }

    digitalWrite(PIN_M0, LOW); digitalWrite(PIN_M1, LOW);
    Serial2.end();
}

void loop() {}
