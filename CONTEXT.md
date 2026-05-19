# Progetto: Sistema IoT LoRa con Display CYD

## Architettura
```
ESP32 classico (TX) ──LoRa E220──► ESP32-S3 (RX) ──ESP-NOW──► CYD ESP32-2432S028 (Display)
```

---

## File del progetto

| File | Dispositivo | Stato |
|------|-------------|-------|
| `E220_Transmitter.ino` | ESP32 classico | funzionante |
| `E220_Receiver.ino` | ESP32-S3 | funzionante |
| `E220_FactoryRestore.ino` | Entrambi | utility |
| `CYD_Display.ino` | ESP32-2432S028 | funzionante |
| `Touch_Cal.ino` | CYD | utility calibrazione |
| `User_Setup.h` | CYD (TFT_eSPI) | configurato |

---

## Modulo LoRa E220-900T22D — Mappa registri (datasheet v1.2)

| Addr | Nome | Contenuto |
|------|------|-----------|
| 00H | ADDH | indirizzo alto nodo |
| 01H | ADDL | indirizzo basso nodo |
| 02H | REG0 | UART_BPS[7:5] + PARITY[4:3] + AIR_RATE[2:0] |
| 03H | REG1 | SUBPACKET[7:6] + RSSI_NOISE[5] + rsvd + TX_PWR[1:0] |
| 04H | CHAN | canale 0-80 intero, freq = 850.125 + CHAN MHz |
| 05H | REG3 | RSSI_BYTE[7] + TX_MODE[6] + rsvd[5] + LBT[4] + rsvd[3] + WOR[2:0] |
| 06H | CRYPT_H | write-only, lettura sempre 0 |
| 07H | CRYPT_L | write-only, lettura sempre 0 |

Config mode: M0=HIGH M1=HIGH
Factory default: ADDR=0000 REG0=0x62 REG1=0x00 CHAN=23 REG3=0x00 CRYPT=0x0000
Protocollo lettura: C1 addr len -> C1 addr len data
Protocollo scrittura flash: C0 addr len data

---

## Parametri LoRa configurati

CHANNEL=23 (873.125MHz), UART=9600, AIR_RATE=2.4kbps
TX_MODE=fixed (0x01), RSSI_BYTE=abilitato
BUILD_REG0=0x62, BUILD_REG1=0x00, BUILD_REG3=0xC0
AES_KEY: 2B7E151628AED2A6ABF7158809CF4F3C (mbedtls CTR, nonce=txCount)

---

## Strutture dati

SensorPayload (9B, TX->RX via LoRa):
  nodeId(1) + batteria_mv(2) + batteria_soc(1) + batteria_crate(1) + timestamp_ms(4)

Pacchetto trasmesso (fixed mode):
  [DEST_H][DEST_L][CHAN] header routing consumato dal modulo
  [nonce 4B][payload AES cifrato 9B][RSSI 1B] = 14 byte all'RX

ESPNowData (11B, RX->CYD via ESP-NOW):
  batteria_mv(2) + batteria_soc(1) + batteria_crate(1) +
  nodeId(1) + rssi(1) + packetCount(4) + errorRate(1)

I sensori reali (temp/umid/press) verranno aggiunti in futuro quando
disponibili — per ora il payload trasporta solo telemetria batteria.

---

## Pin E220

TX (ESP32 classico): TX2=17, RX2=16, M0=4, M1=5, AUX=2
RX (ESP32-S3):       TX2=17, RX2=18, M0=4, M1=5, AUX=6

## MAX17048 fuel gauge (TX)

I2C su ESP32 classico: SDA=21, SCL=22 (default Wire)
Indirizzo: 0x36
Libreria: Adafruit_MAX1704X (Adafruit_MAX17048 maxlipo)
Letture: cellVoltage() V, cellPercent() %, chargeRate() %/h
Alimentato dalla stessa cella che monitora (TX + fototrappola).
Se non rilevato al boot: maxReady=false, campi batteria a 0.

---

## CYD ESP32-2432S028

User_Setup.h:
  ILI9341_2_DRIVER (non ILI9341_DRIVER!)
  USE_HSPI_PORT, TFT_INVERSION_ON
  MISO=12, MOSI=13, SCLK=14, CS=15, DC=2, RST=-1, BL=21
  SPI_FREQUENCY=40000000

Rotazione: setRotation(1) -> 320x240 landscape
Touch XPT2046 su VSPI separato: CLK=25, MISO=39, MOSI=32, CS=33
Libreria touch: XPT2046_Touchscreen (Paul Stoffregen)
Calibrazione 9 punti salvata in NVS ("tcal9"), interpolazione bilineare
Reset calibrazione: tieni GPIO0 premuto al boot

LVGL 9.5:
  lv_conf.h: LV_COLOR_DEPTH 16, MONTSERRAT_14=1, MONTSERRAT_24=1
  Partition scheme: Huge APP (3MB) obbligatorio
  lv_label_set_text_fmt con %f NON funziona su ESP32
  Usare snprintf + lv_label_set_text per i float

ESP-NOW:
  MAC CYD: 88:13:BF:24:B4:14
  Core 3.x recv: void cb(const esp_now_recv_info_t*, const uint8_t*, int)
  Core 3.x send: void cb(const wifi_tx_info_t*, esp_now_send_status_t)

---

## Gotcha importanti

1. E220 config mode: M0=HIGH M1=HIGH (non M0=LOW M1=HIGH)
2. CRYPT_H/L write-only, leggono sempre 0
3. Canale e' byte intero addr 04H, non campo a 5bit in REG3
4. TX_MODE bit6 e' in REG3 addr 05H, non in addr 06H
5. NETID (addr 02H) = 0x62 di fabbrica, azzerarlo rompe la comunicazione
6. ESP32-S3: Serial2 vuole pin espliciti in begin()
7. CYD: display HSPI e touch VSPI su bus separati
8. ILI9341_2_DRIVER obbligatorio sul CYD (non ILI9341_DRIVER)
9. LVGL 9.5 API diversa da v8: lv_display_create, lv_display_set_flush_cb
10. Partition scheme Huge APP obbligatoria (WiFi+LVGL+TFT_eSPI troppo grandi)
11. portENTER_CRITICAL_ISR nella callback ESP-NOW causa blocchi - usare semplice volatile flag

---

## Stato attuale

Funzionante:
- TX LoRa con fixed mode, AES-128 CTR, RSSI byte
- RX LoRa con decifratura, inoltro ESP-NOW al CYD
- CYD display con LVGL, UI focus batteria (SOC%, V, %/h)
- Calibrazione touch 9 punti con feedback visivo
- MAX17048 sul TX per telemetria batteria reale

Da fare:
- Sensori reali (temp/umid/press) — payload da estendere quando disponibili
- Interazione touch sul display (pulsanti, menu)
