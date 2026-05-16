// ================================================================
//  LilyGo T3-S3 V1.2 — LoRaWAN OTAA base sketch
//  Hardware : ESP32-S3 + SX1276 radio
//  Network  : The Things Network v3 (TTN), EU868
//
//  BEFORE FLASHING:
//  1) In lmic_project_config.h set:
//       #define CFG_eu868          1   (or your region)
//       #define CFG_sx1276_radio   1
//       #define LMIC_USE_INTERRUPTS
//  2) boards.h must have  #define LILYGO_T3_S3_V1_2  active.
//
//  HOW TO USE THIS SKETCH
//  ──────────────────────
//  Step 1 ► Register your device on TTN (https://console.cloud.thethings.network)
//            and copy the three keys into the CREDENTIALS section below.
//
//  Step 2 ► Build your sensor payload in the PAYLOAD section of do_send().
//
//  Step 3 ► Flash and open the Serial Monitor at 115200 baud.
//            You should see:  Joining… → Joined! → TX #1 sent
//
//  Everything else in this file can be left untouched.
// ================================================================

#include <lmic.h>       // MCCI LoRaWAN LMIC library
#include <hal/hal.h>    // LMIC hardware abstraction
#include "utilities.h"  // Board init + OLED helpers (don't edit)


// ================================================================
//  SECTION 1 — CREDENTIALS
//  Copy these values from the TTN console → your device → Overview
// ================================================================

// AppEUI (JoinEUI) — leave all-zeros if TTN v3 does not require it
static const u1_t PROGMEM APPEUI[8] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// DevEUI — For TTN issued EUIs the last bytes should be 0xD5, 0xB3, 0x70.
// This MUST be in little endian format. (lsb in TTN)
static const u1_t PROGMEM DEVEUI[8] = {
    0xB5, 0x78, 0x07, 0xD0, 0x7E, 0xD5, 0xB3, 0x70  // ← replace with yours
};

// AppKey — paste from TTN, selecting "MSB" byte order
static const u1_t PROGMEM APPKEY[16] = {
    0x2C, 0x5A, 0x65, 0x31, 0x96, 0xAF, 0xC4, 0x85, 0x69, 0x2B, 0xB7, 0xAC, 0xFA, 0x13, 0xA4, 0x40
};

// These three functions are called by the LMIC library — do not rename them
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }


// ================================================================
//  SECTION 2 — SETTINGS
// ================================================================

// How often to send a packet (seconds).
// EU868 duty-cycle rules limit you to ~1% airtime, so in practice
// SF7 → min ~20 s,  SF9 → min ~120 s.  60 s is a safe default.
static const unsigned TX_INTERVAL_S = 60;

// LoRaWAN port number (1–223). Like a "channel" that lets you route
// different message types inside your TTN application.
static const uint8_t LORA_PORT = 1;


// ================================================================
//  SECTION 3 — INTERNAL STATE  (no need to edit below this line)
// ================================================================

static osjob_t  sendJob;           // LMIC job handle for the TX timer
static bool     joined    = false; // true once OTAA join succeeds
static uint32_t txCounter = 0;    // counts every uplink sent

void do_send(osjob_t *j);  // forward declaration

// Pin mapping — tells LMIC which GPIO the SX1276 is connected to.
// These match the T3-S3 V1.2 schematic; do not change.
const lmic_pinmap lmic_pins = {
    .nss   = RADIO_CS_PIN,                              // chip-select  GPIO 7
    .rxtx  = LMIC_UNUSED_PIN,                           // not used on SX1276
    .rst   = RADIO_RST_PIN,                             // reset         GPIO 8
    .dio   = { RADIO_DIO0_PIN,   // TX/RX done          GPIO 9
               RADIO_DIO1_PIN,   // RX timeout          GPIO 33
               LMIC_UNUSED_PIN },
};


// ================================================================
//  SECTION 4 — OLED DISPLAY HELPER
//  Shows up to 5 rows of text on the 128×64 OLED.
//  Call oledSetRow(rowNumber, "format", value) anywhere you like.
//  Row 0 is the top line; row 4 is the bottom line.
// ================================================================

#ifdef HAS_DISPLAY
  #define OLED_ROWS 5
  #define OLED_COLS 22    // ~22 characters fit per line at the default font

  static char oledShadow[OLED_ROWS][OLED_COLS + 1];  // one buffer per row

  static void oledSetRow(uint8_t row, const char *fmt, ...) {
      if (!u8g2 || row >= OLED_ROWS) return;
      va_list args;
      va_start(args, fmt);
      vsnprintf(oledShadow[row], OLED_COLS + 1, fmt, args);
      va_end(args);
      // Redraw every row so updating one line doesn't erase the others
      u8g2->clearBuffer();
      for (uint8_t r = 0; r < OLED_ROWS; r++) {
          if (oledShadow[r][0])
              u8g2->drawStr(0, (r + 1) * 12, oledShadow[r]);
      }
      u8g2->sendBuffer();
  }
#else
  #define oledSetRow(row, ...) do {} while(0)  // no-op when no display
#endif


// ================================================================
//  SECTION 5 — LMIC EVENT HANDLER
//  LMIC calls this automatically whenever something happens.
//  You can add your own reactions here (e.g. blink an LED, log to SD).
// ================================================================

static void printDownlink() {
    // Helper: pretty-print a received downlink message
    Serial.printf("  Downlink: %d byte(s) →", LMIC.dataLen);
    for (size_t i = 0; i < LMIC.dataLen; i++)
        Serial.printf(" %02X", LMIC.frame[LMIC.dataBeg + i]);
    Serial.println();
}

void onEvent(ev_t ev) {
    switch (ev) {

    // ── Join process ─────────────────────────────────────────────
    case EV_JOINING:
        // Device has started the OTAA join procedure
        Serial.println("Joining TTN…");
        oledSetRow(0, "Joining TTN...");
        break;

    case EV_JOIN_TXCOMPLETE:
        // A Join Request was transmitted but no Accept has arrived yet.
        // LMIC retries automatically — seeing this 1-3 times is normal.
        if (!joined)
            Serial.println("  Join request sent, waiting for gateway…");
        break;

    case EV_JOINED:
        // OTAA succeeded — the device now has a session with TTN
        joined = true;
        Serial.println("Joined! ✓");
        oledSetRow(0, "Joined! :)");
        // Disable link-check mode: it changes the data rate and
        // reduces the maximum payload size unnecessarily.
        LMIC_setLinkCheckMode(0);
        break;

    case EV_JOIN_FAILED:
        // LMIC exhausted all join attempts without hearing back from a gateway.
        // It will NOT retry on its own after this — we must restart manually.
        joined = false;
        Serial.println("Join FAILED — restarting join in 30 s…");
        oledSetRow(0, "Join failed :(");
        oledSetRow(1, "Retry in 30s");
        LMIC_reset();
        os_setTimedCallback(&sendJob, os_getTime() + sec2osticks(30), do_send);
        break;

    case EV_REJOIN_FAILED:
        // Periodic rejoin (Class B) failed — force a full OTAA rejoin.
        joined = false;
        Serial.println("Rejoin FAILED — forcing new OTAA join…");
        oledSetRow(0, "Rejoin fail");
        LMIC_reset();
        os_setTimedCallback(&sendJob, os_getTime() + sec2osticks(30), do_send);
        break;

    // ── Uplink ───────────────────────────────────────────────────
    case EV_TXSTART:
        // Radio is about to transmit
        Serial.printf("Sending packet #%u…\n", txCounter);
        oledSetRow(1, "TX #%u...", txCounter);
        digitalWrite(BOARD_LED, LED_ON);    // LED on while transmitting
        break;

    case EV_TXCOMPLETE:
        // Transmission finished (and RX windows have been checked)
        digitalWrite(BOARD_LED, LED_OFF);

        if (LMIC.txrxFlags & TXRX_ACK) {
            // Network server confirmed receipt (confirmed uplinks only)
            Serial.printf("Packet #%u confirmed  RSSI=%d dBm  SNR=%d\n",
                          txCounter, LMIC.rssi, LMIC.snr);
            oledSetRow(1, "#%u ACK %ddBm", txCounter, LMIC.rssi);
        } else {
            Serial.printf("Packet #%u sent  RSSI=%d dBm  SNR=%d\n",
                          txCounter, LMIC.rssi, LMIC.snr);
            oledSetRow(1, "#%u ok %ddBm SNR=%d ", txCounter, LMIC.rssi, LMIC.snr);
        }

        if (LMIC.dataLen) {
            // A downlink message arrived in the RX1 or RX2 window
            printDownlink();
            oledSetRow(2, "DL: %d bytes", LMIC.dataLen);
        }

        // Schedule the next uplink
        os_setTimedCallback(&sendJob,
                            os_getTime() + sec2osticks(TX_INTERVAL_S),
                            do_send);
        Serial.printf("Next TX in %u s\n\n", TX_INTERVAL_S);
        break;

    case EV_TXCANCELED:
        Serial.println("TX canceled");
        digitalWrite(BOARD_LED, LED_OFF);
        break;

    case EV_RXSTART:
        // Opening a receive window — do NOT print here, it disturbs timing
        break;

    // ── Rarely seen, but useful to know ──────────────────────────
    case EV_LOST_TSYNC:
        Serial.println("Lost time sync");
        break;
    case EV_RESET:
        Serial.println("LMIC reset");
        break;
    case EV_LINK_DEAD:
        // No downlink received for a long time — session is stale.
        // Reset and trigger a fresh OTAA join to restore the connection.
        joined = false;
        Serial.println("Link dead — forcing new OTAA join…");
        oledSetRow(0, "Link dead");
        oledSetRow(1, "Rejoining...");
        LMIC_reset();
        os_setTimedCallback(&sendJob, os_getTime() + sec2osticks(10), do_send);
        break;
    case EV_LINK_ALIVE:
        Serial.println("Link alive again");
        oledSetRow(0, "Link alive :)");
        break;

    default:
        Serial.printf("LMIC event: %u\n", (unsigned)ev);
        break;
    }
}


// ================================================================
//  SECTION 6 — PAYLOAD  ← THIS IS WHERE YOU PUT YOUR SENSOR DATA
// ================================================================
//
//  Rules:
//    • Keep payloads small — every byte costs airtime and battery.
//    • Use integer math: multiply floats by 10 or 100, then truncate.
//    • Write a matching payload formatter in the TTN console to decode.
//
//  Example — temperature as int16 (value × 10):
//
//    float tempC = readSensor();         // e.g. 23.4 °C
//    int16_t t   = (int16_t)(tempC * 10);
//    payload[0]  = t >> 8;              // high byte
//    payload[1]  = t & 0xFF;           // low  byte
//
//  Matching TTN JavaScript formatter:
//    function decodeUplink(input) {
//      var raw = (input.bytes[0] << 8) | input.bytes[1];
//      if (raw > 32767) raw -= 65536;  // handle negative values
//      return { data: { temperature: raw / 10.0 } };
//    }

void do_send(osjob_t *j) {
    if (LMIC.opmode & OP_TXRXPEND) {
        // Radio is still busy with a previous TX/RX — skip this slot
        Serial.println("Radio busy, skipping TX");
        return;
    }

    // ── Build your payload here ──────────────────────────────────
    txCounter++;

    uint8_t payload[2];
    payload[0] = (txCounter >> 8) & 0xFF;   // counter high byte
    payload[1] =  txCounter       & 0xFF;   // counter low  byte

    // ── End of payload ───────────────────────────────────────────

    // Send an unconfirmed uplink on LORA_PORT.
    // Change the last argument from 0 to 1 for a confirmed (ACK) uplink,
    // but use confirmed uplinks sparingly — they double the airtime used.
    LMIC_setTxData2(LORA_PORT, payload, sizeof(payload), /*confirmed=*/0);
}


// ================================================================
//  SETUP & LOOP
// ================================================================

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) { }  // wait for USB Serial to enumerate

    Serial.println("\n=== T3-S3 V1.2  LoRaWAN OTAA ===");

    initBoard();  // SPI, I2C, OLED, SD — handled in utilities.h

    oledSetRow(0, "T3-S3  LoRaWAN by GRC");
    oledSetRow(1, "Starting...");

    os_init();     // start the LMIC event scheduler
    LMIC_reset();  // clear any previous session

    // Adaptive Data Rate (ADR): the network adjusts your data rate
    // automatically to maximise range and minimise airtime.
    // ✓ Enable  for fixed/indoor sensors  (LMIC_setAdrMode(true))
    // ✗ Disable for moving devices        (LMIC_setAdrMode(false))
    LMIC_setAdrMode(false);

    pinMode(BOARD_LED, OUTPUT);
    digitalWrite(BOARD_LED, LED_OFF);

    do_send(&sendJob);  // queue the first packet (also triggers OTAA join)

    Serial.printf("TX interval: %u s\n\n", TX_INTERVAL_S);
}

void loop() {
    // Hand control to the LMIC scheduler on every iteration.
    // Never add blocking code (delay, while loops) here — it breaks timing.
    os_runloop_once();
}
