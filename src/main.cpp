#include <Arduino.h>
#include <SPI.h>    // LoRa-Modul
#include <Wire.h>   // I²C - OLED, Tempsensor
#include "esp_adc_cal.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSans12pt7b.h>
#include <lmic.h>
#include <hal/hal.h>

#include "../private/private-keys.h"

#define PROJECT_NAME "TTN-Demo-Node"

//-----------------------------------------------------------------------------
// Display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

//-----------------------------------------------------------------------------
#define LORA_TX_INTERVAL    60

// LoRa SX1276
#define _lora_sclk 5
#define _lora_miso 19
#define _lora_mosi 27
#define _lora_nss 18
#define _lora_rst 23
#define _lora_dio0 26
#define _lora_dio1 33
#define _lora_dio2 32

// ADC Battery-voltage
#define ADC_PIN         35
#define BATTERY_INTERVAL  1000

//-----------------------------------------------------------------------------
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#ifndef OTAA
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
#else
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}
#endif

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = _lora_nss,                       // chip select on feather (rf95module) CS
  .rxtx = LMIC_UNUSED_PIN,
  .rst = _lora_rst,                       // reset pin
  .dio = {_lora_dio0, _lora_dio1, _lora_dio2}, // assumes external jumpers [feather_lora_jumper]
  // DIO1 is on JP1-1: is io1 - we connect to GPO6
  // DIO1 is on JP5-3: is D2 - we connect to GPO5
};

//-----------------------------------------------------------------------------
typedef enum {
  ALIGN_LEFT,
  ALIGN_CENTER,
  ALIGN_RIGHT
} textalign;

typedef struct {
  int16_t x;    // Horizontal position
  int16_t y;    // Vertical position
  uint16_t w;   // Width
  uint16_t h;   // Height
  uint16_t r;   // Radius
} roundrect_t;

union {
  struct __attribute__((packed)) {
    uint16_t temperature;   // floating point number sflt16-encoded
    uint16_t humidity;      // floating point number uflt16-encoded
    uint16_t pressure;      // floating point number uflt16-encoded
    uint16_t voltage;       // floating point number sflt16-encoded
  } data;
  uint8_t buffer[sizeof data];
} lorapacket;

//-----------------------------------------------------------------------------
int vref = 1100;
uint32_t batteryTimer = 0;
float batteryVoltage = 0;
boolean sensor_ready = false;

uint16_t loraCount = 0;

roundrect_t voltageGauge =    { 0 ,50 , 62, 14, 3};
roundrect_t counterGauge =    {66 ,50 , 62, 14, 3};
roundrect_t temperaturGauge = { 0 ,32 , 62, 14, 3};
roundrect_t humidityGauge =   {66 ,32 , 62, 14, 3};

//-----------------------------------------------------------------------------
void msg_send_to_ttn(osjob_t* j);
void onEvent (ev_t ev);
void initGauge(roundrect_t rect);
void drawGauge(roundrect_t rect, char *text, textalign align = ALIGN_LEFT);

//-----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  
  Serial.println("====================================================================");
  Serial.println("Starting ESP32 - LoRaWAN TTN-Node (ABP)");
  Serial.printf("Run on Core : %d @ %d MHz\n", xPortGetCoreID(), ESP.getCpuFreqMHz());
  Serial.printf("Sketch : %d ; Free : %d\n", ESP.getSketchSize(), ESP.getFreeSketchSpace());

  // Init Display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c)) {  // SSD1306_SWITCHCAPVCC generate display voltage from 3.3V internally
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen -- Adafruit splash screen.
  display.setRotation(2);
  display.display();
  delay(2000);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setFont(&FreeSans12pt7b);
  display.setCursor(8, 22);             // Start at top-left corner
  display.println(F("TTN-Node"));
  display.drawRoundRect(0 ,0 , 128, 26, 5, SSD1306_WHITE);
  display.display();

  initGauge(voltageGauge);
  initGauge(counterGauge);
  initGauge(temperaturGauge);
  initGauge(humidityGauge);

  drawGauge(temperaturGauge, (char *)"Demo", ALIGN_CENTER);
#ifndef OTAA
  drawGauge(humidityGauge, (char *)"ABP", ALIGN_CENTER);
#else
  drawGauge(humidityGauge, (char *)"OTAA", ALIGN_CENTER);
#endif

  // Init ADC for measuring the battery voltage and internal ADC Calibrierung
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);    //Check type of calibration value used to characterize ADC
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    Serial.printf("eFuse Vref    : %u mV\n", adc_chars.vref);
    vref = adc_chars.vref;
  } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
    Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
  } else {
    Serial.printf("Default Vref  : 1100mV\n");
  }

   // Status Prints
  Serial.print("Board : ");
#if defined(ARDUINO_TTGO_LoRa32_V1)
  Serial.println("ARDUINO_TTGO_LoRa32_V1");
#elif defined(ARDUINO_HELTEC_WIFI_LORA_32) || defined(ARDUINO_HELTEC_WIFI_LORA_32_V2) || defined(ARDUINO_HELTEC_WIRELESS_STICK)
  Serial.println("ARDUINO_HELTEC_WIFI_LORA_32");
#else
  Serial.println("unknown");
#endif  

  // TTN init
  Serial.print("Init LoRaWAN LMIC : ");
 
  os_init();
  LMIC_reset();

#ifndef OTAA
  LMIC_setSession(0x13, DEVADDR, (xref2u1_t)&NWKSKEY, (xref2u1_t)&APPSKEY);
#endif

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // Die Region muss in der platformio.ini eingestellt werden

#if defined(UCFG_us915)
  Serial.println("UCFG_us915 ist set");
#endif

#if defined(CFG_eu868)
  Serial.println("CFG_eu868 ist set");
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
#endif

  LMIC_setLinkCheckMode(0);     // Disable link check validation
  LMIC.dn2Dr = DR_SF9;          // TTN uses SF9 for its RX2 window.
  LMIC_setDrTxpow(DR_SF7, 16);  // Set data rate and transmit power for uplink

#ifndef OTAA
  // Start first message sending
  msg_send_to_ttn(&sendjob);
#else
  // Start joining and key exchange
  LMIC_startJoining ();
#endif

  batteryTimer = millis();
}

//-----------------------------------------------------------------------------
void loop() {
  if (batteryTimer < millis()) {
    char textBuffer[16];

    batteryTimer = millis() + BATTERY_INTERVAL;

    batteryVoltage = ((float)analogRead(ADC_PIN) / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
//    Serial.printf("Battery voltage : %2.2f V\n", batteryVoltage);

    sprintf(textBuffer, "%2.2f V", batteryVoltage);
    drawGauge(voltageGauge, textBuffer, ALIGN_RIGHT);
  }

  os_runloop_once();
}

//-----------------------------------------------------------------------------
void msg_send_to_ttn(osjob_t* j) {
  char textBuffer[16];

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println("OP_TXRXPEND, not sending");
  } else {
    // Prepare upstream data transmission at the next possible time.
    lorapacket.data.temperature = LMIC_f2sflt16(23.5 / 100);    // Bereich 80,0°C bis -20,0°C
    lorapacket.data.humidity    = LMIC_f2uflt16((float)56 / 100);      // Bereich 0% - 100%
    lorapacket.data.pressure    = LMIC_f2uflt16((float)1003 / 1100);   // Bereich 950mbar - 1060mbar; 5000m Höhe 53,53%
    lorapacket.data.voltage     = LMIC_f2uflt16(batteryVoltage / 6);  // Bereich 2,5V - 5,0V
    
    Serial.printf("%2.2f V\n", batteryVoltage);

    Serial.printf("Sequence no up/dn : %d / %d\n", LMIC.seqnoUp, LMIC.seqnoDn);

    Serial.printf("RAW Data : | %3d | ", sizeof(lorapacket.buffer));
    for (int i=0; i < sizeof(lorapacket.buffer); i++) {
      Serial.printf("%2x ", lorapacket.buffer[i]);
    }
    Serial.println("|");

    LMIC_setTxData2(1, lorapacket.buffer, sizeof(lorapacket.buffer), 0);
    Serial.println("Packet queued");

    loraCount++;
    sprintf(textBuffer, "%d", loraCount);
    drawGauge(counterGauge, textBuffer, ALIGN_RIGHT);
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

//-----------------------------------------------------------------------------
void onEvent (ev_t ev) {
  Serial.printf("%09d : ", os_getTime());
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
#ifdef OTAA
      {
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        Serial.printf("netid: %d \n", netid);
        Serial.printf("devaddr: %04x\n", devaddr);
        Serial.print("AppSKey: ");
        for (size_t i=0; i<sizeof(artKey); ++i) {
          if (i != 0)
            Serial.print("-");
          Serial.printf("%02x", artKey[i]);
        }
        Serial.println("");
        Serial.print("NwkSKey: ");
        for (size_t i=0; i<sizeof(nwkKey); ++i) {
          if (i != 0)
            Serial.print("-");
          Serial.printf("%02x", nwkKey[i]);
        }
        Serial.println();

        drawGauge(humidityGauge, (char *)"joined");

        msg_send_to_ttn(&sendjob);
      }
#endif
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.printf("EV_TXCOMPLETE (includes waiting for RX windows)\n");
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.printf("Received %d bytes of payload\n", LMIC.dataLen);
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(LORA_TX_INTERVAL), msg_send_to_ttn);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
      break;
  }
}

//-----------------------------------------------------------------------------
void initGauge(roundrect_t rect) {
  display.drawRoundRect(rect.x, rect.y, rect.w, rect.h, rect.r, SSD1306_WHITE);
  display.display();
}

//-----------------------------------------------------------------------------
void drawGauge(roundrect_t rect, char *text, textalign align) {
  int16_t sx, sy, text_x = rect.x+3;
  uint16_t sw, sh;

  display.setFont();
  display.setTextSize(1); 
  display.setTextColor(SSD1306_WHITE);
  display.fillRect(rect.x+2, rect.y+2, rect.w-4, rect.h-4, SSD1306_BLACK);
  display.getTextBounds(text, 0, 0, &sx, &sy, &sw, &sh);

  switch (align) {
    case ALIGN_LEFT:
      break;

    case ALIGN_CENTER:
      text_x = rect.x + (rect.w >> 1) - (sw >> 1) + 1;
      break;

    case ALIGN_RIGHT:
      text_x = rect.x + rect.w - sw - 3;
      break;
  }

  display.setCursor(text_x, rect.y+3);
  display.printf(text);
  display.display();
}