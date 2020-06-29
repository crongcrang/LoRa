// Modified from Github proffalken/HeltecGPS.ino
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <U8x8lib.h>
#include <CayenneLPP.h>
#include <Adafruit_BME280.h>

#define  V1

#define SEALEVELPRESSURE_HPA (1013.25)
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

Adafruit_BME280 bme; //I2C
static float temp,pa,hum,alt;

#define BUILTIN_LED 25
 
// the OLED used
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);
 
// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;

// Create the LPP object
CayenneLPP lpp(51);

// Create the GPS Object
// TinyGPSPlus gps;

// LoRa Pins
#define LoRa_RST  14  // GPIO 14
#define LoRa_CS   18  // GPIO 18
#define LoRa_DIO0 26  // GPIO 26
#define LoRa_DIO2 32  // GPIO 32

#ifdef V2 //WIFI Kit series V1 not support Vext control
  #define LoRa_DIO1    35   // GPIO35 -- SX127x's IRQ(Interrupt Request) V2
#else
  #define LoRa_DIO1    33   // GPIO33 -- SX127x's IRQ(Interrupt Request) V1
#endif



 

  // ABP keys
  //UNO1
  // LoRaWAN NwkSKey, network session key (msb)
  static const PROGMEM u1_t NWKSKEY[] =  { 0xFF, 0xFF, 0x15, 0xFF, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0x20, 0x00, 0x3B }; //ใส่ของตัวเอง
 
  // LoRaWAN AppSKey, application session key (msb)
  static const u1_t PROGMEM APPSKEY[] = { 0xFF, 0xFF, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xFF, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0x55, 0x44, 0x3B }; //ใส่ของตัวเอง
 
  // LoRaWAN end-device address (DevAddr)
  static const u4_t DEVADDR = 0x01020304; //ใส่ของตัวเอง
 
  void os_getArtEui (u1_t* buf) { }
  void os_getDevEui (u1_t* buf) { }
  void os_getDevKey (u1_t* buf) { }
 

 
static uint8_t mydata[] = {13, 37};
static osjob_t sendjob;
 
// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = LoRa_CS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LoRa_RST,
    .dio = { LoRa_DIO0, LoRa_DIO1, LoRa_DIO2 },
};
 
void onEvent (ev_t ev) {
  Serial.print(os_getTime());
//  u8x8.setCursor(0, 5);
//  u8x8.printf("Pa %lu", pa/100000);
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      u8x8.drawString(0, 7, "EV_SCAN_TIMEOUT");
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      u8x8.drawString(0, 7, "EV_BEACON_FOUND");
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      u8x8.drawString(0, 7, "EV_BEACON_MISSED");
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      u8x8.drawString(0, 7, "EV_BEACON_TRACKED");
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      u8x8.drawString(0, 7, "                ");
      u8x8.drawString(0, 7, "EV_JOINING");
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      u8x8.drawString(0, 7, "EV_JOINED ");
      LMIC_setDrTxpow(DR_SF7, 14); //added fixed SF after join for longer range messages
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      u8x8.drawString(0, 7, "EV_RFUI");
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      u8x8.drawString(0, 7, "EV_JOIN_FAILED");
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      u8x8.drawString(0, 7, "EV_REJOIN_FAILED");
      //break;
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      u8x8.drawString(0, 7, "EV_TXCOMPLETE");
      digitalWrite(BUILTIN_LED, LOW);
      if (LMIC.txrxFlags & TXRX_ACK) {
        Serial.println(F("Received ack"));
        u8x8.drawString(0, 7, "Received ACK");
      }
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        u8x8.drawString(0, 6, "RX ");
        Serial.println(LMIC.dataLen);
        u8x8.setCursor(4, 6);
        u8x8.printf("%i bytes", LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
        u8x8.setCursor(0, 7);
        u8x8.printf("RSSI %d SNR %.1d", LMIC.rssi, LMIC.snr);
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      u8x8.drawString(0, 7, "EV_LOST_TSYNC");
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      u8x8.drawString(0, 7, "EV_RESET");
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      u8x8.drawString(0, 7, "EV_RXCOMPLETE");
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      u8x8.drawString(0, 7, "EV_LINK_DEAD");
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      u8x8.drawString(0, 7, "EV_LINK_ALIVE");
      break;
    default:
      Serial.println(F("Unknown event"));
      u8x8.setCursor(0, 7);
      u8x8.printf("UNKNOWN EVENT %d", ev);
      break;
  }
}
 
void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
    u8x8.drawString(0, 7, "OP_TXRXPEND, not sent");
  } else {
    // Read the sensors and pack up the data
      temp = bme.readTemperature();
      pa = bme.readPressure()/100;
      hum = bme.readHumidity();
      alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
  
      Serial.print("Temp:");
      Serial.print(temp);
      Serial.println(" C");
  
      Serial.print("Pa:");
      Serial.print(pa);
      Serial.println(" hPa");
  
      Serial.print("Hum:");
      Serial.print(hum);
      Serial.println(" %");
  
      Serial.print("Alt:");
      Serial.print(alt);
      Serial.println(" m");

      //display sensor
      u8x8.setCursor(0, 3);
      u8x8.printf("T %.2f,H %.2f", temp, hum);
      u8x8.setCursor(0, 5);
      u8x8.print("P:");
      u8x8.print(pa);
      u8x8.print(" hPa.");

      lpp.reset();
      lpp.addTemperature(2, temp);
      lpp.addRelativeHumidity(3, hum);
      lpp.addBarometricPressure(4, pa);
      lpp.addAnalogInput(5, alt);  

    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(2, lpp.getBuffer(), lpp.getSize(), 0);
    Serial.println(F("Packet queued"));
    u8x8.drawString(0, 7, "PACKET QUEUED");
    digitalWrite(BUILTIN_LED, HIGH);
  }
  // Next TX is scheduled after TX_COMPLETE event.
}
 
 
void setup() {
 
  Serial.begin(115200);
  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  //u8x8.drawString(0, 1, "@MBConsultingUK");
  u8x8.drawString(0, 1, "LoRaWAN by CAT");

  // I2c default address is 0x76, if the need to change please modify bme.begin(Addr)

  if (!bme.begin(0x76)) {
        Serial.println("No sensor device found, check line or address!");
        while (1);
  }

  SPI.begin(5, 19, 27);
 
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
 
#ifndef USE_JOINING
    #ifdef PROGMEM
      // On AVR, these values are stored in flash and only copied to RAM
      // once. Copy them to a temporary buffer here, LMIC_setSession will
      // copy them into a buffer of its own again.
      uint8_t appskey[sizeof(APPSKEY)];
      uint8_t nwkskey[sizeof(NWKSKEY)];
      memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
      memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
      LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
      // If not running an AVR with PROGMEM, just use the arrays directly
      LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif
#endif
 
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  // NA-US channels 0-71 are configured automatically
  /* Use Freq in lorabase.h
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  */
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
 
  // Disable link check validation
  LMIC_setLinkCheckMode(0);
 
  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;
  
  LMIC_setDrTxpow(DR_SF9, 14); //set join at SF8 with power 14
  LMIC_setClockError (MAX_CLOCK_ERROR * 10/100);
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);
 
  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}
 
void loop() {
  os_runloop_once();
}
