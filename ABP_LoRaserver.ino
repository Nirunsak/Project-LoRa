#include <CayenneLPP.h>
#include <DHTesp.h>
#include "DHT.h"
#include <Adafruit_GPS.h>
#include <HardwareSerial.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include "SSD1306.h"

static const PROGMEM u1_t NWKSKEY[16] = { 0xD5, 0xC2, 0xA1, 0x99, 0xDE, 0xEA, 0x7D, 0xBE, 0x96, 0x7F, 0x3F, 0xA7, 0x95, 0xE6, 0x3A, 0xAD };


static const u1_t PROGMEM APPSKEY[16] = { 0x0E, 0x04, 0x11, 0xA6, 0x32, 0x52, 0xB1, 0xD6, 0x2A, 0x20, 0x30, 0xDE, 0x98, 0x5D, 0x5C, 0x05 };

static const u4_t DEVADDR = 0x01A48CEA;

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;
// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 30; 

const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 33, 32},
  .rxtx_rx_active = 0,
  .rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
  .spi_freq = 8000000,
};

int dhtPin = 23;
const int pinSensorPIR = 34;
int valorSensorPIR = 0;
int valorDC = 35;
float tensaoDC;
float t=0;
float h=0;
float f=0;

#define DHTTYPE DHT22 
// Adafruit GPS
CayenneLPP lpp(15);
SSD1306 display(0x3c, 4, 15); //construtor do objeto que controlaremos o display
DHT dht(dhtPin, DHTTYPE);

void onEvent (ev_t ev) 
{
  Serial.print(os_getTime());
  Serial.print(": ");
  switch(ev) 
  {
    case EV_TXCOMPLETE:
      Serial.printf("EV_TXCOMPLETE (includes waiting for RX windows)\r\n");;
        if (LMIC.dataLen) {
        // data received in rx slot after tx
        Serial.print(F("Data Received: "));
        Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
        Serial.println();
      }
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
      break;  
    case EV_RXCOMPLETE:
      if (LMIC.dataLen)
      {
        Serial.printf("Received %d bytes\n", LMIC.dataLen);
      }
      break;
    default:
      Serial.printf("Unknown event\r\n");
      break;
  }
}

void do_send(osjob_t* j)
{
  if (LMIC.opmode & OP_TXRXPEND) 
  {
    Serial.printf("OP_TXRXPEND, not sending\r\n");
  } 
  else
  if (!(LMIC.opmode & OP_TXRXPEND)) 
  {  
    lpp.reset();
    float valorTeste = 39;
      
    lpp.addTemperature(1, t);
    lpp.addRelativeHumidity(2, h);
    //lpp.addRelativeHumidity(3, valorTeste);
    
    Serial.printf("Temperature : %.2f, Humidity : %.2f\r\n", t, h);
    display.clear();
    display.drawString(0, 0, "Temperature: ");
    display.drawString(90, 0, String(t));
    display.drawString(0, 20, "Humidity  : ");
    display.drawString(90,20, String(h));
    display.display();    

    LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
         
    Serial.printf("Packet queued\r\n");
  }

}

void setup() 
{
  Serial.begin(115200);
  Serial.printf("Starting...\r\n");
  
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW); // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(16, HIGH);
  display.init();
  display.setFont(ArialMT_Plain_10);
  
  dht.begin();

  
  os_init();
  LMIC_reset();
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
  LMIC_setLinkCheckMode(0);
  LMIC.dn2Dr = DR_SF9;
  LMIC_setDrTxpow(DR_SF7,14);
  Serial.printf("LMIC setup done!\r\n");
  do_send(&sendjob);
}

void loop() 
{
   PegarTemperatura();
  os_runloop_once();
}

void PegarTemperatura(){
valorSensorPIR = digitalRead(pinSensorPIR);
  tensaoDC = ((valorDC*0.00450)*5);
   h = dht.readHumidity();
   t = dht.readTemperature();
  f = dht.readTemperature(true);
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  float hif = dht.computeHeatIndex(f, h);
  float hic = dht.computeHeatIndex(t, h, false);

  delay(2000);
}
