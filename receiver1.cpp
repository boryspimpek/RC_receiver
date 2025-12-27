#include <SPI.h>
#include <RF24.h>

#define CE_PIN 4
#define CSN_PIN 5
#define CRSF_TX_PIN 21 

struct __attribute__((packed)) RadioData {
  uint16_t ch[8];
};

RadioData rx;
RF24 radio(CE_PIN, CSN_PIN);
HardwareSerial CRSF(1);

unsigned long lastPacketTime = 0;
unsigned long lastCrsfSend = 0;
const unsigned long CRSF_INTERVAL = 10; 
const unsigned long FAILSAFE_TIMEOUT = 200;

// CRC8 dla protokołu CRSF
uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
  crc ^= a;
  for (int i = 0; i < 8; i++) {
    crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
  }
  return crc;
}

void packCRSFChannels(uint16_t ch[8], uint8_t *payload) {
  uint32_t bitBuffer = 0;
  uint8_t bitCount = 0;
  uint8_t outIndex = 0;

  for (int i = 0; i < 8; i++) {
    uint16_t v = map(ch[i], 1000, 2000, 172, 1811);
    bitBuffer |= ((uint32_t)v) << bitCount;
    bitCount += 11;
    while (bitCount >= 8) {
      payload[outIndex++] = (uint8_t)(bitBuffer & 0xFF);
      bitBuffer >>= 8;
      bitCount -= 8;
    }
  }
}

void sendCRSF(uint16_t channels[8]) {
  uint8_t buf[26];
  buf[0] = 0xC8; 
  buf[1] = 24;   
  buf[2] = 0x16; 
  packCRSFChannels(channels, &buf[3]);
  uint8_t crc = 0;
  for (int i = 2; i < 25; i++) crc = crc8_dvb_s2(crc, buf[i]);
  buf[25] = crc;
  CRSF.write(buf, 26);
}

void setup() {
  CRSF.begin(420000, SERIAL_8N1, -1, CRSF_TX_PIN); 
  
  if (!radio.begin()) while (1); 
  radio.setAddressWidth(5);
  radio.setDataRate(RF24_2MBPS);
  radio.openReadingPipe(1, 0x11223344AAULL);
  radio.startListening();

  // Domyślne wartości (Failsafe)
  for(int i=0; i<8; i++) rx.ch[i] = 1500;
  rx.ch[0] = 1000; // Gaz na zero (zależnie od mapowania kanałów)
}

void loop() {
  if (radio.available()) {
    radio.read(&rx, sizeof(RadioData));
    lastPacketTime = millis();
  }

  unsigned long now = millis();
  if (now - lastCrsfSend >= CRSF_INTERVAL) {
    lastCrsfSend = now;
    if (now - lastPacketTime > FAILSAFE_TIMEOUT) {
      uint16_t fs[8] = {1000, 1500, 1500, 1500, 1000, 1000, 1000, 1000};
      sendCRSF(fs);
    } else {
      sendCRSF(rx.ch);
    }
  }
}