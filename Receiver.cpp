#include <SPI.h>
#include <RF24.h>

// --- Konfiguracja PINów ---
#define CE_PIN 4
#define CSN_PIN 5
#define CRSF_TX_PIN 21 

RF24 radio(CE_PIN, CSN_PIN);

// Używamy __attribute__((packed)), aby dane miały dokładnie taki rozmiar, jakiego oczekujemy
struct __attribute__((packed)) RcPacket {
  uint16_t ch[8]; 
  uint32_t counter;
};

RcPacket rx;
HardwareSerial CRSF(1);

// --- Timingu ---
unsigned long lastPacketTime = 0;
unsigned long lastCrsfSend = 0;
const unsigned long CRSF_INTERVAL = 10; // Wysyłaj ramkę do FC co 10ms (100Hz)
const unsigned long FAILSAFE_TIMEOUT = 150;

// ---------------- CRC8 DVB-S2 (Optymalizacja) ----------------
uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
  crc ^= a;
  for (int i = 0; i < 8; i++) {
    crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
  }
  return crc;
}

// -------------- Pakowanie kanałów CRSF ----------------
// CRSF używa 11 bitów na kanał
void packCRSFChannels(uint16_t ch[8], uint8_t *payload) {
  uint32_t bitBuffer = 0;
  uint8_t bitCount = 0;
  uint8_t outIndex = 0;

  for (int i = 0; i < 8; i++) {
    uint16_t input = constrain(ch[i], 1000, 2000);
    // Skalowanie PWM (1000-2000) na CRSF (172-1811)
    uint16_t v = map(input, 1000, 2000, 172, 1811);
    
    bitBuffer |= ((uint32_t)v) << bitCount;
    bitCount += 11;

    while (bitCount >= 8) {
      payload[outIndex++] = (uint8_t)(bitBuffer & 0xFF);
      bitBuffer >>= 8;
      bitCount -= 8;
    }
  }
}

void sendCRSF(uint16_t ch[8]) {
  uint8_t buf[26];
  buf[0] = 0xC8;   // Adres odbiornika (Flight Controller)
  buf[1] = 24;     // Długość (Typ + Payload + CRC)
  buf[2] = 0x16;   // Typ: RC Channels Packed

  packCRSFChannels(ch, &buf[3]);

  uint8_t crc = 0;
  for (int i = 2; i < 25; i++) {
    crc = crc8_dvb_s2(crc, buf[i]);
  }
  buf[25] = crc;

  CRSF.write(buf, 26);
}

void setup() {
  Serial.begin(115200);
  
  // Inicjalizacja Serial dla CRSF (420k bodów)
  CRSF.begin(420000, SERIAL_8N1, -1, CRSF_TX_PIN); 

  if (!radio.begin()) {
    Serial.println("NRF24 nie znaleziony!");
    while (1); 
  }

  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);       
  radio.setAddressWidth(5);
  radio.setRetries(0, 15);             
  radio.openReadingPipe(1, 0x11223344AAULL);
  radio.startListening();

  // Wartości początkowe (środek drążków, gaz na minimum)
  for(int i=0; i<8; i++) rx.ch[i] = 1500;
  rx.ch[2] = 1000; 
}

void loop() {
  // 1. Odbiór danych z NRF24
  if (radio.available()) {
    while (radio.available()) {
      radio.read(&rx, sizeof(rx));
    }
    lastPacketTime = millis();
  }

  unsigned long now = millis();

  // 2. Wysyłanie do Betaflight ze stałą częstotliwością
  // To zapewnia stabilny odczyt Link Quality w Betaflight
  if (now - lastCrsfSend >= CRSF_INTERVAL) {
    lastCrsfSend = now;

    if (now - lastPacketTime > FAILSAFE_TIMEOUT) {
      // Tryb Failsafe: Gaz 1000, reszta środek
      uint16_t fs[8] = {1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500};
      sendCRSF(fs);
    } else {
      sendCRSF(rx.ch);
    }
  }
}
