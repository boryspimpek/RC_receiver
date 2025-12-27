#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_PCF8574.h>

// --- KONFIGURACJA PINÓW ---
#define PIN_THROTTLE 32
#define PIN_YAW      33
#define PIN_PITCH    34
#define PIN_ROLL     35
#define PIN_AUX1     36
#define PIN_AUX2     39
#define BTN_TOGGLE_1 13
#define BTN_TOGGLE_2 14
#define CE_PIN       4
#define CS_PIN       5

struct __attribute__((packed)) RadioData {
  uint16_t ch[8]; 
};

RadioData data;
RF24 radio(CE_PIN, CS_PIN);
Adafruit_PCF8574 pcf;

// Zmienne trymowania - teraz zawsze startują od 0 po włączeniu zasilania
int8_t trim[4] = {0, 0, 0, 0}; 
uint8_t lastPcfState = 0xFF;



void setup() {
  Serial.begin(115200);
  

  // Inicjalizacja nRF24
  if (!radio.begin()) { Serial.println("Radio FAIL"); while(1); }
  radio.setAddressWidth(5);
  radio.openWritingPipe(0x11223344AAULL);
  radio.setDataRate(RF24_2MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.stopListening();

  // Inicjalizacja PCF8574
  if (!pcf.begin(0x20, &Wire)) Serial.println("PCF FAIL");
  for (int i = 0; i < 8; i++) pcf.pinMode(i, INPUT_PULLUP);

  pinMode(BTN_TOGGLE_1, INPUT_PULLUP);
  pinMode(BTN_TOGGLE_2, INPUT_PULLUP);
}

uint16_t readAxis(int pin, int8_t trimVal) {
  int raw = analogRead(pin);
  int mapped = map(raw, 0, 4095, 1000, 2000);
  return (uint16_t)constrain(mapped + (trimVal * 4), 1000, 2000);
}

void loop() {
  // 1. Obsługa przycisków trymowania
  uint8_t currentPcf = pcf.digitalReadByte();
  for (int i = 0; i < 8; i++) {
    if (!(currentPcf & (1 << i)) && (lastPcfState & (1 << i))) {
      if (i == 0) trim[0]++; if (i == 1) trim[0]--;
      if (i == 2) trim[1]++; if (i == 3) trim[1]--;
      if (i == 4) trim[2]++; if (i == 5) trim[2]--;
      if (i == 6) trim[3]++; if (i == 7) trim[3]--;
      
      Serial.printf("Trims: T:%d Y:%d P:%d R:%d\n", trim[0], trim[1], trim[2], trim[3]);
    }
  }
  lastPcfState = currentPcf;

  // 2. Pobieranie danych
  data.ch[0] = readAxis(PIN_THROTTLE, trim[0]);
  data.ch[1] = readAxis(PIN_YAW,      trim[1]);
  data.ch[2] = readAxis(PIN_PITCH,    trim[2]);
  data.ch[3] = readAxis(PIN_ROLL,     trim[3]);
  data.ch[4] = map(analogRead(PIN_AUX1), 0, 4095, 1000, 2000);
  data.ch[5] = map(analogRead(PIN_AUX2), 0, 4095, 1000, 2000);
  data.ch[6] = digitalRead(BTN_TOGGLE_1) ? 1000 : 2000;
  data.ch[7] = digitalRead(BTN_TOGGLE_2) ? 1000 : 2000;

  // 3. Wysyłka
  radio.write(&data, sizeof(RadioData));
  
  delay(10);
}