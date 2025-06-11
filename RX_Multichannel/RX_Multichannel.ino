// Revision: RX_ESB_USB_LatchChannel_Rev1.7

#include "nrf_to_nrf.h"
#include "Adafruit_TinyUSB.h"

Adafruit_USBD_MIDI usb_midi;
nrf_to_nrf nrf;

uint8_t buffer[32];
unsigned long lastPacketTime = 0;
unsigned long lastLogTime = 0;
bool latched = false;
uint8_t channels[] = {5, 10, 15, 20};
uint8_t currentChannel = 0xFF;

const unsigned long LATENCY_LIMIT = 8640000UL;  // 24 hours

void setup() {
  usb_midi.begin();
  Serial.begin(115200);
  nrf.begin();
  nrf.setPALevel(NRF_PA_MAX);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (!latched) {
    for (uint8_t i = 0; i < 4; i++) {
      currentChannel = channels[i];
      nrf.setChannel(currentChannel);
      nrf.startListening();
      delay(5);

      unsigned long t0 = millis();
      while (millis() - t0 < 20) {
        if (nrf.available()) {
          nrf.read(buffer, sizeof(buffer));
          sendCC(buffer[0], buffer[1]);
          lastPacketTime = millis();
          latched = true;
          break;
        }
      }

      if (latched) break;
    }
    return;
  }

  if (nrf.available()) {
    nrf.read(buffer, sizeof(buffer));
    sendCC(buffer[0], buffer[1]);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    lastPacketTime = millis();
  }

  if (millis() - lastLogTime >= 250) {
    unsigned long latency = millis() - lastPacketTime;

    if (latency > LATENCY_LIMIT) {
      lastPacketTime = millis();  // reset to avoid overflow
      latency = 0;
    }

    int8_t rssi = -nrf.getRSSI();
    Serial.print("RSSI: ");
    Serial.print(rssi);
    Serial.print(" dBm | Latency: ");
    Serial.print(latency);
    Serial.print(" ms | Channel: ");
    Serial.println(currentChannel);

    lastLogTime = millis();
  }

  delayMicroseconds(200);
}

void sendCC(uint8_t cc, uint8_t val) {
  uint8_t packet[] = {0x0B, 0xB0, cc, val};
  tud_midi_stream_write(0, packet, 4);
}
