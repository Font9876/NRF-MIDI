// Revision: TX_USB_ESB_Setup_Rev3.22c

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "Adafruit_TinyUSB.h"
#include "nrf_to_nrf.h"
#include <Adafruit_SSD1306.h>

Adafruit_USBD_MIDI usb_midi;
nrf_to_nrf nrf;
Adafruit_ADS1115 ads;
Adafruit_SSD1306 display(128, 64, &Wire);

const float VREF = 3.280;
const float HYSTERESIS = 0.9;
const uint8_t ANALOG_CHANNEL_COUNT = 4;
const uint8_t BUTTON_COUNT = 8;
const uint32_t SETUP_TIMEOUT_MS = 7000;
const uint32_t IDLE_TIMEOUT_MS = 1000;

const uint8_t buttonPins[BUTTON_COUNT] = {10, 11, 12, 13, 14, 15, 16, 17};
bool lastButtonState[BUTTON_COUNT] = {0};
bool buttonStates[BUTTON_COUNT] = {0};
bool buttonToggles[BUTTON_COUNT] = {0};
uint8_t lastSent[ANALOG_CHANNEL_COUNT] = {255, 255, 255, 255};

bool lastUSBState = false;
unsigned long lastTouchTime = 0;
uint8_t lastTouchedCC = 255;
uint8_t lastTouchedValue = 255;

bool setupMode = false;
uint8_t selectedSetting = 0;
uint8_t esbChannel = 5;
uint8_t ccLayer = 1;
uint8_t midiChannel = 1;
uint8_t buttonMode = 0;
bool adsConnected = true;
bool needsOledRefresh = true;

unsigned long setupStartTime = 0;
unsigned long lastActivityTime = 0;
bool radioActive = true;

// LED blink state
unsigned long ledTimer = 0;
bool ledState = false;

void updateLEDBlink(bool usbMode) {
  unsigned long interval = usbMode ? 1000 : (ledState ? 2 : 500);
  if (millis() - ledTimer >= interval) {
    ledTimer = millis();
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
  }
}

void drawOLEDStatus(bool isUSB) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("TX1 ");
  display.print(isUSB ? "Scomp Link" : "The Force");
  display.setCursor(0, 20);
  display.printf("Chan %d  Lyr %d  %s", esbChannel, ccLayer, isUSB ? "USB" : "ESB");
  display.display();
  needsOledRefresh = false;
}

void drawSetupMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("SETUP MODE");
  display.setCursor(0, 16);
  display.print("1. 2.4GHz Channel");
  display.setCursor(0, 28);
  display.print("2. CC Layer");
  display.setCursor(0, 40);
  display.print("3. MIDI Channel");
  display.setCursor(0, 52);
  display.print("4. Button Mode");
  display.display();
  setupStartTime = millis();
}

void drawSubMenu(const char* title, const char* options[4]) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(title);
  for (int i = 0; i < 4; i++) {
    display.setCursor(0, 16 + i * 12);
    display.print(options[i]);
  }
  display.display();
}

void resetRadio() {
  if (!radioActive) {
    nrf.begin();
    nrf.setPALevel(NRF_PA_MAX);
    nrf.setAutoAck(true);
    nrf.setRetries(1, 10);
    nrf.setChannel(esbChannel);
    nrf.stopListening();
    radioActive = true;
  }
  lastActivityTime = millis();
}

void idleRadioCheck() {
  if (radioActive && millis() - lastActivityTime > IDLE_TIMEOUT_MS) {
    nrf.powerDown();
    radioActive = false;
  }
}

void handleSetupInput() {
  if (selectedSetting == 0) {
    for (uint8_t i = 0; i < 4; i++) {
      if (!digitalRead(buttonPins[i])) {
        selectedSetting = i + 1;
        delay(300);
      }
    }

    if (selectedSetting == 0) return;

    const char* options1[4] = {"Ch 5", "Ch 10", "Ch 15", "Ch 20"};
    const char* options2[4] = {"Layer 1", "Layer 2", "Layer 3", "Layer 4"};
    const char* options3[4] = {"MIDI Ch 1", "MIDI Ch 2", "MIDI Ch 3", "MIDI Ch 4"};
    const char* options4[4] = {"Momentary", "Toggle", "-", "-"};

    switch (selectedSetting) {
      case 1: drawSubMenu("2.4GHz Channel", options1); break;
      case 2: drawSubMenu("CC Layer", options2); break;
      case 3: drawSubMenu("MIDI Channel", options3); break;
      case 4: drawSubMenu("Button Mode", options4); break;
    }
    delay(500);
  }

  static bool lastD0sub = true;
  bool currentD0sub = digitalRead(0);
  if (lastD0sub && !currentD0sub) {
    delay(20);
    if (!digitalRead(0)) {
      selectedSetting = 0;
      drawSetupMenu();
      return;
    }
  }
  lastD0sub = currentD0sub;

  for (uint8_t i = 0; i < 4; i++) {
    if (!digitalRead(buttonPins[i])) {
      switch (selectedSetting) {
        case 1: esbChannel = 5 + i * 5; break;
        case 2: ccLayer = i + 1; break;
        case 3: midiChannel = i + 1; break;
        case 4: buttonMode = i; break;
      }
      nrf.setChannel(esbChannel);
      nrf.stopListening();
      delay(300);
      drawSetupMenu();
      selectedSetting = 0;
      return;
    }
  }

  if (millis() - setupStartTime > SETUP_TIMEOUT_MS) {
    setupMode = false;
    selectedSetting = 0;
    needsOledRefresh = true;
  }
}

void sendCC(uint8_t cc, uint8_t value) {
  if (setupMode) return;
  resetRadio();
  uint8_t msg[3] = {uint8_t(0xB0 + (midiChannel - 1)), cc, value};
  uint8_t packet[4] = {0x0B, msg[0], msg[1], msg[2]};

  if (TinyUSBDevice.mounted()) {
    tud_midi_stream_write(0, packet, 4);
  } else {
    uint8_t buffer[2] = {cc, value};
    nrf.write(buffer, 2);
  }
}

void setup() {
  TinyUSBDevice.begin();
  delay(200);

  Wire1.setPins(8, 9);
  Wire1.begin();
  Wire.setPins(6, 7);
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  adsConnected = ads.begin(0x48, &Wire1);
  if (!adsConnected) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("## ADS disconnected");
    display.setCursor(0, 16);
    display.print("please reboot");
    display.display();
    for (int i = 0; i < 10; i++) {
      digitalWrite(LED_BUILTIN, i % 2);
      delay(250);
    }
  } else {
    ads.setGain(GAIN_ONE);
    ads.setDataRate(RATE_ADS1115_860SPS);
  }

  NRF_POWER->DCDCEN = 1;
  usb_midi.begin();
  nrf.begin();
  nrf.setPALevel(NRF_PA_MAX);
  nrf.setAutoAck(true);
  nrf.setRetries(0, 0);
  nrf.setChannel(esbChannel);
  nrf.stopListening();
  lastActivityTime = millis();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(0, INPUT_PULLUP);
  for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  lastUSBState = TinyUSBDevice.mounted();
  drawOLEDStatus(lastUSBState);
}

void loop() {
  bool currentUSB = TinyUSBDevice.mounted();
  if (currentUSB != lastUSBState) {
    lastUSBState = currentUSB;
    needsOledRefresh = true;

    if (!currentUSB) {
      delay(20);
      nrf.begin();
      nrf.setPALevel(NRF_PA_MAX);
      nrf.setAutoAck(true);
      nrf.setRetries(0, 0);
      nrf.setChannel(esbChannel);
      nrf.stopListening();
      radioActive = true;
    }
  }

  static bool lastD0 = true;
  bool currentD0 = digitalRead(0);
  if (lastD0 && !currentD0) {
    delay(20);
    if (!digitalRead(0)) {
      if (setupMode && selectedSetting == 0) {
        setupMode = false;
        needsOledRefresh = true;
      } else {
        setupMode = true;
        selectedSetting = 0;
        drawSetupMenu();
      }
    }
  }
  lastD0 = currentD0;

  if (setupMode) {
    handleSetupInput();
    return;
  }

  if (adsConnected) {
    for (uint8_t i = 0; i < ANALOG_CHANNEL_COUNT; i++) {
      int16_t raw = ads.readADC_SingleEnded(i);
      float voltage = ads.computeVolts(raw);
      float norm = constrain(voltage / VREF, 0.0, 1.0);
      float scaled = norm * 127.0;
      uint8_t rounded = round(scaled);
      int diff = rounded - lastSent[i];
      if (abs(diff) > 1 ||
          (diff == 1 && scaled > (lastSent[i] + HYSTERESIS)) ||
          (diff == -1 && scaled < (lastSent[i] - HYSTERESIS))) {
        lastSent[i] = rounded;
        sendCC(28 + i, rounded);
      }
    }
  }

  for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
    bool state = !digitalRead(buttonPins[i]);
    if (buttonMode == 0) {
      if (state != lastButtonState[i]) {
        lastButtonState[i] = state;
        sendCC(20 + i, state ? 127 : 0);
        needsOledRefresh = true;
      }
    } else {
      if (state && !lastButtonState[i]) {
        buttonToggles[i] = !buttonToggles[i];
        sendCC(20 + i, buttonToggles[i] ? 127 : 0);
        needsOledRefresh = true;
      }
      lastButtonState[i] = state;
    }
  }

  idleRadioCheck();
  updateLEDBlink(currentUSB);

  if (needsOledRefresh) {
    drawOLEDStatus(currentUSB);
  }
}
