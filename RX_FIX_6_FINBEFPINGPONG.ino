#include <Arduino.h>
#include "Adafruit_TinyUSB.h"
#include "nrf_to_nrf.h"
#include "InternalFileSystem.h"
#include "flash/flash_nrf5x.h"

Adafruit_USBD_MIDI usb_midi;
Adafruit_USBD_WebUSB webusb;
nrf_to_nrf nrf;

WEBUSB_URL_DEF(landingPage, 1, "diningwork.space/");

struct GlobalSettings {
    uint8_t esbChannel;
    uint8_t activeBank;
    uint8_t brightness;
    bool autoCalEnabled;
    int16_t autoCalMaxValue;
};

// 0x7F000 is the address being used.
// Ensure this does not overlap with your bootloader.
#define GLOBAL_SETTINGS_ADDR 0x7F000
const unsigned long LATENCY_LIMIT = 60000UL; 

GlobalSettings globalSettings;

uint8_t buffer[32];
unsigned long lastPacketTime = 0;

void saveSettings() {
    // 1. Erase the page (Reset to 0xFF)
    flash_nrf5x_erase((uint32_t)GLOBAL_SETTINGS_ADDR);
    
    // 2. Write the new data
    flash_nrf5x_write((uint32_t)GLOBAL_SETTINGS_ADDR, &globalSettings, sizeof(GlobalSettings));
    
    // 3. Flush to commit
    flash_nrf5x_flush();
}

void loadSettings() {
    // Read directly.
    flash_nrf5x_read(&globalSettings, (uint32_t)GLOBAL_SETTINGS_ADDR, sizeof(GlobalSettings));
    
    // Validate: If flash is erased (0xFF) or invalid, set defaults
    bool invalid = (globalSettings.esbChannel > 100) ||
                   (globalSettings.esbChannel == 0xFF) ||
                   (globalSettings.activeBank >= 12) ||  // Assuming NUM_BANKS=12 from TX
                   (globalSettings.brightness < 3 || globalSettings.brightness > 255);
    
    if (invalid) {
        // Sane defaults (e.g., channel 42 as in TX example)
        globalSettings.esbChannel = 42;
        globalSettings.activeBank = 0;
        globalSettings.brightness = 200;
        globalSettings.autoCalEnabled = false;
        globalSettings.autoCalMaxValue = 16383;
        saveSettings();  // Save defaults immediately
    }
}

void resetRadio() {
    nrf.powerDown();
    delay(10);
    nrf.begin();
    nrf.setPALevel(NRF_PA_MAX);
    nrf.setAutoAck(true);
    nrf.setRetries(15, 15);
    nrf.setChannel(globalSettings.esbChannel);
    nrf.startListening();
}

void setup() {
    TinyUSBDevice.setID(0x000a, 0x001a);
    usb_midi.begin();
    webusb.setLandingPage(&landingPage);
    webusb.setLineStateCallback(line_state_callback);  // If needed, define this
    webusb.begin();
    
    // 1. Load from flash
    loadSettings(); 

    // 2. Set channel immediately.
    nrf.begin();
    nrf.setPALevel(NRF_PA_MAX);
    nrf.setAutoAck(true);
    nrf.setRetries(15, 15);
    nrf.setChannel(globalSettings.esbChannel); 
    nrf.startListening();

    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    // --- HANDLE WEBUSB COMMANDS ---
    if (webusb.available()) {
        char cmd = webusb.read();
        
        // COMMAND 'C': Change Channel
        if (cmd == 'C') {
            unsigned long timeout = millis();
            while (!webusb.available() && millis() - timeout < 10);

            if (webusb.available()) {
                uint8_t newCh = webusb.read();
                
                // Only save if different (prevents wearing out flash)
                if (newCh != globalSettings.esbChannel) {
                    globalSettings.esbChannel = newCh;
                    
                    // 1. Update Radio
                    nrf.setChannel(newCh);
                    nrf.startListening();
                    
                    // 2. Save to Flash (Erase + Write + Flush)
                    saveSettings(); 
                    
                    // 3. Reset radio to apply fully
                    resetRadio();
                }
            }
        } 
        // COMMAND 'R': Report Status
        else if (cmd == 'R') {
            unsigned long latency = millis() - lastPacketTime;
            if (latency > LATENCY_LIMIT) {
                lastPacketTime = millis();
                latency = 0;
            }

            if (webusb.connected()) {
                int8_t rssi = -nrf.getRSSI();
                webusb.print("{\"rssi\":" + String(rssi) + ",\"latency\":" + String(latency) + "}\n");
                webusb.flush();
            }
        }
    }

    // --- HANDLE NRF DATA ---
    if (nrf.available()) {
        while (nrf.available()) {
            nrf.read(buffer, 3);
            sendCC(buffer[0], buffer[1], buffer[2]);
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            lastPacketTime = millis();
        }
    }
}



void sendCC(uint8_t status, uint8_t cc, uint8_t val) {
    uint8_t packet[] = {0x0B, status, cc, val};
    if (TinyUSBDevice.mounted()) {
        usb_midi.writePacket(packet); 
    }
}

void line_state_callback(bool connected) {
    if (connected) {
        // Send ESB channel once on connect
        webusb.print("{\"esbChannel\":" + String(globalSettings.esbChannel) + "}\n");
        webusb.flush();
    }
}