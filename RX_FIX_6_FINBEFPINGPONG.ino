#include <Arduino.h>
#include "Adafruit_TinyUSB.h"
#include "nrf_to_nrf.h"
#include "InternalFileSystem.h"
#include "flash/flash_nrf5x.h"

Adafruit_USBD_MIDI usb_midi;
Adafruit_USBD_WebUSB webusb;
nrf_to_nrf nrf;

WEBUSB_URL_DEF(landingPage, 1, "diningwork.space/");

const uint8_t ESB_BRIDGE_MAGIC = 0xFA;
const uint8_t ESB_BRIDGE_PAYLOAD_SIZE = 28;
const unsigned long ESB_BRIDGE_RESPONSE_TIMEOUT_MS = 250;

struct EsbBridgePacket {
    uint8_t magic;
    uint8_t msgId;
    uint8_t chunkIndex;
    uint8_t totalChunks;
    char payload[ESB_BRIDGE_PAYLOAD_SIZE];
};

uint8_t bridgeMsgId = 1;

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

bool collectBridgeResponse(uint8_t msgId, String& response) {
    response = "";
    uint8_t expectedChunks = 0;
    uint8_t receivedChunks = 0;
    unsigned long startTime = millis();

    while (millis() - startTime < ESB_BRIDGE_RESPONSE_TIMEOUT_MS) {
        if (!nrf.available()) {
            continue;
        }

        uint8_t bytes = nrf.getDynamicPayloadSize();
        if (bytes == 0) {
            continue;
        }

        if (bytes == 3) {
            nrf.read(buffer, 3);
            sendCC(buffer[0], buffer[1], buffer[2]);
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            lastPacketTime = millis();
            continue;
        }

        EsbBridgePacket packet = {};
        if (bytes > sizeof(packet)) bytes = sizeof(packet);
        nrf.read(&packet, bytes);

        if (packet.magic != ESB_BRIDGE_MAGIC || packet.msgId != msgId) {
            continue;
        }

        if (expectedChunks == 0) {
            expectedChunks = packet.totalChunks;
        }

        uint8_t payloadLength = bytes > 4 ? bytes - 4 : 0;
        for (uint8_t i = 0; i < payloadLength; i++) {
            response += packet.payload[i];
        }
        receivedChunks++;

        if (expectedChunks != 0 && receivedChunks >= expectedChunks) {
            return true;
        }
    }

    return false;
}

bool sendBridgeCommand(const String& command, String& response) {
    if (command.length() > ESB_BRIDGE_PAYLOAD_SIZE) {
        return false;
    }

    EsbBridgePacket packet = {};
    packet.magic = ESB_BRIDGE_MAGIC;
    packet.msgId = bridgeMsgId++;
    packet.chunkIndex = 0;
    packet.totalChunks = 1;
    command.toCharArray(packet.payload, ESB_BRIDGE_PAYLOAD_SIZE + 1);

    nrf.stopListening();
    nrf.write(&packet, 4 + command.length());
    nrf.startListening();

    return collectBridgeResponse(packet.msgId, response);
}

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
    nrf.enableDynamicPayloads(32);
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
    nrf.enableDynamicPayloads(32);
    nrf.setChannel(globalSettings.esbChannel); 
    nrf.startListening();

    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    // --- HANDLE WEBUSB COMMANDS ---
    if (webusb.available()) {
        char cmd = webusb.read();

        // COMMAND 'C': Change Channel (binary)
        if (cmd == 'C') {
            unsigned long timeout = millis();
            while (!webusb.available() && millis() - timeout < 10);

            if (webusb.available()) {
                uint8_t newCh = webusb.read();
                if (newCh != globalSettings.esbChannel) {
                    globalSettings.esbChannel = newCh;
                    nrf.setChannel(newCh);
                    nrf.startListening();
                    saveSettings(); 
                    resetRadio();
                }
            }
        } 
        // COMMAND 'R': Report Status (binary)
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
        } else if (cmd == '\n' || cmd == '\r') {
            return;
        } else {
            String command = String((char)cmd) + webusb.readStringUntil('\n');
            command.trim();
            if (command.length() > 0) {
                String response;
                if (sendBridgeCommand(command, response)) {
                    if (webusb.connected()) {
                        webusb.print(response + "\n");
                        webusb.flush();
                    }
                } else if (webusb.connected()) {
                    webusb.print("{\"error\":\"tx_timeout\"}\n");
                    webusb.flush();
                }
            }
        }
    }

    // --- HANDLE NRF DATA ---
    if (nrf.available()) {
        while (nrf.available()) {
            uint8_t bytes = nrf.getDynamicPayloadSize();
            if (bytes == 0) {
                continue;
            }
            if (bytes == 3) {
                nrf.read(buffer, 3);
                sendCC(buffer[0], buffer[1], buffer[2]);
                digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
                lastPacketTime = millis();
                continue;
            }
            EsbBridgePacket packet = {};
            if (bytes > sizeof(packet)) bytes = sizeof(packet);
            nrf.read(&packet, bytes);
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
