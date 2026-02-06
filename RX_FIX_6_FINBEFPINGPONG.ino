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
const uint8_t ESB_PAYLOAD_MAX = 32;
const uint8_t ESB_COMMAND_DATA_MAX = 30;
const uint8_t ESB_RESPONSE_DATA_MAX = 28;

const uint8_t ESB_TYPE_CC = 0x01;
const uint8_t ESB_TYPE_CMD = 0x02;
const uint8_t ESB_TYPE_RESP = 0x03;
const uint8_t ESB_TYPE_PING = 0x04;

const uint8_t COMMAND_QUEUE_SIZE = 10;
char commandQueue[COMMAND_QUEUE_SIZE][ESB_COMMAND_DATA_MAX + 1];
uint8_t commandQueueHead = 0;
uint8_t commandQueueTail = 0;

char responseBuffer[512];
uint16_t responseBufferIndex = 0;
uint8_t expectedResponseChunk = 0;
uint8_t expectedResponseChunkCount = 0;

GlobalSettings globalSettings;

uint8_t buffer[32];
unsigned long lastPacketTime = 0;

bool isCommandQueueFull() {
    return ((commandQueueHead + 1) % COMMAND_QUEUE_SIZE) == commandQueueTail;
}

bool isCommandQueueEmpty() {
    return commandQueueHead == commandQueueTail;
}

bool enqueueCommand(const char *command) {
    if (isCommandQueueFull()) return false;
    strncpy(commandQueue[commandQueueHead], command, ESB_COMMAND_DATA_MAX);
    commandQueue[commandQueueHead][ESB_COMMAND_DATA_MAX] = '\0';
    commandQueueHead = (commandQueueHead + 1) % COMMAND_QUEUE_SIZE;
    return true;
}

bool dequeueCommand(char *outCommand) {
    if (isCommandQueueEmpty()) return false;
    strncpy(outCommand, commandQueue[commandQueueTail], ESB_COMMAND_DATA_MAX);
    outCommand[ESB_COMMAND_DATA_MAX] = '\0';
    commandQueueTail = (commandQueueTail + 1) % COMMAND_QUEUE_SIZE;
    return true;
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
    nrf.enableDynamicPayloads(123);
    nrf.enableAckPayload();
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
    nrf.enableDynamicPayloads(123);
    nrf.enableAckPayload();
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
        } else {
            String command = String(cmd) + webusb.readStringUntil('\n');
            command.trim();
            if (!enqueueCommand(command.c_str())) {
                if (webusb.connected()) {
                    webusb.print("error:queue_full\n");
                    webusb.flush();
                }
            }
        }
    }

    // --- HANDLE NRF DATA ---
    if (nrf.available()) {
        while (nrf.available()) {
            uint8_t bytes = nrf.getDynamicPayloadSize();
            if (bytes == 0 || bytes > sizeof(buffer)) bytes = sizeof(buffer);
            nrf.read(buffer, bytes);
            lastPacketTime = millis();

            uint8_t payloadType = buffer[0];
            if (payloadType == ESB_TYPE_CC && bytes >= 5) {
                sendCC(buffer[2], buffer[3], buffer[4]);
                if (webusb.connected()) {
                    webusb.print("cc:" + String(buffer[3]) + "," + String(buffer[4]) + "\n");
                    webusb.flush();
                }
                digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            } else if (payloadType == ESB_TYPE_RESP && bytes >= 4) {
                uint8_t chunkIndex = buffer[1];
                uint8_t chunkCount = buffer[2];
                uint8_t dataLen = buffer[3];
                if (dataLen > bytes - 4) dataLen = bytes - 4;

                if (chunkIndex == 0) {
                    responseBufferIndex = 0;
                    expectedResponseChunk = 0;
                    expectedResponseChunkCount = chunkCount;
                }

                if (chunkIndex != expectedResponseChunk) {
                    responseBufferIndex = 0;
                    expectedResponseChunk = 0;
                    expectedResponseChunkCount = chunkCount;
                    continue;
                }

                if (responseBufferIndex + dataLen < sizeof(responseBuffer)) {
                    memcpy(responseBuffer + responseBufferIndex, buffer + 4, dataLen);
                    responseBufferIndex += dataLen;
                }

                expectedResponseChunk++;
                if (expectedResponseChunk >= expectedResponseChunkCount) {
                    responseBuffer[responseBufferIndex] = '\0';
                    if (strncmp(responseBuffer, "esb_channel_updated:", 20) == 0) {
                        int newChannel = atoi(responseBuffer + 20);
                        if (newChannel != globalSettings.esbChannel) {
                            globalSettings.esbChannel = newChannel;
                            saveSettings();
                            resetRadio();
                        }
                    }
                    if (webusb.connected()) {
                        webusb.print(responseBuffer);
                        webusb.flush();
                    }
                    responseBufferIndex = 0;
                    expectedResponseChunk = 0;
                    expectedResponseChunkCount = 0;
                }
            }

            uint8_t ackPayload[ESB_PAYLOAD_MAX];
            uint8_t ackLength = 2;
            if (!isCommandQueueEmpty()) {
                char commandBuffer[ESB_COMMAND_DATA_MAX + 1];
                if (dequeueCommand(commandBuffer)) {
                    uint8_t commandLength = strlen(commandBuffer);
                    ackPayload[0] = ESB_TYPE_CMD;
                    ackPayload[1] = commandLength;
                    memcpy(ackPayload + 2, commandBuffer, commandLength);
                    ackLength = 2 + commandLength;
                }
            } else {
                ackPayload[0] = ESB_TYPE_PING;
                ackPayload[1] = 0;
            }
            nrf.writeAckPayload(1, ackPayload, ackLength);
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
