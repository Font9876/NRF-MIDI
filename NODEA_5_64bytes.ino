#include "nrf_to_nrf.h"
#include <SPI.h>

nrf_to_nrf radio;

uint8_t address[][6] = { "1Node", "2Node" };

enum PayloadType : uint8_t {
  PAYLOAD_PING = 0,
  PAYLOAD_TEXT = 1,
  PAYLOAD_COMMAND = 2,
  PAYLOAD_RESPONSE = 3
};

struct PayloadStruct {
  uint8_t type;       // PayloadType
  uint8_t msgID;      // My ID
  uint8_t ackID;      // Confirmation ID
  char message[64];   // Big buffer
};

PayloadStruct txPayload; 
PayloadStruct rxPayload;

uint8_t myMsgCounter = 1;      // Tracking my messages
uint8_t lastReceivedID = 0;    // Tracking what I got from B
char pendingResponse[64];
bool hasPendingResponse = false;

uint8_t currentDataRate = NRF_2MBPS;
uint8_t currentPALevel = NRF_PA_MAX;

void setDataRate(uint8_t rate) {
  currentDataRate = rate;
  radio.setDataRate(rate);
}

void setPALevel(uint8_t level) {
  currentPALevel = level;
  radio.setPALevel(level);
}

void handleCommand(const char *command, char *response, size_t responseSize) {
  if (strncmp(command, "SET_RATE ", 9) == 0) {
    if (strcmp(command + 9, "1") == 0) {
      setDataRate(NRF_1MBPS);
      snprintf(response, responseSize, "OK SET_RATE 1");
      return;
    }
    if (strcmp(command + 9, "2") == 0) {
      setDataRate(NRF_2MBPS);
      snprintf(response, responseSize, "OK SET_RATE 2");
      return;
    }
    snprintf(response, responseSize, "ERR SET_RATE (use 1 or 2)");
    return;
  }

  if (strncmp(command, "SET_POWER ", 10) == 0) {
    if (strcmp(command + 10, "MIN") == 0) {
      setPALevel(NRF_PA_MIN);
      snprintf(response, responseSize, "OK SET_POWER MIN");
      return;
    }
    if (strcmp(command + 10, "LOW") == 0) {
      setPALevel(NRF_PA_LOW);
      snprintf(response, responseSize, "OK SET_POWER LOW");
      return;
    }
    if (strcmp(command + 10, "HIGH") == 0) {
      setPALevel(NRF_PA_HIGH);
      snprintf(response, responseSize, "OK SET_POWER HIGH");
      return;
    }
    if (strcmp(command + 10, "MAX") == 0) {
      setPALevel(NRF_PA_MAX);
      snprintf(response, responseSize, "OK SET_POWER MAX");
      return;
    }
    snprintf(response, responseSize, "ERR SET_POWER (MIN|LOW|HIGH|MAX)");
    return;
  }

  if (strcmp(command, "STATUS") == 0) {
    const char *rateLabel = (currentDataRate == NRF_1MBPS) ? "1" : "2";
    const char *powerLabel = "MAX";
    if (currentPALevel == NRF_PA_MIN) powerLabel = "MIN";
    else if (currentPALevel == NRF_PA_LOW) powerLabel = "LOW";
    else if (currentPALevel == NRF_PA_HIGH) powerLabel = "HIGH";
    snprintf(response, responseSize, "STATUS RATE=%s POWER=%s", rateLabel, powerLabel);
    return;
  }

  snprintf(response, responseSize, "ERR UNKNOWN_CMD");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  if (!radio.begin()) {
    Serial.println(F("Radio hardware not responding!"));
    while (1) {}
  }

  setPALevel(NRF_PA_MAX);
  setDataRate(NRF_2MBPS);
  
  // Enable large payloads
  radio.enableDynamicPayloads(123);
  radio.enableAckPayload();

  radio.openWritingPipe(address[1]); 
  radio.openReadingPipe(1, address[0]);
  radio.stopListening(); 
  
  Serial.println(F("--- TX NODE A (Master) ---"));
  Serial.println(F("Type to send. I will also Auto-Ping to pick up cached msgs from B."));
}

void loop() {
  // --- 1. PREPARE PACKET ---
  // Default to PING
  txPayload.type = PAYLOAD_PING; 
  txPayload.message[0] = '\0';
  
  if (hasPendingResponse) {
    txPayload.type = PAYLOAD_RESPONSE;
    txPayload.msgID = myMsgCounter++;
    strncpy(txPayload.message, pendingResponse, sizeof(txPayload.message) - 1);
    txPayload.message[sizeof(txPayload.message) - 1] = '\0';
    hasPendingResponse = false;
  } else if (Serial.available()) {
    txPayload.type = PAYLOAD_TEXT;
    txPayload.msgID = myMsgCounter++; 
    
    int len = Serial.readBytesUntil('\n', txPayload.message, 63);
    txPayload.message[len] = '\0';
    
    Serial.print(F("[Sending ID ")); Serial.print(txPayload.msgID);
    Serial.print(F("]: ")); Serial.println(txPayload.message);
  }

  // CRITICAL: Always tell B what we last received
  txPayload.ackID = lastReceivedID; 

  // --- 2. SEND ---
  // Calc size: type(1) + msgID(1) + ackID(1) + string + null(1)
  uint8_t sizeToSend = 3 + strlen(txPayload.message) + 1;

  if (radio.write(&txPayload, sizeToSend)) {
    
    // --- 3. CHECK RESPONSE ---
    if (radio.available()) {
      uint8_t bytes = radio.getDynamicPayloadSize();
      if (bytes > sizeof(rxPayload)) bytes = sizeof(rxPayload);
      radio.read(&rxPayload, bytes);
      
      if (rxPayload.msgID != lastReceivedID) {
        if (rxPayload.type == PAYLOAD_TEXT) {
          Serial.print(F("  -> [RX from B] ID:")); 
          Serial.print(rxPayload.msgID);
          Serial.print(F(" Msg: "));
          Serial.println(rxPayload.message);
        } else if (rxPayload.type == PAYLOAD_COMMAND) {
          char response[64];
          handleCommand(rxPayload.message, response, sizeof(response));
          strncpy(pendingResponse, response, sizeof(pendingResponse) - 1);
          pendingResponse[sizeof(pendingResponse) - 1] = '\0';
          hasPendingResponse = true;
        }

        // Update our record so we confirm it next time
        lastReceivedID = rxPayload.msgID;
      }
    }
  }

  // Small delay to allow B to process
  delay(5); 
}
