#include "nrf_to_nrf.h"
#include <SPI.h>

nrf_to_nrf radio;

uint8_t address[][6] = { "1Node", "2Node" };

struct PayloadStruct {
  uint8_t type;       // 0=PING, 1=TEXT
  uint8_t msgID;      // My ID
  uint8_t ackID;      // Confirmation ID
  char message[64];   // Big buffer
};

PayloadStruct txPayload; 
PayloadStruct rxPayload;

uint8_t myMsgCounter = 1;      // Tracking my messages
uint8_t lastReceivedID = 0;    // Tracking what I got from B

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  if (!radio.begin()) {
    Serial.println(F("Radio hardware not responding!"));
    while (1) {}
  }

  radio.setPALevel(NRF_PA_MAX);
  radio.setDataRate(NRF_2MBPS);
  
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
  txPayload.type = 0; 
  txPayload.message[0] = '\0';
  
  // If user typed something, change to TEXT
  if (Serial.available()) {
    txPayload.type = 1; // Real Text
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
      
      // If it's a new TEXT message (and not one we already saw)
      if (rxPayload.type == 1 && rxPayload.msgID != lastReceivedID) {
        Serial.print(F("  -> [RX from B] ID:")); 
        Serial.print(rxPayload.msgID);
        Serial.print(F(" Msg: "));
        Serial.println(rxPayload.message);

        // Update our record so we confirm it next time
        lastReceivedID = rxPayload.msgID;
      }
    }
  }

  // Small delay to allow B to process
  delay(5); 
}