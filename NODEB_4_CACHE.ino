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
  uint8_t type;     
  uint8_t msgID;    
  uint8_t ackID;    
  char message[64]; 
};

PayloadStruct rxData;      
PayloadStruct ackPayload;  

// --- QUEUE SYSTEM ---
#define MAX_QUEUE 10
char msgQueue[MAX_QUEUE][64]; // Can hold 10 messages (commands)
int queueHead = 0; // Where we write new msgs
int queueTail = 0; // Where we read msgs to send
uint8_t currentMsgID = 0; // The ID currently assigned to the text at queueTail

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  if (!radio.begin()) {
    Serial.println(F("Radio hardware not responding!"));
    while (1) {}
  }

  radio.setPALevel(NRF_PA_MAX);
  radio.setDataRate(NRF_2MBPS);

  radio.enableDynamicPayloads(123);
  radio.enableAckPayload();

  radio.openReadingPipe(1, address[1]); 
  radio.startListening(); 

  // Load initial empty ping so radio is ready
  ackPayload.type = PAYLOAD_PING;
  ackPayload.msgID = 0; 
  ackPayload.ackID = 0;
  radio.writeAckPayload(1, &ackPayload, 3);
  
  Serial.println(F("--- RX NODE B (With Caching) ---"));
  Serial.println(F("Web commands will be queued if Node A is offline."));
}

void loop() {
  // --- 1. HANDLE USER INPUT (ADD COMMAND TO QUEUE) ---
  if (Serial.available()) {
    // Check if queue is full
    int nextHead = (queueHead + 1) % MAX_QUEUE;
    if (nextHead != queueTail) {
      int len = Serial.readBytesUntil('\n', msgQueue[queueHead], 63);
      msgQueue[queueHead][len] = '\0';
      
      Serial.print(F("[Queued Command]: "));
      Serial.println(msgQueue[queueHead]);
      
      queueHead = nextHead; // Advance head
    } else {
      Serial.println(F("[Error] Queue Full! Wait for Node A."));
    }
  }

  // --- 2. HANDLE RADIO TRAFFIC ---
  uint8_t pipe;
  if (radio.available(&pipe)) {
    
    // A. Read what Node A sent
    uint8_t bytes = radio.getDynamicPayloadSize();
    if (bytes > sizeof(rxData)) bytes = sizeof(rxData);
    radio.read(&rxData, bytes);

    if (rxData.type == PAYLOAD_TEXT) {
      Serial.print(F("RX from A: "));
      Serial.println(rxData.message);
    } else if (rxData.type == PAYLOAD_RESPONSE) {
      Serial.print(F("Response from A: "));
      Serial.println(rxData.message);
    }

    // --- B. MANAGE OUTGOING QUEUE ---
    
    // Check if Node A confirmed our last message?
    if (currentMsgID != 0 && rxData.ackID == currentMsgID) {
       Serial.print(F("   (Node A confirmed ID "));
       Serial.print(currentMsgID);
       Serial.println(F(")"));
       
       // Success! Remove from queue.
       currentMsgID = 0; 
       queueTail = (queueTail + 1) % MAX_QUEUE;
    }

    // --- C. PREPARE NEXT ACK PAYLOAD ---
    
    if (queueTail != queueHead) { 
      // We have data waiting in queue!
      
      // If we haven't assigned an ID yet, give it one
      if (currentMsgID == 0) {
        static uint8_t globalCounter = 1;
        currentMsgID = globalCounter++;
        if (currentMsgID == 0) currentMsgID = 1; // Skip 0
      }

      // Load data from Queue into ACK
      ackPayload.type = PAYLOAD_COMMAND;
      ackPayload.msgID = currentMsgID;
      strcpy(ackPayload.message, msgQueue[queueTail]);
      
      // Tell A what we received from it (Echo back its ID)
      ackPayload.ackID = rxData.msgID;

      uint8_t size = 3 + strlen(ackPayload.message) + 1;
      radio.writeAckPayload(1, &ackPayload, size);
      
    } else {
      // Queue empty, send Idle Ping
      ackPayload.type = PAYLOAD_PING;
      ackPayload.msgID = 0;
      ackPayload.ackID = rxData.msgID; // Still need to Ack A's messages
      ackPayload.message[0] = '\0';
      
      radio.writeAckPayload(1, &ackPayload, 3); 
    }
  }
}
