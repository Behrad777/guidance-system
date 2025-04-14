#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino.h>

//can receive a maximum of 33 byte payload
//ground on esp32
RF24 radio(21, 22); // CE, CSN
uint8_t txAddress[] = "gndrec";
uint8_t rxAddress[] = "mslrec";
char reply[32];

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.openReadingPipe(1, rxAddress);
  radio.openWritingPipe(txAddress);
  radio.setAutoAck(1, true);
}

void loop() {


  radio.stopListening();
  
  if(Serial.available()){
    String input = Serial.readStringUntil('\n');
    char request[33];
    input.toCharArray(request, sizeof(request));


    radio.write(&request, sizeof(request));
    Serial.print("Sent: ");
    Serial.println(input);
    
    delay(5);
    //wait for response
    radio.startListening();  
    unsigned long startTime = millis();
    bool timeout = true;

    //200ms timeout
    while (millis() - startTime < 350) {
      if (radio.available()) {
        radio.read(&reply, sizeof(reply));
        Serial.print("Received: ");
        Serial.println(reply);
        timeout = false;
        break;
      } 
    }

    if (timeout) {
      Serial.println("No reply received------------------Retry-------------------");
    }

}
  
}
