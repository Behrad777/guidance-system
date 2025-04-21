#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino.h>
#include <AES.h>
#include <AESLib.h>
#include <AES_config.h>
#include <string.h>
#include <Base64.h>

//can receive a maximum of 33 byte payload
//ground on esp32
RF24 radio(21, 22); // CE, CSN
AES aes;
uint8_t txAddress[] = "gndrec";
uint8_t rxAddress[] = "mslrec";
char reply[32];

byte cipher[32];
byte msg_out[32];
String key="yeahhhhbabyyyyyy";// 16 bytes
String iv="bigbyomboballs77"; //16 bytes

void do_encrypt(byte msg[32], String key_str, String iv_str) {
  byte iv[16];
  memcpy(iv, iv_str.c_str(), 16);

  byte key[16];
  memcpy(key, key_str.c_str(), 16);

  aes.set_key(key, 16);
  aes.cbc_encrypt(msg, cipher, 2, iv);  // 32 bytes = 2 AES blocks
}

void do_decrypt(byte cipher[32], String key_str, String iv_str) {
  byte iv[16];
  memcpy(iv, iv_str.c_str(), 16);

  byte key[16];
  memcpy(key, key_str.c_str(), 16);

  aes.set_key(key, 16);
  aes.cbc_decrypt(cipher, msg_out, 2, iv);  // 2 blocks = 32 bytes
}

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.openReadingPipe(1, rxAddress);
  radio.openWritingPipe(txAddress);
}

void loop() {


  radio.stopListening();
  
  if(Serial.available()){
    String input = Serial.readStringUntil('\n');
    char request[33];
    input.toCharArray(request, sizeof(request));

    byte msg[32] = {
        0xDE, 0xAD, 0xBE, 0xEF, 0x11, 0x22, 0x33, 0x44,
        0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC,
        0x01, 0x02, 0x03, 0x04, 0xA0, 0xB0, 0xC0, 0xD0,
        0xE0, 0xF0, 0x00, 0x10, 0x20, 0x30, 0x40, 0x50
      };
    
    do_encrypt(msg, key, iv);
    radio.write(&cipher, sizeof(cipher));
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
          for (int i = 0; i < 32; i++) {  //always 32 byte receive 
            Serial.print("0x");
            Serial.print(reply[i], HEX);
            Serial.print(" ");
          }
          Serial.println();
        timeout = false;
        break;
      }
    }

    if (timeout) {
      Serial.println("No reply received-------------------------------------");
    }

}
  
}
