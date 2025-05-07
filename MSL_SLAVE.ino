#define byte uint8_t  // Fix ambiguity
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h> 
#include <AES.h>
#include <AESLib.h>
#include <AES_config.h>
#include <string.h>

RF24 radio(21, 22); // CE, CSN

uint8_t txAddress[] = "gndrec";
uint8_t rxAddress[] = "mslrec";

AES aes;
uint8_t recv_req[32];
uint8_t msg_out[32];

String key_str = "yeahhhhbabyyyyyy";  // 16 bytes
String iv_str  = "bigbyombomanss77";  // 16 bytes

void do_encrypt(uint8_t msg[32], String key_str, String iv_str) {
  uint8_t iv[16];
  memcpy(iv, iv_str.c_str(), 16);

  uint8_t key[16];
  memcpy(key, key_str.c_str(), 16);

  aes.set_key(key, 16);
  aes.cbc_encrypt(msg, msg_out, 2, iv);  // output to msg_out
}

void do_decrypt(uint8_t cipher[32], String key_str, String iv_str) {
  uint8_t iv[16];
  memcpy(iv, iv_str.c_str(), 16);

  uint8_t key[16];
  memcpy(key, key_str.c_str(), 16);

  aes.set_key(key, 16);
  aes.cbc_decrypt(cipher, msg_out, 2, iv);
}

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.openWritingPipe(rxAddress);
  radio.openReadingPipe(1, txAddress);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    radio.read(&recv_req, sizeof(recv_req));

    Serial.print("Got encrypted: ");
    for (int i = 0; i < 32; i++) {
      Serial.print("0x");
      Serial.print(recv_req[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    do_decrypt(recv_req, key_str, iv_str);

    Serial.print("Decrypted: ");
    for (int i = 0; i < 32; i++) {
      Serial.print("0x");
      Serial.print(msg_out[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    radio.stopListening();
    delay(80); // Simulate processing
    radio.write(&recv_req, sizeof(recv_req));
    Serial.println("Sent DATA_REPLY");
    delayMicroseconds(150);
    radio.startListening();
  }
}
