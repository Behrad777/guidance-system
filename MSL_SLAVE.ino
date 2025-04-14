#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h> 

using namespace std;
char recv_req[32];
//msl on arduino
RF24 radio(9, 10); //CE CSN pins to arduino, needs to be esp32 for more current
uint8_t txAddress[] = "gndrec";
uint8_t rxAddress[] = "mslrec";

void setup() {
  Serial.begin(115200);
  //init radio settings
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.openWritingPipe(rxAddress);
  radio.openReadingPipe(1, txAddress);
  radio.startListening();
  radio.setAutoAck(1, true);

}

void loop() {
  if (radio.available()) {
    radio.read(&recv_req, sizeof(recv_req));
    Serial.print("Got: ");
    Serial.println(recv_req);

    radio.stopListening();
    delayMicroseconds(150); 
    const char response[] = "DATA_REPLY";
    Serial.println("Sent DATA_REPLY");
    radio.write(&recv_req, sizeof(recv_req)); 
    delayMicroseconds(150);
    radio.startListening();
  }
} 

