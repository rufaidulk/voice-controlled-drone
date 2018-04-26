#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>

RF24 radio(9,10);
const uint64_t pipe = 0xE8E8F0F0E1LL; // Needs to be the same for communicating between 2 NRF24L01 
char recieveMess[50]="";
String theMessage = "";
int msg[1];

void setup(void){
bool done = false;
Serial.begin(9600);
radio.begin(); // Start the NRF24L01
//radio.printDetails();
radio.openReadingPipe(1,pipe); // Get NRF24L01 ready to receive
//radio.setPALevel(RF24_PA_MIN);
radio.startListening(); // Listen to see if information received
//Serial.println(done);
}

void loop(void){
  if (radio.available()){
    bool done = false;  
      radio.read(msg, 1); 
      char theChar = msg[0];
      if (msg[0] != 2){
        theMessage.concat(theChar);
        }
      else {
       Serial.println(theMessage);
       theMessage= ""; 
      }
}
}

