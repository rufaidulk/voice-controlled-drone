#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>


RF24 radio(9,10);
const uint64_t pipe = 0xE8E8F0F0E1LL; // Needs to be the same for communicating between 2 NRF24L01 

int msg[1];
char message[50];
String voice;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  radio.begin(); // Start the NRF24L01
  radio.openWritingPipe(pipe); // Get NRF24L01 ready to transmit
  
}

void loop() {

   while (Serial.available()) { //Check if there is an available byte to read
    delay(10); //Delay added to make thing stable
    char c = Serial.read(); //Conduct a serial read
    if (c == '#') {
      break; //Exit the loop when the # is detected after the word
    }
    voice += c; //Shorthand for voice = voice + c
    //voice.toCharArray(message, 50);
    
  }
    if (voice.length() > 0) {
      int messageSize = voice.length();
      for (int i = 0; i < messageSize; i++) {
        int charToSend[1];
        charToSend[0] = voice.charAt(i);
        radio.write(charToSend,1);
        }  
//send the 'terminate string' value...  
      msg[0] = 2; 
      radio.write(msg,1);
      radio.powerDown(); 
      delay(1000);
      radio.powerUp();

      Serial.println(voice);
      voice="";
      }
      /*
      //----------Control motors----------//
      if (voice == "*start motors") {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(3000);
      }
      else if (voice == "*fly low") {
      digitalWrite(LED_BUILTIN, LOW);
      delay(3000);
      }*/
      
    
}

