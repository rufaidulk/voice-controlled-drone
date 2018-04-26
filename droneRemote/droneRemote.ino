#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>

RF24 radio(9,10); // CE, CSN
const uint64_t pipe = 0xE8E8F0F0E1LL; // Needs to be the same for communicating between 2 NRF24L01 
int msg[1];
String text="";

const int VRxPin= 0; //VRx pin connected to arduino pin A0
const int VRyPin= 1; //VRy pin connected to arduino in A1
const int VRxPin_1= 4; //VRx pin connected to arduino pin A2
const int SwButtonPin= 8; //SW pin connected to arduino pin D8

int pressed= -1; //this variable will determine whether joystick has been pressed down (selected)
int x= -1;//this variable will hold the X-coordinate value
int y= -1; //this variable will hold the Y-coordinate value
int z= -1; //this variable will hold the Y-coordinate value

void readJoystick() {
pressed= digitalRead(SwButtonPin);//reads whether joystick has been pressed down (selected) or not
x= analogRead(VRxPin);//reads the X-coordinate value
y= analogRead(VRyPin);//reads the Y-coordinate value
z= analogRead(VRxPin_1);//reads the Z-coordinate value
}

void setup() {
pinMode(SwButtonPin, INPUT);//sets the SW switch as input
//digitalWrite(SwButtonPin, HIGH);//sets the SW button HIGH
Serial.begin(9600);//sets the baud rate
radio.begin(); // Start the NRF24L01
radio.openWritingPipe(pipe); // Get NRF24L01 ready to transmit
//radio.setPayloadSize(1);
//radio.setPALevel(RF24_PA_HIGH); 
//radio.setDataRate(RF24_250KBPS);
}

void loop() {
readJoystick();//calls this function which reads the digital input button SW, the X-coordinate and the Y-coordinate
Serial.println("X: ");
delay(1000);
Serial.println(x);//prints the X-coordinate
delay(1000);
Serial.println("Y: ");
delay(1000);
Serial.println(y);//prints the Y-coordinate
delay(1000);
Serial.println("Z: ");
delay(1000);
Serial.println(z);//prints the Z-coordinate
delay(1000);

//condition
if(x<450){ text = "left";}
else if(x>510){ text="right";}
else if(y>510){ text="back";}
else if(y<450){ text="front";}
else if(z<450){ text="up";}
else if(z>510){ text="down";}
else{ text ="";}

//transmission
if (text.length() > 0) {
      int messageSize = text.length();
      for (int i = 0; i < messageSize; i++) {
        int charToSend[1];
        charToSend[0] = text.charAt(i);
        radio.write(charToSend,1);
        }  
//send the 'terminate string' value...  
      msg[0] = 2; 
      radio.write(msg,1);
      radio.powerDown(); 
      delay(1000);
      radio.powerUp();

      Serial.println(text);
      text="";
      }

/*
Serial.println(" Pressed: ");
delay(1000);
Serial.println(pressed);//prints whether joystick knob has been pressed or not
delay(1000);*/
}

