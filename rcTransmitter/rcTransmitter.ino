//Constant variables relating to pin locations
const int chA=8;  //roll
const int chB=9;  //pitch
//const int chC=10; //throttle
const int chD=11; //yaw

//Varibles to store and display the values of each channel
int ch1;  //roll
int ch2;  //pitch
int ch3;  //throttle
int ch4;  //yaw


//================================================//
//min pitch=1250 and max pitch=1900
//min roll=1100 and max roll=1900
//min yaw=1050 and max yaw=1800
//min throttle=1150 and max throttle=1900


// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // Set input pins
  pinMode(chA,INPUT);
  pinMode(chB,INPUT);
//  pinMode(chC,INPUT);
  pinMode(chD,INPUT);

}

//Main Program
void loop() {
  // read the input channels
  ch1 = pulseIn (chA,HIGH);  //Read and store channel 1
  Serial.print ("roll:");  //Display text string on Serial Monitor to distinguish variables
  Serial.print (ch1);     //Print in the value of channel 1
  Serial.println ("|");
   delay(1000);

  ch2 = pulseIn (chB,HIGH);
  Serial.print ("pitch");
  Serial.print (ch2);
  Serial.println ("|");
   delay(1000);

  /*ch3 = pulseIn (chC,HIGH);
  Serial.print ("Ch3:");
  Serial.print (ch3);
  Serial.println ("|");
   delay(1000);*/

  ch4 = pulseIn (chD,HIGH);
  Serial.print ("yaw");
  Serial.print (ch4);
  Serial.println ("|");
  delay(1000);

}
