
#include<Servo.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>




// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//my variables
float angleRoll;
float anglePitch;
float angleYaw;

//pid variables
unsigned long lastTime;
double lastAnglePitch=0, lastAngleRoll=0, lastAngleYaw=0;
double pitch_ITerm, roll_ITerm;

double kp=1;
double ki=0;
double kd=0;


int SampleTime = 1000; //1 sec
int pitchOutMin=-400, pitchOutMax=400;
int rollOutMin=-400, rollOutMax=400;
int yawOutMin=-400, yawOutMax=400;
bool inAuto = false;

float desiredPitchAngle=2.35;
float desiredRollAngle=-1.86;
float desiredYawAngle=11.02;

double pitchOutput, rollOutput, yawOutput;

#define MANUAL 0
#define AUTOMATIC 1
 
#define DIRECT 0
#define REVERSE 1
int controllerDirection = DIRECT;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

//esc motor
Servo motor_1;
Servo motor_2;
Servo motor_3;
Servo motor_4;

int esc_1;
int esc_2;
int esc_3;
int esc_4;


RF24 radio(9,10);
const uint64_t pipe = 0xE8E8F0F0E1LL; // Needs to be the same for communicating between 2 NRF24L01 
char recieveMess[50]="";
String theMessage = "";
int msg[1];

//========================//
//Constant variables relating to pin locations
const int chA=8;  //roll
const int chB=9;  //pitch
//const int chC=10; //throttle
const int chD=11; //yaw

//Varibles to store and display the values of each channel
int ch1;  //roll
int ch2;  //pitch
//int ch3;  //throttle
int ch4;  //yaw
//================================================//
//min pitch=1300 and max pitch=1700
//min roll=1250 and max roll=1700
//min yaw=1050 and max yaw=1800
//min throttle=1150 and max throttle=1900

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void angleCalculate(){
  // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            angleYaw=ypr[0] * 180/M_PI;
            anglePitch=ypr[1] * 180/M_PI;
            angleRoll=ypr[2] * 180/M_PI;
            Serial.print("ypr\t");
            Serial.print(angleYaw);
            Serial.print("\t");
            Serial.print(anglePitch);
            Serial.print("\t");
            Serial.println(angleRoll);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
  }
  
void Compute()
{
   Serial.println("compute called");
   //if(!inAuto) return;
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      //pitch calculation
      /*Compute all the working error variables*/
      double pitch_error = desiredPitchAngle - anglePitch;
      pitch_ITerm+= (ki * pitch_error);
      if(pitch_ITerm > pitchOutMax) pitch_ITerm= pitchOutMax;
      else if(pitch_ITerm < pitchOutMin) pitch_ITerm= pitchOutMin;
      double pitch_dInput = (anglePitch - lastAnglePitch);
 
      /*Compute PID Output*/
      pitchOutput = kp * pitch_error + pitch_ITerm- kd * pitch_dInput;
      if(pitchOutput > pitchOutMax) pitchOutput = pitchOutMax;
      else if(pitchOutput < pitchOutMin) pitchOutput = pitchOutMin;
 
      /*Remember some variables for next time*/
      lastAnglePitch = anglePitch;
      lastTime = now;

      //roll calculations
      /*Compute all the working error variables*/
      double roll_error = desiredRollAngle - angleRoll;
      roll_ITerm+= (ki * roll_error);
      if(roll_ITerm > rollOutMax) roll_ITerm= rollOutMax;
      else if(roll_ITerm < rollOutMin) roll_ITerm= rollOutMin;
      double roll_dInput = (angleRoll - lastAngleRoll);
 
      /*Compute PID Output*/
      rollOutput = kp * roll_error + roll_ITerm- kd * roll_dInput;
      if(rollOutput > rollOutMax) rollOutput = rollOutMax;
      else if(rollOutput < rollOutMin) rollOutput = rollOutMin;
 
      /*Remember some variables for next time*/
      lastAngleRoll = angleRoll;
      lastTime = now;

      Serial.println("pid calculated");
      Serial.print("pitch output");Serial.println(pitchOutput);
      Serial.print("roll output");Serial.println(rollOutput);
      Serial.print("pitch error");Serial.println(pitch_error);
      Serial.print("roll error");Serial.println(roll_error);
/*
      //yaw calculations
      
      double error = desiredYawAngle - angleYaw;
      ITerm+= (ki * error);
      if(ITerm > yawOutMax) ITerm= yawOutMax;
      else if(ITerm < yawOutMin) ITerm= yawOutMin;
      double dInput = (angleYaw - lastAngleYaw);
 
      //Compute PID Output
      yawOutput = kp * error + ITerm- kd * dInput;
      if(yawOutput > yawOutMax) yawOutput = yawOutMax;
      else if(yawOutput < yawOutMin) yawOutput = yawOutMin;
 
      //Remember some variables for next time
      lastAngleYaw = angleYaw;
      lastTime = now;*/
   }
}
 
void SetTunings(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0|| Kd<0) return;
 
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
 
  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
   Serial.println("pid values passed");
   Serial.print("kp ");Serial.println(kp);
   Serial.print("ki ");Serial.println(ki);
   Serial.print("kd ");Serial.println(kd);
}
 
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
 

 
void SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        Initialize();
    }
    inAuto = newAuto;
}

void Initialize()
{
   lastAnglePitch = anglePitch;
   pitch_ITerm = pitchOutput;
   if(pitch_ITerm> pitchOutMax) pitch_ITerm= pitchOutMax;
   else if(pitch_ITerm< pitchOutMin) pitch_ITerm= pitchOutMin;

    lastAngleRoll = angleRoll;
   roll_ITerm = rollOutput;
   if(roll_ITerm> rollOutMax) roll_ITerm= rollOutMax;
   else if(roll_ITerm< rollOutMin) roll_ITerm= rollOutMin;
}
 
void SetControllerDirection(int Direction)
{
   controllerDirection = Direction;
}

void arm(){
  motor_1.write(170);
  motor_2.write(170);
  motor_3.write(170);
  motor_4.write(170);
  delay(2000);
  motor_1.write(90);
  motor_2.write(90);
  motor_3.write(90);
  motor_4.write(90);
  delay(2000);
 /* motor_1.write(140);
  motor_2.write(140);
  motor_3.write(140);
  motor_4.write(140);
  delay(2000);*/
  Serial.println("motor armed");
  }

void disarm(){
  while(1)
  {
    motor_1.writeMicroseconds(1000);
    motor_2.writeMicroseconds(1000);
    motor_3.writeMicroseconds(1000);
    motor_4.writeMicroseconds(1000);
    ch1 = pulseIn (chA,HIGH);
    Serial.println("motor disarmed");
    Serial.print("channel1 ");Serial.println(ch1);
    if(ch1<1600){
      break;
      }
  }
 }


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(9600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    //==================================//
    motor_1.attach(4);//right front ccw
  motor_2.attach(5);//right rear cw
  motor_3.attach(6);//left rear ccw
  motor_4.attach(7);//left front cw

  pinMode(chA, INPUT);
  pinMode(chB,INPUT);
  //pinMode(chC,INPUT);
  pinMode(chD,INPUT);

  bool done = false;
Serial.begin(9600);
radio.begin(); // Start the NRF24L01
//radio.printDetails();
radio.openReadingPipe(1,pipe); // Get NRF24L01 ready to receive
//radio.setPALevel(RF24_PA_MIN);
radio.startListening(); // Listen to see if information received
//Serial.println(done);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
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
    angleCalculate();
    SetTunings(kp, ki, kd);
    Compute();
    arm();
   /* ch1 = pulseIn (chA,HIGH);
    Serial.print("channel1 ");Serial.println(ch1);
    if(ch1>1600){
      
      disarm();
      }*/
 // int throttle = 1500;//pulseIn (chB,HIGH);
  Serial.print("throttle reicieved");
  Serial.println(throttle);
                                                         //The motors are started.
    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    if(throttle < 1200) throttle = 1200;
    esc_1 = throttle - pitchOutput + rollOutput; //Calculate the pulse for esc 1 (front-right - CCW) ie,M2

if(esc_1 <1200) esc_1 = 1200;
if(esc_1 >1800) esc_1 = 1800;

    
    esc_2 = throttle + pitchOutput + rollOutput; //Calculate the pulse for esc 2 (rear-right - CW) ie,M4

if(esc_2 <1200) esc_2 = 1200;
if(esc_2 >1800) esc_2 = 1800;
    
    esc_3 = throttle + pitchOutput - rollOutput; //Calculate the pulse for esc 3 (rear-left - CCW) ie,M3

   if(esc_3 <1200) esc_3 = 1200;
   if(esc_3 >1800) esc_3 = 1800; 
   
    esc_4 = throttle - pitchOutput - rollOutput ; //Calculate the pulse for esc 4 (front-left - CW) ie,M1

    if(esc_4 < 1200) esc_4 = 1200;
    if(esc_4 > 1800) esc_4 = 1800;

    Serial.print("esc_1 ");Serial.println(esc_1);
    Serial.print("esc_2 ");Serial.println(esc_2);
    Serial.print("esc_3 ");Serial.println(esc_3);
    Serial.print("esc_4 ");Serial.println(esc_4);

    motor_1.writeMicroseconds(esc_1);
    motor_2.writeMicroseconds(esc_2);
    motor_3.writeMicroseconds(esc_3);
    motor_4.writeMicroseconds(esc_4);
    
}
