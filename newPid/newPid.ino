/*working variables*/
unsigned long lastTime;
double desiredPitchAngle, desiredRollAngle, desiredYawAngle;
double lastAnglePitch=0, lastAngleRoll=0, lastAngleYaw=0;
double ITerm;
double kp, ki, kd;
int SampleTime = 1000; //1 sec
int pitchOutMin=-400, pitchOutMax=400;
int rollOutMin=-400, rollOutMax=400;
int yawOutMin=-400, yawOutMax=400;
bool inAuto = false;

float desiredPitchAngle=2.35;
float desiredRollAngle=-1.86;
float desiredYawAngle=11.02;
 
#define MANUAL 0
#define AUTOMATIC 1
 
#define DIRECT 0
#define REVERSE 1
int controllerDirection = DIRECT;
 
void Compute()
{
   if(!inAuto) return;
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      //pitch calculation
      /*Compute all the working error variables*/
      double error = desiredPitchAngle - anglePitch;
      ITerm+= (ki * error);
      if(ITerm > pitchOutMax) ITerm= pitchOutMax;
      else if(ITerm < pitchOutMin) ITerm= pitchOutMin;
      double dInput = (anglePitch - lastAnglePitch);
 
      /*Compute PID Output*/
      pitchOutput = kp * error + ITerm- kd * dInput;
      if(pitchOutput > pitchOutMax) pitchOutput = pitchOutMax;
      else if(pitchOutput < pitchOutMin) pitchOutput = pitchOutMin;
 
      /*Remember some variables for next time*/
      lastAnglePitch = anglePitch;
      lastTime = now;

      //roll calculations
      /*Compute all the working error variables*/
      double error = desiredRollAngle - angleRoll;
      ITerm+= (ki * error);
      if(ITerm > rollOutMax) ITerm= rollOutMax;
      else if(ITerm < rollOutMin) ITerm= rollOutMin;
      double dInput = (angleRoll - lastAngleRoll);
 
      /*Compute PID Output*/
      rollOutput = kp * error + ITerm- kd * dInput;
      if(rollOutput > rollOutMax) rollOutput = rollOutMax;
      else if(rollOutput < rollOutMin) rollOutput = rollOutMin;
 
      /*Remember some variables for next time*/
      lastAngleRoll = angleRoll;
      lastTime = now;

      //yaw calculations
      /*Compute all the working error variables*/
      double error = desiredYawAngle - angleYaw;
      ITerm+= (ki * error);
      if(ITerm > yawOutMax) ITerm= yawOutMax;
      else if(ITerm < yawOutMin) ITerm= yawOutMin;
      double dInput = (angleYaw - lastAngleYaw);
 
      /*Compute PID Output*/
      yawOutput = kp * error + ITerm- kd * dInput;
      if(yawOutput > yawOutMax) yawOutput = yawOutMax;
      else if(yawOutput < yawOutMin) yawOutput = yawOutMin;
 
      /*Remember some variables for next time*/
      lastAngleYaw = angleYaw;
      lastTime = now;
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
   ITerm = pitchOutput;
   if(ITerm> pitchOutMax) ITerm= pitchOutMax;
   else if(ITerm< pitchOutMin) ITerm= pitchOutMin;

    lastAngleRoll = angleRoll;
   ITerm = rollOutput;
   if(ITerm> rollOutMax) ITerm= rollOutMax;
   else if(ITerm< rollOutMin) ITerm= rollOutMin;
}
 
 
void SetControllerDirection(int Direction)
{
   controllerDirection = Direction;
}
