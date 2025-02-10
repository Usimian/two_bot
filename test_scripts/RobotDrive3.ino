#include <QuickPID.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Serial commands
const int ReadEncoders = 1; // Read encoders <ReadEncoders>, return <encoderLeft, encoderRight>
const int SetVelocity = 2;  // Set motor velocity <SetVelocity#motor#velocity(mm/sec)>, return nothing
const int MoveDistance = 3; // Move wheel set distance <MoveDistance#motor#distance(mm))>, return nothing
const int Move = 4;         // Move wheel continuous <Move#motor#direction>, return nothing
const int MoveStatus = 5;   // Read move status <MoveStatus>, return <left motor status#right motor status>  true if move complete

const int LeftMotor = 1;
const int RightMotor = 2;

const float wheelCirc = 60 * 3.14159;             // Drive wheel (mm/rev)
const float ticksPerRev = 408;                    // Encoder (ticks/rev)
const float ticksPerMm = ticksPerRev / wheelCirc; // ticks/mm

// const int pingPin = 5; // Trigger Pin of Ultrasonic Sensor
// const int echoPin = 6; // Echo Pin of Ultrasonic Sensor/60

// DIO pin/interrupts, Feather Express M4 can use any DIO pin
const int encoderRA = 5;  // Right wheel encoder A (interrupt)
const int encoderRB = 6;  // Right wheel encoder B
const int encoderLA = 9;  // Left wheel encoder A (interrupt)
const int encoderLB = 10; // Right wheel encoder B

// Current encoder values
float encoderValueR = 0; // Current encoder value
float encoderValueL = 0;

float encoderSetptR = 0; // Encoder value to find
float encoderSetptL = 0;

float velSetptR = 0; // Max wheel velocity value to find
float velSetptL = 0;

float velOutputR = 0; // Velocity output from position PID
float velOutputL = 0;

float motorSpeedR = 0; // Motor speed output
float motorSpeedL = 0;

float velRight = 0; // Current measured wheel velocity (mm/sec)
float velLeft = 0;

int motorDirR = FORWARD;
int motorDirL = FORWARD;

Adafruit_DCMotor *myMotor[] = {
    AFMS.getMotor(LeftMotor),
    AFMS.getMotor(RightMotor)};

// Loop time variables
unsigned long previousMillis = 0;
const long interval = 20;

// Command buffer variables
int cmdIndex = 0;
int cmdData[20];

boolean newCommand = false; // True when full command received but unprocessed

// ******* PID variables *******
float Kp = 12, Ki = 3, Kd = .8;    // Distance PID tuning
float KpV = 1, KiV = 3, KdV = 0.3; // Velocity PID tuning

// Specify PID links
QuickPID myPIDRight(&encoderValueR, &velOutputR, &encoderSetptR); // Setpt in mm
QuickPID myPIDLeft(&encoderValueL, &velOutputL, &encoderSetptL);

QuickPID myPIDRightVel(&velRight, &motorSpeedR, &velOutputR); // Setpt in mm/sec
QuickPID myPIDLeftVel(&velLeft, &motorSpeedL, &velOutputL);

// ***** ARDUINO SETUP *****
void setup()
{
  Serial.begin(9600); // Starting Serial Terminal
  while (!Serial)
    ; // Wait for Serial port to be ready

  pinMode(LED_BUILTIN, OUTPUT);

  //  pinMode(pingPin, OUTPUT); // Ultrasonic ping
  //  pinMode(echoPin, INPUT);  // Ultrasonic echo

  // Right  Motor
  pinMode(encoderRA, INPUT);
  pinMode(encoderRB, INPUT);

  digitalWrite(encoderRA, HIGH); // turn pullup resistor on
  digitalWrite(encoderRB, HIGH); // turn pullup resistor on

  // call updateEncoderL() when any high/low changed seen on interrupt
  attachInterrupt(digitalPinToInterrupt(encoderRA), updateEncoderR, RISING);

  // Left Motor
  pinMode(encoderLA, INPUT);
  pinMode(encoderLB, INPUT);

  digitalWrite(encoderLA, HIGH); // turn pullup resistor on
  digitalWrite(encoderLB, HIGH); // turn pullup resistor on

  // call updateEncoderR() when any high/low changed seen on interrupt
  attachInterrupt(digitalPinToInterrupt(encoderLA), updateEncoderL, RISING);

  // Init distance PID controller
  myPIDRight.SetTunings(Kp, Ki, Kd);
  myPIDRight.SetProportionalMode(myPIDRight.pMode::pOnMeas);
  myPIDRight.SetMode(myPIDRight.Control::timer);
  myPIDRight.SetOutputLimits(-255.0, 255.0);
  myPIDRight.SetSampleTimeUs(100000);

  myPIDLeft.SetTunings(Kp, Ki, Kd);
  myPIDLeft.SetProportionalMode(myPIDLeft.pMode::pOnMeas);
  myPIDLeft.SetMode(myPIDLeft.Control::timer);
  myPIDLeft.SetOutputLimits(-255.0, 255.0);
  myPIDLeft.SetSampleTimeUs(100000);

  // Init velocity PID controller
  myPIDRightVel.SetTunings(KpV, KiV, KdV);
  myPIDRightVel.SetProportionalMode(myPIDRightVel.pMode::pOnMeas);
  myPIDRightVel.SetMode(myPIDRightVel.Control::timer);
  myPIDRightVel.SetOutputLimits(0.0, 255.0);
  myPIDRightVel.SetSampleTimeUs(100000);

  myPIDLeftVel.SetTunings(KpV, KiV, KdV);
  myPIDLeftVel.SetProportionalMode(myPIDLeftVel.pMode::pOnMeas);
  myPIDLeftVel.SetMode(myPIDLeftVel.Control::timer);
  myPIDLeftVel.SetOutputLimits(0.0, 255.0);
  myPIDLeftVel.SetSampleTimeUs(100000);

  Serial.println("Robot Drive TEST running");
  if (!AFMS.begin()) // create with the default frequency 1.6KHz
  {
    Serial.println("Could not find Motor Shield");
    while (1)
      ; // Sit here until restart
  }
  else
  {
    // Init drive motors 0 (off) to 255 (max vel)
    for (int i = 0; i < 2; i++)
    {
      myMotor[i]->setSpeed(150);
      myMotor[i]->run(FORWARD);
      // turn on motor
      myMotor[i]->run(RELEASE);
    }
    Serial.println("Motor Shield found");
  }
}

// ***** ARDUINO LOOP *****
void loop()
{
  // ***********************************
  // ***** Right Motor PID control *****
  // ***********************************
  myPIDRight.Compute(); // PID compute distance (mm)

  if (velOutputR >= 0) // Determine motor direction
    motorDirR = FORWARD;
  else
    motorDirR = BACKWARD;

  // Limit velocity level to velSetpt
  velOutputR = abs(velOutputR);
  if (velOutputR > velSetptR)
    velOutputR = velSetptR;

  myPIDRightVel.Compute(); // PID compute Velocity (mm/sec)

  myMotor[RightMotor - 1]->setSpeed(abs(motorSpeedR));
  myMotor[RightMotor - 1]->run(motorDirR); // FORWARD or BACKWARD

  // **********************************
  // ***** Left Motor PID control *****
  // **********************************
  myPIDLeft.Compute(); // PID compute distance (mm)

  if (velOutputL >= 0) // Determine motor direction
    motorDirL = FORWARD;
  else
    motorDirL = BACKWARD;

  // Limit velocity level to velSetpt
  velOutputL = abs(velOutputL);
  if (velOutputL > velSetptL)
    velOutputL = velSetptL;

  myPIDLeftVel.Compute(); // PID compute Velocity (mm/sec)

  myMotor[LeftMotor - 1]->setSpeed(abs(motorSpeedL));
  myMotor[LeftMotor - 1]->run(motorDirL);

  if ((motorSpeedL > 0) || (motorSpeedR > 0))
    digitalWrite(LED_BUILTIN, HIGH); // LED on when motors on
  else
    digitalWrite(LED_BUILTIN, LOW); // LED off when motors off

  // Command parsing code
  // unsigned long currentMillis = millis();
  // if (currentMillis - previousMillis >= interval)   // execute every 'interval' msec
  // {
  //   previousMillis = currentMillis;

  if (recvWithStartEndMarkers()) // Get command
  {
    int cmd = cmdData[0];       // 1 = Read Encoders, 2 = Set Velocity, 3 = Move Distance, 4 = Move Continuous, 5 = Get Move Status
    int motor = cmdData[1];     // 1 = right, 2 = left
    long vel = abs(cmdData[2]); // Velocity (+) (mm/sec)
    long dist = cmdData[2];     // Distance (+/-) (mm)

    if (cmd == ReadEncoders) // Get encoder values
    {
      Serial.print("<");
      Serial.print((long)encoderValueL);
      Serial.print("#");
      Serial.print((long)encoderValueR);
      Serial.println(">");
    }
    else if (cmd == SetVelocity) // Set max velocity (mm/sec)
    {
      if (motor == RightMotor)
        velSetptR = vel; // mm/sec
      else if (motor == LeftMotor)
        velSetptL = vel; // mm/sec

      // Serial.print("<");
      // Serial.print("1");
      // Serial.println(">");
    }
    else if (cmd == MoveDistance) // Motor move fixed distance (mm)
    {
      if (motor == RightMotor)
      {
        // ResetEncoders(RightMotor);
        encoderSetptR = (long)(dist * ticksPerMm); // ticks
      }
      else if (motor == LeftMotor)
      {
        // ResetEncoders(LeftMotor);
        encoderSetptL = (long)(dist * ticksPerMm);
      }
      // Serial.print("<");
      // Serial.print("1");
      // Serial.println(">");
    }
    else if (cmd == Move) // Motor move continuous
    {
      if (motor == RightMotor)
      {
        // ResetEncoders(RightMotor);
        if (dist > 0)
          encoderSetptR = (long)(100000000 * ticksPerMm); // far enough to be continuous forward
        else
          encoderSetptR = (long)(-100000000 * ticksPerMm); // far enough to be continuous backward
      }
      else if (motor == LeftMotor)
      {
        // ResetEncoders(LeftMotor);
        if (dist > 0)
          encoderSetptL = (long)(100000000 * ticksPerMm); // ticks
        else
          encoderSetptL = (long)(-100000000 * ticksPerMm); // ticks
      }
      // Serial.print("<");
      // Serial.print("1");
      // Serial.println(">");
    }
    else if (cmd == MoveStatus) // Return true when motor reaches distance setpt
    {
      Serial.print("<");
      if (((encoderValueL != encoderSetptL) || (velLeft != 0)))
        Serial.print("0");
      else
        Serial.print("1");
      Serial.print("#");
      if (((encoderValueR != encoderSetptR) || (velRight != 0)))
        Serial.print("0");
      else
        Serial.print("1");
      Serial.println(">");
    }
    newCommand = false;
  }
  // }
}

// Interrupt on right motor A encoder
void updateEncoderR()
{
  static unsigned long oldTickTime = 0; // usec
  unsigned long tickTime;
  unsigned long dTickTime;

  if (digitalRead(encoderRB) == HIGH) // if ENCODER_B is high increase the count
    encoderValueR++;                  // increment the count
  else                                // else decrease the count
    encoderValueR--;                  // decrement the count

  tickTime = micros();                           // usec
  dTickTime = tickTime - oldTickTime;            // usec
  velRight = 1000000 / (ticksPerMm * dTickTime); // mm/sec
  oldTickTime = tickTime;
}

// Interrupt on left motor A encoder
void updateEncoderL()
{
  static unsigned long oldTickTime = 0; // msec
  unsigned long tickTime;
  unsigned long dTickTime;

  if (digitalRead(encoderLB) == HIGH) // if ENCODER_B is high increase the count
    encoderValueL--;                  // increment the count
  else                                // else decrease the count
    encoderValueL++;                  // decrement the count

  tickTime = micros();                          // usec
  dTickTime = tickTime - oldTickTime;           // usec
  velLeft = 1000000 / (ticksPerMm * dTickTime); // mm/sec
  oldTickTime = tickTime;
}

long getDistance() // Read ultrasonic sensor distance (uSec)
{
  //   digitalWrite(pingPin, LOW);
  //   delayMicroseconds(2);
  //   digitalWrite(pingPin, HIGH);
  //   delayMicroseconds(10);
  //   digitalWrite(pingPin, LOW);
  //
  //   return (pulseIn(echoPin, HIGH));
  return 1260; // TEST VALUE
}

bool recvWithStartEndMarkers() // Return true when command parsed
{
  const int numChars = 32;
  static char receivedChars[numChars];
  static boolean recvInProgress = false;
  static byte ndx = 0; // Rcv buffer index

  const char startMarker = '<';
  const char endMarker = '>';
  const char delimiterMarker = '#';

  char rc;

  while (Serial.available() > 0 && newCommand == false)  // Serial data available and no pending command
  {
    rc = Serial.read();
    if (rc == startMarker)
    {
      //      Serial.print("startMarker\n");
      recvInProgress = true;
      ndx = 0;
      cmdIndex = 0;
    }
    else if (recvInProgress == true)
    {
      if (rc == endMarker)
      {
        //        Serial.print("endMarker\n");
        receivedChars[ndx] = '\0';               // Terminate string
        cmdData[cmdIndex] = atoi(receivedChars); // Save command
        recvInProgress = false;
        newCommand = true; // Command received in cmdData[]
        return true;
      }
      else if (rc == delimiterMarker)
      {
        //        Serial.print("delimiterMarker\n");
        receivedChars[ndx] = '\0';               // Terminate string
        cmdData[cmdIndex] = atoi(receivedChars); // Save command
        //        Serial.println(cmdIndex);
        cmdIndex = min(cmdIndex + 1, 9);
        ndx = 0;
      }
      else
      {
        //        Serial.print("char: ");
        //        Serial.print(rc);
        //        Serial.print('\n');
        receivedChars[ndx] = rc;
        ndx = min(ndx + 1, numChars - 1);
      }
    }
  }
  return false; // no command yet
}
