#include <Wire.h>
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <Ultrasonic.h>



/**************************
*
* Setup a two servo tentacle arm to trigger on a approaching pwm signal
*
*
*/

//********** PINOUT **************
//
#define TRIG_PIN 12
#define ECHO_PIN 13
#define SCL_PIN 5  // Useless but documentation
#define SDA_PIN 5  // Useless but documentation
#define SERVO1_PIN 9
#define SERVO2_PIN 10
#define SERVO3_PIN 5
#define SERVO4_PIN 6

//********** Ultrasonic **************
//
Ultrasonic ultrasonic(12, 13);
long lastSonicRange = 0;

//********** Servo **************
//
Servo Servo1;
Servo Servo2;
Servo Servo3;
Servo Servo4;

class AsyncServo
{
  Servo *aServo; // pointer to servo object
  int pin;

  public: int targetPosition;
  int currentPosition;
  int moveIncrement;
  public: bool finishedMove = false;

  int updateInterval;
  unsigned long lastUpdate;

  public: AsyncServo(Servo *Driver, int Pin, int Interval, int Speed, int tPos, int cPos){
    aServo = Driver;
    pin = Pin;
    updateInterval = Interval;
    targetPosition = tPos;
    currentPosition = cPos;
    moveIncrement = Speed;
  }

  void Update() {
    Serial.println("Update");
    if ((millis() - lastUpdate) > updateInterval) {
      lastUpdate = millis();
      if (currentPosition < targetPosition) {
        currentPosition += moveIncrement;
        aServo->write(currentPosition);
      } else if (currentPosition > targetPosition) {
        currentPosition -= moveIncrement;
        aServo->write(currentPosition);
      } else if (currentPosition == targetPosition){
        Serial.println("finishedMove");
        finishedMove = true;
      }
      Serial.print(pin);
      Serial.print("servo current pos: ");
      Serial.println(currentPosition);
      Serial.print(pin);
      Serial.print("servo target pos:");
      Serial.println(targetPosition);
    }
  }

  int getCurPos(){
    return currentPosition;
  }

  void setSpeed(int newSpeed){
    moveIncrement = newSpeed;
  }

  void goToPos(int newPos){
    targetPosition = newPos;
  }

  void newMove(int newPos, int newSpeed){
    Serial.print("finished Move: ");
    Serial.println(finishedMove);
    finishedMove = false;
    Serial.print("finished Move: ");
    Serial.println(finishedMove);
    moveIncrement = newSpeed;
    Serial.print("New Move Increment: ");
    Serial.println(moveIncrement);
    targetPosition = newPos;
    Serial.print("New Target: ");
    Serial.println(targetPosition);
  }
};

AsyncServo servo1(&Servo1, SERVO1_PIN, 1, 1, 20, 0);
//AsyncServo servo2(&Servo2, SERVO2_PIN, 1, 1, 0, 0);
//AsyncServo servo3(&Servo3, SERVO3_PIN, 3, 3, 10, 0);
//AsyncServo servo4(&Servo4, SERVO4_PIN, 3, 3, 10, 0);





int motionStep = 0;
bool firstMove = true;
// Initialize at the default i2c address, we are only using one expander
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //Setup Servo Driver
  Servo1.attach(SERVO1_PIN);  
  Servo2.attach(SERVO2_PIN); 
  Servo3.attach(SERVO3_PIN);
  Servo4.attach(SERVO4_PIN);
    //Setup Arm Parameters
  ultrasonic.setTimeout(40000UL);
  //Setup Ultrasonic Sensor
  //pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SERVO1_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  //Arm.setJointPos(0, newJointPos);
  //Arm.setJointPos(1, newJointPos);
  //newJointPos += 10;
  servo1.Update();
  //servo2.Update();
  come(servo1);
  //come(Servo2);
  sonicDirection(ultrasonic.read());
  //if (newJointPos >= 175) {
  //  newJointPos = 15;
  //}
  
  delay(1000);
}

// Returns 0,1,2 if the range is decreasing, stable, increasing
int sonicHysterisis = 20;
int sonicDirection(long reading){
  //Serial.println(reading);
  if(reading <= (lastSonicRange - sonicHysterisis)){
    //Serial.println("Range Decreasing");
    lastSonicRange = reading;
    return 0;
  }else if(reading >= lastSonicRange + sonicHysterisis){
    //Serial.println("Range increasing");
    lastSonicRange = reading;
    return 2;
  }else {
    //Serial.println("Range stable");
    lastSonicRange = reading;
    return 1;
  }
  lastSonicRange = reading;
}


void come(AsyncServo& movingServo1){//}, AsyncServo movingServo2) {
  int servo1_Move[] =   {0, 180, 0, 180};
  int servo_Speed[] =  {1, 1,   1,   1};
  int servo2_Move[] =   {0, 180, 0, 180};
  int maxStep = 3;
  
  if(movingServo1.finishedMove){
    Serial.println("finished move!");
    movingServo1.newMove(servo1_Move[motionStep], servo_Speed[motionStep]);
    //movingServo2.newMove(servo2_Move[motionStep], servo_Speed[motionStep]);
    motionStep++;
    Serial.print("motionStep ");
    Serial.println(motionStep);
  }
  
  //if( movingServo2.finishedMove){
    
  //}
  if( motionStep == maxStep){
    motionStep = 0;
  }

  firstMove = false;
}

void breathe(Servo movingServo1, Servo movingServo2){

}

void scream(Servo movingServo1, Servo movingServo2){

}
