#include <Wire.h>
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>



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


Servo Servo1;
Servo Servo2;
Servo Servo3;
Servo Servo4;

class AsyncServo
{
  Servo *aServo; // pointer to servo object
  int pin;

  int targetPosition;
  int currentPosition;
  int moveIncrement;

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
    if ((millis() - lastUpdate) > updateInterval) {
      lastUpdate = millis();
      if (currentPosition < targetPosition) {
        currentPosition += moveIncrement;
        aServo->write(currentPosition);
      } else if (currentPosition > targetPosition) {
        currentPosition -= moveIncrement;
        aServo->write(currentPosition);
      }
      Serial.println(currentPosition);
    }
  }
};


int Servo1_pos = 0;  // variable to store the servo position
int Servo2_pos = 0;  // variable to store the servo position
int Servo3_pos = 0;  // variable to store the servo position
int Servo4_pos = 0;  // variable to store the servo position

int newJointPos = 10;

// Initialize at the default i2c address, we are only using one expander
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  // put your setup code here, to run once:
  //Setup Servo Driver
  Servo1.attach(SERVO1_PIN);  
  Servo2.attach(SERVO2_PIN); 
  Servo3.attach(SERVO3_PIN);
  Servo4.attach(SERVO4_PIN);
    //Setup Arm Parameters

  //Setup Ultrasonic Sensor
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SERVO1_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  //Arm.setJointPos(0, newJointPos);
  //Arm.setJointPos(1, newJointPos);
  //newJointPos += 10;

  //come(Servo1);
  //come(Servo2);

  //if (newJointPos >= 175) {
  //  newJointPos = 15;
  //}
}




void come(Servo movingServo, int pos) {
  for (pos = 0; pos <= 180; pos += 1) {  // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    movingServo.write(pos);  // tell servo to go to position in variable 'pos'
    delay(15);               // waits 15 ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) {  // goes from 180 degrees to 0 degrees
    movingServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                           // waits 15 ms for the servo to reach the position
  }
}
