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

AsyncServo servo1(&Servo1, SERVO1_PIN, 3, 3, 10, 0);
AsyncServo servo2(&Servo2, SERVO2_PIN, 3, 3, 10, 0);
AsyncServo servo3(&Servo3, SERVO3_PIN, 3, 3, 10, 0);
AsyncServo servo4(&Servo4, SERVO4_PIN, 3, 3, 10, 0);


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

  //come(Servo1);
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
  Serial.println(reading);
  if(reading <= (lastSonicRange - sonicHysterisis)){
    Serial.println("Range Decreasing");
    lastSonicRange = reading;
    return 0;
  }else if(reading >= lastSonicRange + sonicHysterisis){
    Serial.println("Range increasing");
    lastSonicRange = reading;
    return 2;
  }else {
    Serial.println("Range stable");
    lastSonicRange = reading;
    return 1;
  }
  lastSonicRange = reading;
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
