/* Firgelli Automations
 * Limited or no support: we do not have the resources for Arduino code support
 * 
 * Program enables momentary direction control of actuator using push button
 */
 
#include <elapsedMillis.h>
elapsedMillis timeElapsed;

int RPWM = 10;   
int LPWM = 11;
int sensorPin = A0;

int sensorVal;
int Speed=255;
float strokeLength = 12.0;                           //customize to your specific stroke length
float extensionLength;

int maxAnalogReading;
int minAnalogReading;
int minStepperMove;

unsigned long previousMillis = 0;  
long StepperTimeOn = 3000;   
unsigned long steps = 0;
byte StepperMoveNow = 0;


// Stepper Configuration
#include <AccelStepper.h>
AccelStepper stepper1(2, 9, 12);  // 1 sets the MotorInterfaceType to DRIVER; pin 9 = step; pin 12 = direction
const int sleepPin = 8;        // used to minimize the power consumption when not in use
const int resetPin = 7;        // used to "home" the motor
const int ms1Pin = 6;          // define pins for stepping mode
const int ms2Pin = 5;          // stepping modes change the step resolution of the motor
const int ms3Pin = 4;          // higher resolution comes at the expense of higher speeds and torque
const int enable1 = 3;        




void setup() {
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(sensorPin, INPUT);
  Serial.begin(9600);
//  maxAnalogReading = moveToLimit(1);
//  minAnalogReading = moveToLimit(-1);    
  maxAnalogReading = 550;   //THIS IS HOW Y CHAGNE LENGTH. Max is 930, min is 0, 530
  minAnalogReading = 110;    //keep 0 for bottom of stepper
  minStepperMove = maxAnalogReading - 10;

  int stepperSpeed = 500;  
  stepper1.setMaxSpeed(stepperSpeed); // sets the maximum steps per second, which determines how fast the motor will turn
  stepper1.setAcceleration(1000); // sets the acceleration rate in steps per second
  stepper1.setSpeed(stepperSpeed);

  pinMode(resetPin, OUTPUT);
  pinMode(sleepPin, OUTPUT);

  pinMode(ms1Pin, OUTPUT);        // set step mode pins as outputs
  pinMode(ms2Pin, OUTPUT);
  pinMode(ms3Pin, OUTPUT);
  digitalWrite(resetPin, HIGH);   // set reset and sleep HIGH to enable the stepper motor driver operation
  digitalWrite(sleepPin, HIGH);
  /*

  digitalWrite(ms1Pin, LOW);     // full step (LOW, LOW, LOW)
  digitalWrite(ms2Pin, LOW);      // runs smoother in half step mode (HIGH, LOW, LOW) but might need to increase current for required torque
  digitalWrite(ms3Pin, LOW);  */
}

void loop(){
//---------- GO HOME ------------
int turns = 3;
int MAX = 4*(turns);
steps = (10)*133.3333;
Serial.println(steps);
stepper1.moveTo(steps);
stepper1.runToPosition();
delay(2000);
  unsigned long currentMillis = millis();

/* //----------MOVE ACTUATOR UP------------  
  Serial.println("Extending...");
  sensorVal = analogRead(sensorPin);
  while(sensorVal < maxAnalogReading){

       StepperMoveNow = 1;
    driveActuator(1, Speed);
    displayOutput();  
  }
  driveActuator(0, Speed);
  delay(1000); */
  

    //----------MOVE STEPPER CCW (1 TURN)------------
    steps = steps - (133.3333);
    Serial.println(steps);
    stepper1.moveTo(steps);
    stepper1.runToPosition();
    delay(2000);

    //----------MOVE STEPPER CCW (1 TURN)------------
    steps = steps - (133.3333);
    Serial.println(steps);
    stepper1.moveTo(steps);
    stepper1.runToPosition();
    delay(2000);

     //----------MOVE STEPPER CCW (1 TURN)------------
    steps = steps - (133.3333);
    Serial.println(steps);
    stepper1.moveTo(steps);
    stepper1.runToPosition();
    delay(2000);

     //----------MOVE STEPPER CCW (1 TURN)------------
    steps = steps - (133.3333);
    Serial.println(steps);
    stepper1.moveTo(steps);
    stepper1.runToPosition();
    delay(2000);
    
 /* //----------MOVE ACTUATOR DOWN------------
  Serial.println("Retracting...");
  sensorVal = analogRead(sensorPin);
  while(sensorVal > minAnalogReading){   //before retracting delay to spin motor
    driveActuator(-1, Speed);
    displayOutput();   
  }
  driveActuator(0, Speed);
  delay(1000);
*/
}

int moveToLimit(int Direction){
  int prevReading=0;
  int currReading=0;
  do{
    prevReading = currReading;
    driveActuator(Direction, Speed);
    timeElapsed = 0;
    while(timeElapsed < 200){ delay(1);}           //keep moving until analog reading remains the same for 200ms
    currReading = analogRead(sensorPin);
  }while(prevReading != currReading);
  return currReading;
}

float mapfloat(float x, float inputMin, float inputMax, float outputMin, float outputMax){
 return (x-inputMin)*(outputMax - outputMin)/(inputMax - inputMin)+outputMin;
}

void displayOutput(){
  sensorVal = analogRead(sensorPin);
    extensionLength = mapfloat(sensorVal, float(minAnalogReading), float(maxAnalogReading), 0.0, strokeLength);
    Serial.print("Analog Reading: ");
    Serial.print(sensorVal);
    Serial.print("\tActuator extension length: ");
    Serial.print(extensionLength);
    Serial.println(" inches");  
}

void driveActuator(int Direction, int Speed){
  switch(Direction){
    case 1:       //extension
      analogWrite(RPWM, Speed);
      analogWrite(LPWM, 0);
      break;
   
    case 0:       //stopping
      analogWrite(RPWM, 0);
      analogWrite(LPWM, 0);
      break;

    case -1:      //retraction
      analogWrite(RPWM, 0);
      analogWrite(LPWM, Speed);
      break;
  }
}

void driveStepper(int turns) {
//delay(4000);
    steps = steps + (turns)*133.3333;
    stepper1.moveTo(steps);
    stepper1.runToPosition();
    delay(1);
    
}
