#include <QTRSensors.h>
#include <QuickPID.h>

//motor pins and speeds
const int PWMA = 11;
const int AIN2 = 10;
const int AIN1 = 9;
const int STDBY = 8;
const int BIN1 = 7; 
const int BIN2 = 6;
const int PWMB = 5;
int speedL = 125;
int speedR = -125;

//sensor stuff
// const int s0 = 7; //left
// const int s1 = 6; //middle
// const int s2 = 5; //right
QTRSensors qtr;
uint16_t sensorValues[3];

//for PID
float Kp = 1; //low single digits
float Ki = 0; // <0.1
float Kd = 0; //low single digits
float position; //PID input
float steeringVal; //PID output
float setPoint; //goal position error is 0, directly centered on lines
QuickPID myPID(&position, &steeringVal, &setPoint);

void setup() {
  //pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STDBY, OUTPUT);
  digitalWrite(STDBY, HIGH);

  //sensor setup
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7,A6,A5},3); //A7 is left
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT); //Arduino LED
  digitalWrite(LED_BUILTIN, HIGH);
  //callibrating sensors
  for (uint16_t i=0; i<400; i++){
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); //indicates done with callibration
  // //print callibration minimum value
  Serial.begin(9600);
  // for (uint8_t i = 0; i < 3; i++){
  //   Serial.print(qtr.calibrationOn.minimum[i]);
  //   Serial.print(' ');
  // }
  // Serial.println();
  // //print callibration max values
  // for (uint8_t i = 0; i < 3; i++){
  //   Serial.print(qtr.calibrationOn.maximum[i]);
  //   Serial.print(' ');
  // }

  //PID stuff
  uint16_t position = qtr.readLineBlack(sensorValues);
  setPoint = 1000;
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetMode(myPID.Control::automatic);

  Serial.println();
  delay(1000);
}

void loop() {
  // read calibrated sensor values and obtain a measure of the line position
  uint16_t position = qtr.readLineBlack(sensorValues);
  for (uint8_t i = 0; i < 3; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);
  Serial.println(analogRead(6));
  delay(250);
  // Serial.print("Position");
  // Serial.println(position);
  //if robot leaves the line entirely
  // while(sensorValues[0]<=700 && sensorValues[1]<=700 && sensorValues[2]<=700){ //not sure about this
  //   if(position==0){
  //     drive(125,-125); //turn left if line disappeared to the left
  //   }
  //   else{
  //     drive(-125,125); //turn right
  //   }
  //   position = qtr.readLineBlack(sensorValues);
  // }
//   myPID.Compute();
//   Serial.print("SteeringVal");
//   Serial.println(steeringVal);
//   drive(speedL - steeringVal, speedR - steeringVal); //not sure abt adding/subtracting here
 }

void motorWrite(int spd, int pin_IN1, int pin_IN2, int pin_PWM){
  if (spd < 0){ //go one way
    digitalWrite(pin_IN1, HIGH); 
    digitalWrite(pin_IN2, LOW);
  }
  else{ //go the other way
    digitalWrite(pin_IN1, LOW);
    digitalWrite(pin_IN2, HIGH);
  }
  spd = abs(spd);
  spd = constrain(spd, 0, 255);
  analogWrite(pin_PWM, spd);
}

void drive(int speedL, int speedR){
  motorWrite(speedL, AIN1, AIN2, PWMA);
  motorWrite(speedR, BIN1, BIN2, PWMB);
}
