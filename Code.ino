#include <QTRSensors.h>

const int PWMA = 11;
const int AIN2 = 10;
const int AIN1 = 9;
const int STDBY = 8;
const int BIN1 = 7; 
const int BIN2 = 6;
const int PWMB = 5;
const int s0 = 7; //left
const int s1 = 6; //middle
const int s2 = 5; //right
QTRSensors qtr;
uint16_t sensorValues[3];
int speedL = 200;
int speedR = -200;

void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STDBY, OUTPUT);
  digitalWrite(STDBY, HIGH);
  analogWrite(s0, INPUT);
  analogWrite(s1, INPUT);
  analogWrite(s2, INPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){s0,s1,s2},3);
  delay(500);

  //Arduino LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  //callibrate
  for (uint16_t i=0; i<400; i++){
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); //indicates done with callibration

  //print callibration minimum value
  Serial.begin(9600);
  for (uint8_t i = 0; i < 3; i++){
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  //print callibration max values
  for (uint8_t i = 0; i < 3; i++){
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  delay(1000);
}

void loop() {
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);
  // int left = sensorValues[0];
  // int middle = sensorValues[1];
  // int right = sensorValues[2];
  
  // sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  // for (uint8_t i = 0; i < 3; i++)
  // {
  //   Serial.print(sensorValues[i]);
  //   Serial.print('\t');
  // }
  Serial.println(position);

  delay(250);
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
