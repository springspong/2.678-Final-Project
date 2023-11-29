const int PWMA = 11;
const int AIN2 = 10;
const int AIN1 = 9;
const int STDBY = 8;
const int BIN1 = 7; 
const int BIN2 = 6;
const int PWMB = 5;

void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STDBY, OUTPUT);
  digitalWrite(STDBY, HIGH);
}

void loop() {
  drive(100,-100); // +A and -B is forward
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
