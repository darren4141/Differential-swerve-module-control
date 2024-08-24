const int fwdPin1 = 2;
const int bwdPin1 = 4;
const int enablePin1 = 15;
const int encA1 = 16;
const int encB1 = 17;

int encoderCount[] = {0, 0};

float eI[] = {0, 0};
float ePPrevious[] = {0, 0};
float previousTime[] = {0, 0};

float previousLogTime = 0;
int previousEncoderCount = 0;

float currentSpeed;

void setup() {
  // put your setup code here, to run once:
  pinMode(fwdPin1, OUTPUT);
  pinMode(bwdPin1, OUTPUT);

  pinMode(encA1, INPUT);
  pinMode(encB1, INPUT);

  attachInterrupt(digitalPinToInterrupt(encA1), tickEncoder1, RISING);
  
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
//  digitalWrite(fwdPin1, HIGH);
//  digitalWrite(bwdPin1, LOW);
//
//  digitalWrite(fwdPin2, LOW);
//  digitalWrite(bwdPin2, HIGH);
//
//  analogWrite(enablePin1, 255);
//  analogWrite(enablePin2, 255);


  float currentSpeed = logSpeed(0);

  float pwr1 = calculatePID(0, 1000, currentSpeed 1, 0.4, 0.01);

  if(pwr1 > 0){
    analogWrite(enablePin1, abs(pwr1));
    digitalWrite(fwdPin1, HIGH);
    digitalWrite(bwdPin1, LOW);
  }else{
    analogWrite(enablePin1, abs(pwr1));
    digitalWrite(fwdPin1, LOW);
    digitalWrite(bwdPin1, HIGH);
  }

}

void tickEncoder1(){
  if(digitalRead(encA1) > digitalRead(encB1)){
    encoderCount[0]++;
  }else{
    encoderCount[0]--;
  }
}

void tickEncoder2(){
  if(digitalRead(encA2) > digitalRead(encB2)){
    encoderCount[1]++;
  }else{
    encoderCount[1]--;
  }
}

float calculatePID(int motorNum, float targetSpeed, float currentSpeed, float kP, float kI, float kD){
  long currentTime = micros();
  float dT = ((float)(currentTime - previousTime[motorNum])) / 1.0e6;

  int eP = targetSpeed - currentSpeed;
  eI[motorNum] = eI[motorNum] + (eP * dT);
  float eD = (eP - ePPrevious[motorNum])/dT;

  previousTime[motorNum] = currentTime;
  ePPrevious[motorNum] = eP;

  float pwr = (kP * eP) + (kI * eI[motorNum]) + (kD * eD);

  if(abs(pwr) > 255){
    if(pwr > 0){
      pwr = 255;
    }else{
      pwr = -255;
    }
  }

  return pwr;
}

float logSpeed(int motorNum){
    long currentLogTime = micros();
    int currentEncoderCount = encoderCount[motorNum];

    float speed = (currentEncoderCount - previousEncoderCount)/(currentLogTime - previousLogTime);

    previousEncoderCount = currentEncoderCount;
    previousLogTime = currentLogTime;
    return speed;
}