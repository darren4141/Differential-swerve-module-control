const int fwdPin1 = 2;
const int bwdPin1 = 4;
const int enablePin1 = 15;
const int encA1 = 16;
const int encB1 = 17;

const int fwdPin2 = 18;
const int bwdPin2 = 19;
const int enablePin2 = 5;
const int encA2 = 22;
const int encB2 = 23;

int encoderCount1 = 0;
int encoderCount2 = 0;

float eI = 0;
float ePPrevious = 0;
float previousTime = 0;

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
  digitalWrite(fwdPin1, HIGH);
  digitalWrite(bwdPin1, LOW);

  float pwr = calculatePID(1000, 1, 0.4, 0.01);

  if(pwr > 0){
    analogWrite(enablePin1, abs(pwr));
    digitalWrite(fwdPin1, HIGH);
    digitalWrite(bwdPin1, LOW);
  }else{
    analogWrite(enablePin1, abs(pwr));
    digitalWrite(fwdPin1, LOW);
    digitalWrite(bwdPin1, HIGH);
  }
  
  Serial.print(encoderCount1);
  Serial.print(", ");
  Serial.println(pwr);
}

void tickEncoder1(){
  if(digitalRead(encA1) > digitalRead(encB1)){
    encoderCount1++;
  }else{
    encoderCount1--;
  }
}

float calculatePID(int target, float kP, float kI, float kD){
  long currentTime = micros();
  float dT = ((float)(currentTime - previousTime)) / 1.0e6;

  int eP = target - encoderCount;
  eI = eI + (eP * dT);
  float eD = (eP - ePPrevious)/dT;

  previousTime = currentTime;
  ePPrevious = eP;

  float pwr = (kP * eP) + (kI * eI) + (kD * eD);

  if(abs(pwr) > 255){
    if(pwr > 0){
      pwr = 255;
    }else{
      pwr = -255;
    }
  }

  return pwr;
}