const int fwdPin = 16;
const int bwdPin = 17;
const int enablePin = 15;
const int encA = 18;
const int encB = 19;

int encoderCount = 0;

float eI = 0;
float ePPrevious = 0;
float previousTime = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(fwdPin, OUTPUT);
  pinMode(bwdPin, OUTPUT);

  pinMode(encA, INPUT);
  pinMode(encB, INPUT);

  attachInterrupt(digitalPinToInterrupt(encA), tickEncoder, RISING);
  
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(fwdPin, HIGH);
  digitalWrite(bwdPin, LOW);

  float pwr = pidControl(1000, 1, 0.4, 0.01);

  if(pwr > 0){
    analogWrite(enablePin, abs(pwr));
    digitalWrite(fwdPin, HIGH);
    digitalWrite(bwdPin, LOW);
  }else{
    analogWrite(enablePin, abs(pwr));
    digitalWrite(fwdPin, LOW);
    digitalWrite(bwdPin, HIGH);
  }
  

  
  
  Serial.print(encoderCount);
  Serial.print(", ");
  Serial.println(pwr);
}

void tickEncoder(){
  if(digitalRead(encA) > digitalRead(encB)){
    encoderCount++;
  }else{
    encoderCount--;
  }
}

float pidControl(int target, float kP, float kI, float kD){
  long currentTime = micros();
  float dT = ((float)(currentTime - previousTime)) / 1.0e6;

  int eP = target - encoderCount;
  eI = eI + (eP * dT);
  float eD = (eP - ePPrevious)/dT;

  previousTime = currentTime;
  ePPrevious = eP;

  float pwr = (kP * eP) + (kI * eI) + (kD * eD);

  if(pwr > 255){
    pwr = 255;
  }

  return (kP * eP) + (kI * eI) + (kD * eD);
}
