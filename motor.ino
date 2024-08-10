class Motor
{
  int fwdPin;
  int bwdPin;
  int enablePin;
  int encPinA;
  int encPinB;
  
  int encoderCount;
  int kP;
  int kI;
  int kD;
  
  float eI = 0;
  float ePPrevious = 0;
  float previousTime = 0;

  public:
  Motor(int fwdPin, int bwdPin, int enablePin, int encPinA, int encPinB)
  {
    this->fwdPin = fwdPin;
    this-> bwdPin = bwdPin;
    this-> enablePin = enablePin;
    this-> encPinA = encPinA;
    this-> encPinB = encPinB;
    encoderCount = 0;
  }


  void initialize(){
    pinMode(fwdPin, OUTPUT);
    pinMode(bwdPin, OUTPUT);
    pinMode(encPinA, INPUT);
    pinMode(encPinB, INPUT);
  }

  void setEncoderCount(int count){
    encoderCount = count;
  }

  int getEncoderCount(){
    return encoderCount;
  }

  void setPIDConstants(int kP, int kI, int kD){
    this-> kP = kP;
    this-> kI = kI;
    this-> kD = kD;    
  }

//  float calculatePID(int targetEncoderCount){
//    long currentTime = micros();
//    float dT = ((float)(currentTime - previousTime)) / 1.0e6;
//
//    int eP = targetEncoderCount - encoderCount;
//    eI = eI + (eP * dT);
//    float eD = (eP - ePPrevious)/dT;
//
//    previousTime = currentTime;
//    ePPrevious = eP;
//
//    float pwr = (kP * eP) + (kI * eI) + (kD * eD);
//
//    if(abs(pwr) > 255){
//      if(pwr > 0{
//        pwr = 255;
//      }else{
//        pwr = -255;
//      }
//    }
//
//    return pwr;
//    Serial.print(pwr);
//  }

//  void setPosition(targetEncoderCount){
//    float pwr = calculatePID(targetEncoderCount);
//    if(pwr > 0){
//      analogWrite(enablePin, abs(pwr));
//      digitalWrite(fwdPin, HIGH);
//      digitalWrite(bwdPin, LOW);
//    }else{
//      analogWrite(enablePin, abs(pwr));
//      digitalWrite(fwdPin, LOW);
//      digitalWrite(bwdPin, HIGH);
//    }
//  }

};


//Motor motorA {Motor(16, 17, 15, 18, 19)};
Motor motorA(16, 17, 15, 18, 19);
int encoderCount = 0;

static void tickEncoder(){
    if(digitalRead(18) > digitalRead(19)){
      encoderCount++;
    }else{
      encoderCount--;
    }
}

void setup() {
  motorA.initialize();
  motorA.setPIDConstants(1, 0.4, 0.01);
  attachInterrupt(digitalPinToInterrupt(18), tickEncoder, RISING);  
  Serial.begin(9600);
}

void loop() {
  motorA.setPIDConstants(1, 0.7, micros());
  
  Serial.print(", ");  
  Serial.println(motorA.getEncoderCount());
}


