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

int encoderCount[] = {0, 0};

class Motor{
  public:
    Motor(int new_fwdPin, int new_bwdPin, int new_enablePin): fwdPin(new_fwdPin), bwdPin(new_bwdPin), enablePin(new_enablePin)
    {

    }

    void initialize(){
      pinMode(fwdPin, OUTPUT);
      pinMode(bwdPin, OUTPUT);
    }    
    
    void setPower(int power){
        if(power > 0){
        analogWrite(enablePin, abs(power));
        digitalWrite(fwdPin, HIGH);
        digitalWrite(bwdPin, LOW);
      }else{
        analogWrite(enablePin, abs(power));
        digitalWrite(fwdPin, LOW);
        digitalWrite(bwdPin, HIGH);
      }
    }

    int getEncoderCount(){
      return encoderCount;
    }

    void setEncoderCount(int new_encoderCount){
      encoderCount = new_encoderCount;
    }

    void incEncoderCount(){
      encoderCount++;
    }

    void decEncoderCount(){
      encoderCount--;
    }

  private:
    int fwdPin;
    int bwdPin;
    int enablePin;
    int encoderCount;
};

Motor motor1(2, 4, 15);
Motor motor2(18, 19, 5);

void tickEncoder1(){
  if(digitalRead(encA1) > digitalRead(encB1)){
    motor1.incEncoderCount();
  }else{
    motor1.decEncoderCount();
  }
}

void tickEncoder2(){
  if(digitalRead(encA2) > digitalRead(encB2)){
    motor2.incEncoderCount();
  }else{
    motor2.decEncoderCount();
  }
}


void setup() {
  // put your setup code here, to run once:
  motor1.initialize();
  motor2.initialize();
  pinMode(encA1, INPUT);
  pinMode(encB1, INPUT);
  pinMode(encA2, INPUT);
  pinMode(encB2, INPUT);
  attachInterrupt(digitalPinToInterrupt(encA1), tickEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encA2), tickEncoder2, RISING);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  motor1.setPower(-255);
  motor2.setPower(255);
  Serial.println(motor1.getEncoderCount());
  Serial.println(motor2.getEncoderCount());
}
