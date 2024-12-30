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

class Module{
  public:
    Module(Motor& new_motor1, Motor& new_motor2): motor1(new_motor1), motor2(new_motor2){
        motor1.initialize();
        motor2.initialize();
    }

    void setPower(int power){
      motor1.setPower(power);
      motor2.setPower(-pwer);
    }

  int getEncoderOffset(){
    return (motor2.getEncoderCount()) + (motor1.getEncoderCount());
  }

  private:
    Motor& motor1;
    Motor& motor2;
};

Motor motor1(2, 4, 15);
Motor motor2(18, 19, 5);

Module module(motor1, motor2);

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
  pinMode(encA1, INPUT);
  pinMode(encB1, INPUT);
  pinMode(encA2, INPUT);
  pinMode(encB2, INPUT);
  attachInterrupt(digitalPinToInterrupt(encA1), tickEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encA2), tickEncoder2, RISING);
  Serial.begin(9600);

}

void loop() {
  module.setSpeed(200);
  Serial.print(motor1.getEncoderCount());
  Serial.print(" | ");
  Serial.print(motor2.getEncoderCount());
  Serial.print(" | ");
  Serial.println(module.getEncoderOffset());
}
