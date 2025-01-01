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

int cycle = 0;

class Motor {
public:
  Motor(int new_fwdPin, int new_bwdPin, int new_enablePin): fwdPin(new_fwdPin), bwdPin(new_bwdPin), enablePin(new_enablePin) {
    encoderCount = 0;
    prevEncoderCount = 0;
  }

  void initialize() {
    pinMode(fwdPin, OUTPUT);
    pinMode(bwdPin, OUTPUT);
  }

  void setPIDconstants(double kP, double kI, double kD){
    PIDconstants[0] = kP;
    PIDconstants[1] = kI;
    PIDconstants[2] = kD;
  }

  void setPower(int power) {
    if (power > 0) {
      analogWrite(enablePin, abs(power));
      digitalWrite(fwdPin, HIGH);
      digitalWrite(bwdPin, LOW);
    } else {
      analogWrite(enablePin, abs(power));
      digitalWrite(fwdPin, LOW);
      digitalWrite(bwdPin, HIGH);
    }
  }

  void setSpeed(float targetSpeed){
    long currentTime = micros();
    float dT = ((float)(currentTime - previousTime)) / 1.0e6;
    float speed = (encoderCount - prevEncoderCount) / dT;

    if((cycle % 50) != 0){

      setPower(previousPower);
      return;
    }

    float eP = targetSpeed - speed;
    eI = eI + (eP*dT);
    float eD = (eP - ePPrevious)/dT;

    prevEncoderCount = encoderCount;
    previousTime = currentTime;
    ePPrevious = eP;
    speedPrevious = speed;

    float power = (PIDconstants[0] * eP) + (PIDconstants[1] * eI) + (PIDconstants[2] * eD);

    if(abs(power) > 255){
      power = 255;
    }
    previousPower = previousPower + power;
    setPower(previousPower);
  }

  int getEncoderCount() {
    return encoderCount;
  }

  void setEncoderCount(int new_encoderCount) {
    encoderCount = new_encoderCount;
  }

  void incEncoderCount() {
    encoderCount++;
  }

  void decEncoderCount() {
    encoderCount--;
  }

  float getSpeed(){
    return speedPrevious;
  }

private:
  int fwdPin;
  int bwdPin;
  int enablePin;
  int encoderCount;
  float PIDconstants[6] = {};
  int prevEncoderCount = 0;
  long previousTime = 0;
  float speedPrevious = 0;
  float ePPrevious = 0;
  float eI;
  float previousPower;
};

class Module {
public:
  Module(Motor& new_motor1, Motor& new_motor2)
    : motor1(new_motor1), motor2(new_motor2) {
    motor1.initialize();
    motor2.initialize();
  }

  void setPower(int power) {
    motor1.setPower(power);
    motor2.setPower(-power);
  }

  void setSpeed(int speed){
    motor1.setSpeed(speed);
    motor2.setSpeed(-speed);
  }

  int getEncoderOffset() {
    return (motor2.getEncoderCount()) + (motor1.getEncoderCount());
  }

private:
  Motor& motor1;
  Motor& motor2;
};

Motor motor1(2, 4, 15);
Motor motor2(18, 19, 5);

Module module(motor1, motor2);

void tickEncoder1() {
  if (digitalRead(encA1) > digitalRead(encB1)) {
    motor1.incEncoderCount();
  } else {
    motor1.decEncoderCount();
  }
}

void tickEncoder2() {
  if (digitalRead(encA2) > digitalRead(encB2)) {
    motor2.incEncoderCount();
  } else {
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
  motor1.setPIDconstants(0.75, 0.005, 0.01);
  motor2.setPIDconstants(0.75, 0.005, 0.01);
  Serial.begin(9600);
}

float previousCalculate = 0;
float previousCalculate2 = 0;


void loop() {
  cycle++;
  
  module.setSpeed(200);

  // motor1.setSpeed(300, true);
  // motor2.setSpeed(300, false);
  Serial.print(motor1.getSpeed());
  Serial.print(" | ");
  Serial.println(motor2.getSpeed());
  // Serial.println(motor2.getSpeed());
  // Serial.println(module.getEncoderOffset());
  
}
