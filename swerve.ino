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

const int pollingRateLookup[261] = { 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6 };

int cycle = 0;

class Motor {
public:
  Motor(int new_fwdPin, int new_bwdPin, int new_enablePin)
    : fwdPin(new_fwdPin), bwdPin(new_bwdPin), enablePin(new_enablePin) {
    encoderCount = 0;
    prevEncoderCount = 0;
  }

  void initialize() {
    pinMode(fwdPin, OUTPUT);
    pinMode(bwdPin, OUTPUT);
  }

  void setSpeedPIDconstants(double kP, double kI, double kD, double kD2) {
    PIDconstants[0] = kP;
    PIDconstants[1] = kI;
    PIDconstants[2] = kD;
    PIDconstants[3] = kD2;
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

  void setSpeed(float targetSpeed) {
    if (targetSpeed == 0) {
      setPower(0);
      return;
    }


    int pollingRate;
    if (abs(targetSpeed) > 260) {
      pollingRate = 5;
    } else {
      pollingRate = pollingRateLookup[abs((int)round(targetSpeed))];
    }

    long currentTime = micros();
    float dT = ((float)(currentTime - previousTime)) / 1.0e6;
    float speed = (encoderCount - prevEncoderCount) / dT;

    if ((cycle % pollingRate) != 0) {
      setPower(previousPower);
      return;
    }

    float eP = targetSpeed - speed;
    eI = eI + (eP * dT);
    float eD = (eP - ePPrevious) / dT;

    prevEncoderCount = encoderCount;
    previousTime = currentTime;
    ePPrevious = eP;
    speedPrevious = speed;

    float power;

    if (abs(targetSpeed) > 125) {
      power = (PIDconstants[0] * eP) + (PIDconstants[1] * eI) + (PIDconstants[2] * eD);
    } else {
      power = (PIDconstants[0] * eP * 0.75) + (PIDconstants[1] * eI) + (PIDconstants[2] * eD);
    }

    if (abs(power) > 255) {
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

  float getSpeed() {
    return speedPrevious;
  }

private:
  int fwdPin;
  int bwdPin;
  int enablePin;
  int encoderCount;
  float PIDconstants[4] = {};
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

  void turn(int power) {
    motor1.setPower(power);
    motor2.setPower(power);
  }

  void setSpeed(float speed) {
    //calculate adjusted error using a PID loop, using targetAngle - getAngle() as e(t)
    long currentTime = micros();
    float dT = ((float)(currentTime - previousTime)) / 1e06;

    float eP = targetAngle - getAngle();
    eI = eI + (eP * dT);
    float eD = (eP - ePPrevious) / dT;

    float adjustedError = (int)((eP * PIDconstants[3]) + (eI * PIDconstants[4]) + (eD * PIDconstants[5]));

    Serial.print("ADJ error: ");
    Serial.println(adjustedError);

    motor1.setSpeed(speed - adjustedError);
    motor2.setSpeed(-speed + adjustedError);
  }

  void setPIDconstants(float kP, float kI, float kD) {
    PIDconstants[0] = kP;
    PIDconstants[1] = kI;
    PIDconstants[2] = kD;
  }

  void setSpeedAdjPIDconstants(float kP, float kI, float kD){
    PIDconstants[3] = kP;
    PIDconstants[4] = kI;
    PIDconstants[5] = kD;
  }

  void turnToAngle(float targetAngle) {
    long currentTime = micros();
    float dT = ((float)(currentTime - previousTime)) / 1.0e6;

    float eP = targetAngle - getAngle();
    eI = eI + (eP * dT);
    float eD = (eP - ePPrevious) / dT;

    int power = (int)((eP * PIDconstants[0]) + (eI * PIDconstants[1]) + (eD * PIDconstants[2]));

    if (abs(power) > 255) {
      if (power < 0) {
        power = -255;
      } else {
        power = 255;
      }
    }

    // Serial.print(power);
    // Serial.print(" | ");

    turn(power);
  }

  int getEncoderOffset() {
    return (motor2.getEncoderCount()) + (motor1.getEncoderCount());
  }

  float getAngle() {
    return (float)(((motor1.getEncoderCount() + motor2.getEncoderCount()) % 25000) * 0.0144);  // 0.0144 is equivalent to * 360 / 25000
  }

  void update() {
    float currentAngle = getAngle();

    if (abs(currentAngle - targetAngle) > 3) {  //3 degree tolerance
      if (prevState == 1) {
        previousTime = micros();
        eI = 0;
        ePPrevious = 0;
        prevState = 0;
      }
      turnToAngle(targetAngle);
    } else {
      if (prevState == 0) {
        previousTime = micros();
        eI = 0;
        ePPrevious = 0;
        prevState = 1;
      }
      setSpeed(targetSpeed);  //temporarily 6 but i need to make a lookup table
    }
  }

  void setTargetSpeed(float new_targetSpeed) {
    targetSpeed = new_targetSpeed;
  }

  void setTargetAngle(float new_targetAngle) {
    targetAngle = new_targetAngle;
  }

  float getTargetAngle(){
    return targetAngle;
  }

private:
  Motor& motor1;
  Motor& motor2;
  float PIDconstants[6] = {};
  float previousTime = 0;
  float eI = 0;
  float ePPrevious = 0;
  float targetAngle = 0;
  float targetSpeed = 0;
  int prevState = 0;  //0 --> turning, 1 --> speed control
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

double kP = 0.5;
double kI = 0;
double kD = 0;
double kD2 = 0.125;

void setup() {
  pinMode(encA1, INPUT);
  pinMode(encB1, INPUT);
  pinMode(encA2, INPUT);
  pinMode(encB2, INPUT);
  attachInterrupt(digitalPinToInterrupt(encA1), tickEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encA2), tickEncoder2, RISING);
  motor1.setSpeedPIDconstants(kP, kI, kD, kD2);
  motor2.setSpeedPIDconstants(kP, kI, kD, kD2);
  module.setPIDconstants(100, 0, 0);  //raise P, add an I term
  module.setSpeedAdjPIDconstants(10, 0, 0);
  Serial.begin(9600);
  module.setTargetSpeed(200);
  // module.setTargetAngle(30);
}

float previousCalculate = 0;
float previousCalculate2 = 0;

int speed = 300;
// int pollingRate = round((double)(0.0002 * speed * speed) - (0.134 * speed) + 27.2);
//0.0002x^2 - 0.134x + 26.2
// int pollingRate = round((double)(-0.00009 *speed *speed) + (0.0022 * speed) + 10.972);
//-0.00009x^2 + 0.0022x + 10.972


void loop() {
  cycle++;
  // if(Serial.available() > 0){
  //   speed = Serial.read();
  // }
  // motor1.setPower(255);
  // motor2.setPower(255);
  // module.setSpeed(speed);
  // module.turnToAngle(90);
  module.setTargetAngle(30);
  module.setTargetSpeed(200);
  // Serial.print(motor1.getSpeed());
  // Serial.print(" | ");
  // Serial.print(motor2.getSpeed());
  // Serial.print(" | ");
  Serial.print(module.getAngle() - module.getTargetAngle());
  Serial.print(" | ");
  // Serial.println(pollingRateLookup[speed]);
  // Serial.println(module.getAngle());
  module.update();

  // motor1.setSpeed(300, true);
  // motor2.setSpeed(300, false);
  // Serial.print(motor1.getSpeed());
  // Serial.print(" | ");
  // Serial.print(motor2.getSpeed());
  // Serial.print(" | ");
  // Serial.print(speed);
  // Serial.print(" | ");
  // Serial.print(-speed);
  // Serial.print(" | ");
  // Serial.print(pollingRate);
  // Serial.print(" | ");
  // Serial.print(module.getEncoderOffset());
  // Serial.print(" | ");
  // Serial.println(module.getAngle());


  // Serial.println(motor2.getSpeed());
  // Serial.println(module.getEncoderOffset());
}
