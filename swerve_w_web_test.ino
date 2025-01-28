#include <WiFi.h>
#include <ESPAsyncWebServer.h>

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

const char *ssid = "Cik_Mesh_1200_5BA0";
const char *password = "19950420edward";

AsyncWebServer server(80);  // Web server on port 80
int WS_power = 0;  // Variable to store slider value
int sliderValue = 0;  // Variable to store slider value

const int pollingRateLookup[261] = { 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6 };

int cycle = 0;

const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Motor Control</title>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; margin-top: 50px; }
    input[type=range] { width: 80%; }
    .value { font-size: 20px; color: #333; }
  </style>
</head>
<body>
  <h2>Motor Control</h2>
  <p>Adjust power:</p>
  <input type="range" id="slider" min="0" max="255" value="0" oninput="updateSlider(this.value)">
  <p>Power: <span id="sliderValue">0</span></p>
  <h3>Motor Speeds</h3>
  <p>Motor 1: <span id="motor1Speed">0</span> RPM</p>
  <p>Motor 2: <span id="motor2Speed">0</span> RPM</p>

  <script>
    function updateSlider(value) {
      document.getElementById('sliderValue').innerText = value;
      fetch(`/setPower?value=${value}`);
    }
    setInterval(() => {
      fetch('/getSpeeds')
        .then(response => response.json())
        .then(data => {
          document.getElementById('motor1Speed').innerText = data.motor1;
          document.getElementById('motor2Speed').innerText = data.motor2;
        });
    }, 500);
  </script>
</body>
</html>
)rawliteral";

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
    long currentTime = micros();
    float dT = ((float)(currentTime - previousTime)) / 1.0e6;
    float speed = (encoderCount - prevEncoderCount) / dT;
    prevEncoderCount = encoderCount;
    previousTime = currentTime;
    speedPrevious = speed;
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

    // Serial.print("ADJ error: ");
    // Serial.println(adjustedError);
    motor1.setSpeed(speed + adjustedError);
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

    if (abs(currentAngle - targetAngle) > 2) {  //2 degree tolerance
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

    Serial.print(motor1.getSpeed());
    Serial.print(" | ");
    Serial.print(motor2.getSpeed());
    Serial.print(" | ");
    Serial.println(abs(currentAngle - targetAngle));
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
  Serial.begin(115200);
  pinMode(encA1, INPUT);
  pinMode(encB1, INPUT);
  pinMode(encA2, INPUT);
  pinMode(encB2, INPUT);
  attachInterrupt(digitalPinToInterrupt(encA1), tickEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encA2), tickEncoder2, RISING);
  motor1.setSpeedPIDconstants(kP, kI, kD, kD2);
  motor2.setSpeedPIDconstants(kP, kI, kD, kD2);
  module.setPIDconstants(100, 0, 0);  //raise P, add an I term
  module.setSpeedAdjPIDconstants(15, 0, 0);
  module.setTargetSpeed(200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());

  // Serve the HTML page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", webpage);
  });

  // Handle slider input
  server.on("/setPower", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      sliderValue = request->getParam("value")->value().toInt();
      Serial.println("Slider Value: " + String(sliderValue));
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/getSpeeds", HTTP_GET, [](AsyncWebServerRequest *request) {
    int motor1Speed = motor1.getSpeed();  // Replace with actual function
    int motor2Speed = motor2.getSpeed();  // Replace with actual function
    String json = "{\"motor1\":" + String(motor1Speed) + ",\"motor2\":" + String(motor2Speed) + "}";
    request->send(200, "application/json", json);
  });

  server.begin();
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
  // module.setTargetAngle(30);
  // module.setTargetSpeed(200);
  // Serial.print(motor1.getSpeed());
  // Serial.print(" | ");
  // Serial.print(motor2.getSpeed());
  // Serial.print(" | ");
  // Serial.print(module.getAngle() - module.getTargetAngle());
  // Serial.print(" | ");
  // Serial.println(pollingRateLookup[speed]);
  // Serial.println(module.getAngle());
  // module.update();
  module.setPower(sliderValue);

  Serial.print(motor)

}
