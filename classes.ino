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
    // encoder = new_encoder
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

  private:
    int fwdPin;
    int bwdPin;
    int enablePin;
    // Encoder encoder;
};

Motor motor1(2, 4, 15);
Motor motor2(18, 19, 5);

void setup() {
  // put your setup code here, to run once:
  motor1.initialize();
  motor2.initialize();
}

void loop() {
  // put your main code here, to run repeatedly:
  motor1.setPower(-150);
  motor2.setPower(150);

}

// class Encoder{
//   int encPinA;
//   int encPinB;
//   int count;
//   public:
//     Encoder(int new_encPinA, int new_encPinB);
// };

// Encoder::Encoder():
//   encPinA = new_encPinA,
//   encPinB = new_encPinB{
//     count = 0;

// }
