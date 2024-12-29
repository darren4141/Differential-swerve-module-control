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

// class Encoder{
  
//   public:
//     int encPinA;
//     int encPinB;
//     int count;
//     Encoder(int new_encPinA, int new_encPinB): encPinA(new_encPinA), encPinB(new_encPinB), count(0){
//       pinMode(encPinA, INPUT);
//       pinMode(encPinB, INPUT);
//     }
    
//     int getCount(){
//       // Serial.println(count);
//       return count;
//     }
//     void incEncoder(){
//       count++;
//     }

//     // static void handleInterrupt(){
//     //   Serial.println("a");
//     //   if(instance){
//     //     instance->tickEncoder();
//     //   }
//     // }

//     // void tickEncoder(){
//     //   Serial.print("ticking encoder ");
//     //   Serial.println(encPinA);
//     //   if(digitalRead(encPinA) > digitalRead(encPinB)){
//     //     count++;
//     //   }else{
//     //     count--;
//     //   }
//     // }

//     // static void setInstance(Encoder *new_instance){
//     //   instance = new_instance;
//     // }



//   private:
// };

class Motor{
  public:
    Motor(int new_fwdPin, int new_bwdPin, int new_enablePin, Encoder new_encoder): fwdPin(new_fwdPin), bwdPin(new_bwdPin), enablePin(new_enablePin), encoder(new_encoder)
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
    Encoder encoder;
};

void tickEncoder1(){
  Serial.print("ticking encoder ");
  Serial.println(encA1);
  if(digitalRead(encA1) > digitalRead(encB1)){
    encoderCount[0]++;
  }else{
    encoderCount[0]--;
  }
}

void tickEncoder2(){
  Serial.print("ticking encoder ");
  Serial.println(encA2);
  if(digitalRead(encA2) > digitalRead(encB2)){
    encoderCount[1]++;
  }else{
    encoderCount[1]--;
  }
}

Encoder encoder1(16, 17);
Encoder encoder2(22, 23);

Motor motor1(2, 4, 15, encoder1);
Motor motor2(18, 19, 5, encoder2);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  motor1.initialize();
  motor2.initialize();

  // encoder1.setInstance(&encoder1);
  // encoder2.setInstance(&encoder2);
  attachInterrupt(digitalPinToInterrupt(16), tickEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(22), tickEncoder2, RISING);

}

void loop() {
  // put your main code here, to run repeatedly:
  motor1.setPower(-150);
  motor2.setPower(150);
  Serial.println(encoderCount[0]);
  Serial.println(encoderCount[1]);
}
