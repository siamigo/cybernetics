#include <AVR_RTC.h>
#include <util/atomic.h>


#define ENCA 2
#define ENCB 3
#define PWM 11
#define IN2 6
#define IN1 7

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;
int pwrT = 0;
int pwr = 24;
int dir = 1;
int target = 6200;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  
  Serial.println("target pos"); 

}

void loop() {
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = posi;
  }
  
  if (pos >= 512)
  {
    pwr = 64;
  }
  else if (pos >= 1024)
    {
        pwr = 22;
    }
    else if (pos >= 1340)
    {
        pwr = 64;
    }
    else if (pos >= 2400)
    {
        pwr = 96;
    }
    else if (pos >= 4400)
    {
        pwr = 33;
    }
    else if (pos >= 5500)
    {
        pwr = 12;
    }
    else if (pos >= 6000)
    {
        pwr = 4;
    }
    else if (pos >= target)
    {
        pwr = 0;
    }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);

  float targetAngle = (360./1024.)*target;
  int voltage = pwrT/255.*12.*1000.;
  float angle = (360./1024.)*pos;
  int T = millis();
  
  Serial.print(targetAngle);
  Serial.print(" ");
  Serial.print(voltage);
  Serial.print(" ");
  Serial.print(angle);
  Serial.print(" ");
  Serial.print(T);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}
