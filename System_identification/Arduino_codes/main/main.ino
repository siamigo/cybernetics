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
  int target = 6620;

  float kp = 0.003531;
  float kd = 0.0001133;
  float ki = 0.003472;

  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = posi;
  }

  int e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 127 ){
    pwr = 127;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);


  // store previous error
  eprev = e;

  float targetAngle = (360./1024.)*target;
  int voltage = pwr/255.*12.*1000.;
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
