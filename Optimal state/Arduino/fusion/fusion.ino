#include <HardWire.h>
#include <VL53L0X.h>
#include <I2C_MPU6886.h>
#include <AVR_RTC.h>
#include <util/atomic.h>

#include <Ethernet.h>
#include <EthernetUdp.h>

VL53L0X range_sensor;
I2C_MPU6886 imu(I2C_MPU6886_DEFAULT_ADDRESS, Wire);

IPAddress ip(192, 168, 10, 240);
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

EthernetUDP udp_server;

char packet_buffer[UDP_TX_PACKET_MAX_SIZE];

#define ENCA 2
#define ENCB 3
#define PWM 11
#define IN2 6
#define IN1 7

volatile int posi = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
int pwrT = 0;
float ar = 9.2 / 2; // Axel radius mm

float Pi = 3.14159;

unsigned int oldT = 0;
unsigned int dt = 1;

void setup() 
{
  Serial.begin(115200);
  Wire.begin();
  Ethernet.begin(mac, ip);
  delay(500);

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);

  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  
  if(!initialize())
     while(true)
       delay(10);

  imu.begin();  
  range_sensor.startContinuous();

  udp_server.begin(8888);
  Serial.println("Setup complete");
}

bool initialize()
{
  range_sensor.setTimeout(500);
  if(!range_sensor.init())
  {
    Serial.println("Failed to detect and initialize range sensor!");
    return false;
  }
  
  if(Ethernet.hardwareStatus() == EthernetNoHardware) 
  {
    Serial.println("Ethernet shield was not found.");
    return false;
  }
  else if(Ethernet.hardwareStatus() == EthernetW5500) 
    Serial.println("Found W5500 ethernet shield");

  if(Ethernet.linkStatus() == LinkOFF) 
  {
    Serial.println("Ethernet::LinkOff: is the cable connected?");
    return false;
  }
  else if(Ethernet.linkStatus() == LinkON)
    Serial.println("Ethernet::LinkOn");
  return true;
}
  
void loop() 
{ 
  int packet_size = udp_server.parsePacket();
  if(packet_size) 
  {
    float accel[3];
    float gyro[3];
    float t;
    float d;

    int pos = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = posi;
    } 

    float angle = (360./1024.)*pos;
    unsigned int T = millis();
    dt = T - oldT;
    oldT = millis();
  
    imu.getAccel(&accel[0], &accel[1], &accel[2]);
    imu.getGyro(&gyro[0], &gyro[1], &gyro[2]);
    imu.getTemp(&t);
    d = range_sensor.readRangeContinuousMillimeters();
    
    String sensor_values;
    sensor_values.concat(angle); sensor_values.concat(",");
    sensor_values.concat(accel[2]*9.81); sensor_values.concat(",");
    sensor_values.concat(d); sensor_values.concat(",");
    sensor_values.concat(dt);

    udp_server.read(packet_buffer, UDP_TX_PACKET_MAX_SIZE);
    float x_k = String(packet_buffer).toFloat();
    Serial.print("Updated x_k:  "); Serial.println(x_k);
    Serial.print("Measured distance:  "); Serial.println(d);

    udp_server.beginPacket(udp_server.remoteIP(), udp_server.remotePort());
    udp_server.write(sensor_values.c_str(), sensor_values.length());
    udp_server.endPacket();

    float targetMm = 175.0;

    float target = targetMm / ar * 1024./(2.*Pi); // Target = theta(rad)/axelRadius * 1024 / 2pi
  
    float kp = 0.03687;
    float kd = 0.004946;
    float ki = 0.0006871;
  
    int e = x_k - target;
  
    // derivative
    float dedt = (e-eprev)/(dt);
  
    // integral
    eintegral = eintegral + e*dt;
  
    // control signal
    float u = kp*e + kd*dt + ki*eintegral;
  
    // motor power
    float pwr = fabs(u);
    if( pwr > 255 ){
      pwr = 255;
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
  
    int voltage = pwrT/255.*12.*1000.;
    /*
    Serial.print(target);
    Serial.print(" ");
    Serial.print(voltage);
    Serial.print(" ");
    Serial.print(x_k);
    Serial.println();*/
  }
}

void printPackageMetaInfo(int packet_size)
{
  Serial.print("Received packet of size ");
  Serial.println(packet_size);
  Serial.print("From ");
  IPAddress remote = udp_server.remoteIP();
  for(int i = 0; i < 4; i++) 
  {
    Serial.print(remote[i], DEC);
    if(i < 3) 
      Serial.print(".");
  }
  Serial.print(", port ");
  Serial.println(udp_server.remotePort());
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
