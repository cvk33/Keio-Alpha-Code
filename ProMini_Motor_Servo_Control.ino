/*Based Code For Hyperloop Micro Pod Control System
This code is written by David Chew
It is part of the sensor and control architecture.
Updated on 15 May. */

/* Description of architecture
 Yun as Main Brain
 Uno as Sensor Input
 Leonardo as PWM output
 
Requirements:
 This is the code for Leonardo
 Leonardo should receive i2c command from Yun (req1)
 and send PWM signal to Motor Controller (req2)
 and send PWM setting to Comm User via i2c (req3)
 */

// Leonardo is Slave in I2C//
#include <Wire.h>

#define servopinLF 10
#define servopinRF 11
#define servopinLB 9
#define servopinRB 6

#define SERVIN_LF 1800
#define SERVFLAT_LF 1700
#define SERVOUT_LF 1600
#define SERVIN_RF 1350
#define SERVFLAT_RF 1450
#define SERVOUT_RF 1550
#define SERVIN_LB 950
#define SERVFLAT_LB 1050
#define SERVOUT_LB 1150
#define SERVIN_RB 1450
#define SERVFLAT_RB 1350
#define SERVOUT_RB 1290

#define PWMPIN 5

int x=0;
int j=0;
int pulsems=1300;
int halfwidth=10000;

void setup() {
  Wire.begin(9); //join i2c bus as address 9
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  //Serial.begin(9600);
  pinMode(13, OUTPUT); //LED pin 
  pinMode(PWMPIN, OUTPUT); // pin 5 is PWM pin
  pinMode(servopinLF,OUTPUT);
  pinMode(servopinRF,OUTPUT);
  pinMode(servopinLB,OUTPUT);
  pinMode(servopinRB,OUTPUT);
}

void loop() {
    
  digitalWrite(PWMPIN,HIGH);   //(req 2)
  delayMicroseconds(pulsems);
  digitalWrite(PWMPIN, LOW);
  delayMicroseconds(halfwidth-pulsems);
  delayMicroseconds(halfwidth);  //we need 20ms but this code can read until 16343 so we delay 10ms twice
  
  if (x==1){
    pulsems=1680;
    digitalWrite(PWMPIN,HIGH);
    delayMicroseconds(pulsems);
    digitalWrite(PWMPIN, LOW);
    delayMicroseconds(halfwidth-pulsems);
    delayMicroseconds(halfwidth);
  }else if(x==2){
    pulsems=1400;
    digitalWrite(PWMPIN,HIGH);
    delayMicroseconds(pulsems);
    digitalWrite(PWMPIN, LOW);
    delayMicroseconds(halfwidth-pulsems);
    delayMicroseconds(halfwidth);
  }else if(x==3){
    pulsems=1300;
    digitalWrite(PWMPIN,HIGH);
    delayMicroseconds(pulsems);
    digitalWrite(PWMPIN, LOW);
    delayMicroseconds(halfwidth-pulsems);
    delayMicroseconds(halfwidth);
  }else if(x==5){
    front();
  }else if(x==6){
    flat();
  }else if(x==7){
    back();
  }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()

void receiveEvent(int howMany) {  //(req1)
  x = Wire.read();    // receive byte as an integer
  //Serial.println(x);// print the integer 
}

void requestEvent(){    //req3)
  digitalWrite (13,HIGH);
  if (x==1){ Wire.write("LEO9_Full");}
  else if(x==2){ Wire.write("LEO9_Half");}
  else if(x==3){ Wire.write("LEO9_Stop");}
  else if(x==4){ Wire.write("LEO9_Cheq");}
  else { Wire.write("LEO9_None");}
  delay(100);
  digitalWrite(13,LOW);
}

void front(){
  for(j=0;j<20;j++){
    digitalWrite(servopinLF,HIGH);
    delayMicroseconds(SERVIN_LF);
    digitalWrite(servopinLF,LOW);
  }
  for(j=0;j<20;j++){
    digitalWrite(servopinRF,HIGH);
    delayMicroseconds(SERVIN_RF);
    digitalWrite(servopinRF,LOW);
  }
  for(j=0;j<20;j++){
    digitalWrite(servopinLB,HIGH);
    delayMicroseconds(SERVOUT_LB);
    digitalWrite(servopinLB,LOW);
  }
  for(j=0;j<20;j++){
    digitalWrite(servopinRB,HIGH);
    delayMicroseconds(SERVOUT_RB);
    digitalWrite(servopinRB,LOW);
  }
}
void back(){
  for(j=0;j<20;j++){
    digitalWrite(servopinLF,HIGH);
    delayMicroseconds(SERVOUT_LF);
    digitalWrite(servopinLF,LOW);
  }
  for(j=0;j<20;j++){
    digitalWrite(servopinRF,HIGH);
    delayMicroseconds(SERVOUT_RF);
    digitalWrite(servopinRF,LOW);
  }
  for(j=0;j<20;j++){
    digitalWrite(servopinLB,HIGH);
    delayMicroseconds(SERVIN_LB);
    digitalWrite(servopinLB,LOW);
  }
  for(j=0;j<20;j++){
    digitalWrite(servopinRB,HIGH);
    delayMicroseconds(SERVIN_RB);
    digitalWrite(servopinRB,LOW);
  }
}
void flat(){
  for(j=0;j<20;j++){
    digitalWrite(servopinLF,HIGH);
    delayMicroseconds(SERVFLAT_LF);
    digitalWrite(servopinLF,LOW);
  }
  for(j=0;j<20;j++){
    digitalWrite(servopinRF,HIGH);
    delayMicroseconds(SERVFLAT_RF);
    digitalWrite(servopinRF,LOW);
  }
  for(j=0;j<20;j++){
    digitalWrite(servopinLB,HIGH);
    delayMicroseconds(SERVFLAT_LB);
    digitalWrite(servopinLB,LOW);
  }
  for(j=0;j<20;j++){
    digitalWrite(servopinRB,HIGH);
    delayMicroseconds(SERVFLAT_RB);
    digitalWrite(servopinRB,LOW);
  }
}

