/*Based Code For Hyperloop Micro Pod Control System
This code is written by David Chew
It is part of the sensor and control architecture.
Updated on 7 June. */

/*Description of Architecture
 Yun as Main Brain
 Uno as Sensor Input
 Leonardo as PWM output
 
 Requirements:
 This is the code for Yun
 Yun should receive Serial Input from Comm User (req1)
 and send PWM command to Leonardo via i2C       (req2)
 and receive Sensor Input from V6180X i2C sensor (req3)
 and receive message from UNO (req4)
 and send to Comm User via Serial(req5)
 */

// Yun is Master in I2C//
#include <Wire.h>            //for i2C
#include <VL6180X.h>        // for proximity sensor
#include <Bridge.h>         // uControl to Linux Device 
#include <YunClient.h>      // TCP/IP connection


//------------------------VL6180X sensor setup----------
VL6180X sensor;
uint8_t sensorValue = 0;      // the sensor value
int sensorMin = 0;            // minimum sensor value
int sensorMax = 200; 
int calibration=50;

//constants for smoothing of proximate sensor
const int numReadings = 3;
int readings[numReadings];    // the readings from the analog input
int readIndex = 0;            // the index of the current reading
int total = 0;                // the running total
int average = 0;              // the average

// these constants describe the pins. They won't change:
const int groundpin = 18;     // analog input pin 4 -- ground
const int powerpin = 19;      // analog input pin 5 -- voltage
const int xpin = A3;          // x-axis of the accelerometer
const int ypin = A2;          // y-axis
const int zpin = A1;          // z-axis (only on 3-axis models)
int xx=0;             //Current Voltage value of X-axis of accelerate sensor
int yy=0;             //Current Voltage value of Y-axis of accelerate sensor
int zz=0;             //Current Voltage value of Z-axis of accelerate sensor
double speedx=0.0;            //Storage of current speed
int tare=481;                 //Storage of tare of X-axis accelerator sensor
unsigned long recordtime[6];  //Storage of record time for calculate the delay of different parts
unsigned long delaytime[6];   //Storage of delay time for different parts
/**
 * About how to record delay:
 * Start record:  recordtime[i]=micros();
 * Between start & end is your testing code
 * End record:    delaytime[i]=micros()-recordtime[i]
 */

//------------------Define Port Number and client------------------------------
#define PORT 5001   //Change PORT number HERE!!!
YunClient client;
//-------------------------------------------------------
void setup() {
  Bridge.begin();       //establish communication
  Wire.begin();         // establish i2C comm
  //Serial.begin(9600);
  pinMode(13, OUTPUT); //LED pin
  delay (1000);
  //Serial.println("Initializing");
  //delay(4000); 
  
  //------VL6180X sensor initiation----------
  sensor.init();
  sensor.configureDefault();
  sensor.setTimeout(500);

  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  // Provide ground and power by using the analog inputs as normal
  // digital pins.  This makes it possible to directly connect the
  // breakout board to the Arduino.  If you use the normal 5V and
  // GND pins on the Arduino, you can remove these lines.
  pinMode(groundpin, OUTPUT);
  pinMode(powerpin, OUTPUT);
  digitalWrite(groundpin, LOW);
  digitalWrite(powerpin, HIGH);
  
  //-------------------Opening Message----------------------
  
  //Serial.println("Welcome to Keio Alpha Hyperloop...Set your command");
  //Serial.println("Press 1 = Full Power On, 2 = Half Power On, 3 = Power Off, 4 = Status Check");
  
  //setupADC();
}

int incomingByte = 0;               //command 1,2,3,4
unsigned long time1=0;
char cc;
void loop() {
  time1= micros();
  IPAddress addr(192,168,2,11);    //Change the IP Address HERE!!!
                                    //192.168.1.11 for LAN
                                    //192.168.2.11 for Wi-Fi
  client.connect(addr, PORT);
  if(client.connected()){
    client.println("connected!");   //Tell Com that Yun is connected with COMM
  }

  while(client.connected()) {
    //cc = client.read();      //incoming data
    
    while(client.available()){
      recordtime[4]= micros();
      cc = client.read();      //incoming data
      incomingByte=0;          //initiate incomingbyte as 0
      switch(cc){
        case '1':
          incomingByte=1;
          break;
        case '2':
          incomingByte=2;
          break;
        case '3':
          incomingByte=3;
          break;
        case '4':
          incomingByte=4;
          break;
        case '5':
          incomingByte=5;
          break;
        case '6':
          incomingByte=6;
          break;
        case '7':
          incomingByte=7;
          break;
        case '0':
          speedx=0;   // reset the speed to 0
          break;
        case 'r':
          tare=xx;    // Set the current accelerate value as reference value
          break;
        default:
          break;
      }
      if(incomingByte!=0){          //only if incoming byte is none 0, data will be send
        //Serial.println("Start 9 wire.write");
        Wire.beginTransmission(9);  // Transmit to device #9 Leonardo
        Wire.write(incomingByte);   // send 5 byte e.g. 1400     (req2)
        Wire.endTransmission();     // stop transmitting
        //Serial.println("End 9 wire.write");
        digitalWrite(13, LOW);      //indicate Command Sent
        //--------------------Ask Data From UNO-----------------------
        Wire.requestFrom(8, 10);    // request 10 bytes from slave device #8 UNO(re4)
        while (Wire.available()){   // slave may send less than requested
          //Serial.println("Wire.available of 8, 10");
          char c = Wire.read();     // receive a byte as character
          client.print(c);          // print the character (req5)
          //Serial.print(c);
        }
        //------------------Ask Data From Leonardo-------------------   
        Wire.requestFrom(9, 9);     // request 9 bytes from slave device #9 Leonardo (re4)
        while (Wire.available()) {  // slave may send less than requested
          //Serial.println("Wire.available of 9, 9");
          char s = Wire.read();     // receive a byte as int
          client.print(s);          // print the character   (req5)
          //Serial.print(s);
        }
        client.println();           //this is to seperate sensor data from uCont feedback
      }
      
      /**
       * read analog value while receiving control cmd from computer
       */
      recordtime[0]= micros();
      xx=analogRead(xpin);
      yy=analogRead(ypin);
      zz=analogRead(zpin);
      delaytime[0]=micros()-recordtime[0];
      
      UpdateProximitySensor();
      delaytime[4]=micros()-recordtime[4];
    }
    /** 
     *  read analog value while not receiving control cmd from computer
     */
    xx=analogRead(xpin);
    yy=analogRead(ypin);
    zz=analogRead(zpin);

    recordtime[5]= micros();
    UpdateProximitySensor();
    delaytime[5]=micros()-recordtime[5];
  }
  client.stop();
}

void UpdateProximitySensor(){
  /**
   * Start request the proximity sensor
   */
  recordtime[1]= micros();
  sensorValue = sensor.readRangeSingle();
  delaytime[1]=micros()-recordtime[1];  // record the total time of requesting proximity sensor
  
  //sensorValue = map(sensorValue, sensorMin, sensorMax, 0, 255); //convert analog value to mm unit
  sensorValue = constrain(sensorValue, 0, 255);
  //sensorValue=sensorValue-calibration;
  
  // smoothing process starts here....................................
  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = sensorValue;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }  
  average = total / numReadings;        // calculate the average
  
  /**
   * End of request proximity sensor
   */
   
  

  // *********************************************************************
     unsigned long deltatime=micros()-time1;  // Time record for whole period
     time1= micros();                         // Refresh the counter
  // *********************************************************************

  /**
   * Caculation of force of X-axis
   * Caculation of speed
   */
  int diff=xx-tare;
  int range=2;   //define reference value to reduce jitter
  if(diff>range or diff<-range){
    speedx+=(((double)diff)/75.0*9.81) * deltatime/1000000.0;
  }
  double g=(xx-481.0)/75.0*9.81;
  
  /**
   * Put all data into content string
   * and waiting for transmit to computer
   */
  recordtime[2]= micros();
  String content="sen:h1:";
         content+=average;
         content+=":";
         content+=g;
         content+=":";
         content+=xx;
         content+=":";
         content+=speedx;
         content+=":";
         content+=yy;
         content+=":";
         content+=zz;
         content+=":";
         content+=GetTemp2();
         content+=":";
         content+=delaytime[0]/1000.0;
         content+=":";
         content+=delaytime[1]/1000.0;
         content+=":";
         content+=delaytime[2]/1000.0;
         content+=":";
         content+=delaytime[3]/1000.0;
         content+=":";
         content+=deltatime/1000.0;
         content+=":";
         content+=delaytime[4]/1000.0;
         content+=":";
         content+=delaytime[5]/1000.0;

  delaytime[2]=micros()-recordtime[2];
  if (sensor.timeoutOccurred()) { client.print("sen:Height1:TIMEOUT"); }
  recordtime[3]= micros();
  client.println(content);
  delaytime[3]=micros()-recordtime[3];
}

/**
 * This GetTemp2() is used to get temperature of Yun
 */
double GetTemp2(void){
  double t;
  // Set the internal reference and mux for the ATmega32U4.
  ADMUX = 0b11000111;
  ADCSRB |= (1 << MUX5);    // enable the ADC
  delay(5);                 // wait for voltages to become stable. Delay comes from here *****************
  ADCSRA |= _BV(ADSC);      // Start the ADC
                        // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));
  delay(5);                 // Delay comes from here ********************
  ADCSRA |= _BV(ADSC);      // Start the ADC
                        // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));
  byte low = ADCL;
  byte high = ADCH;
  t = (high << 8) | low;
  t = (t-273)/1.00;           //Convert from Kelvin to Celcius plus Offset
  return (t);
}


