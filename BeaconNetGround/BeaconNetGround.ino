//Primary code for the airborne system for BEACONNet
//Targeted for the Arduino Due
//Developed against TinyGPS++ v0.94b

/*
Purpose of this code is to provide the BEACONNet functionality to
the Arduino Due.  Primary goal is to parse a gps feed and sample
the built in sensors, generating the proprietary NMEA-compatable 
$PEDGE sentence.  Future implmentations will include support for
$PEDGM (broadcast message), $PEDGD (other data), $PEDGC (command 
from ground) and $PEDGA (ack) sentences

Port Assignments:
Serial: Computer connection 1, all data + commands
SerialUSB: NMEA Reconstruction (Future Ground Only, intended to be Tx only)
Serial1: GPS (Rx only)
Serial2: 9xTend (Tx/Rx) and OpenLog (Tx only)
Serial3: Header (not connected - future expansion - could be used to replace Serial for Uno compatibility)

Analog assignments:
A7: Rail voltage (should be within a couple of bits of max)
A8: Temperature
A9: Pressure
A10: Humidity
A11: Ground (should be within a couple of bits of min)
*/

void setup()
{ 
  //Set up the serial ports - the two USB ports connected to the computer are set for high-
  //speed operation to make things happen as quickly as possible
  Serial.begin(115200);
  Serial1.begin(4800);
  Serial2.begin(115200);
  Serial3.begin(115200);
  SerialUSB.begin(115200);
}

void loop()
{
  if(Serial2.available())
  {
    Serial.write(Serial2.read());
  }
}


//To do: parse PEDGE messages, enable placement of my position on the map, enable sending and receiving messages






/*
#include <TinyGPS++.h>

#define DEMO_MODE 0 //set this to 0 for flight configuration; otherwise, the two following switches matter
#define ANALOG_DATA 0 //use live analog data in demo mode
#define GPS_DATA 0 //use live GPS data in demo mode
#define REPORT_FREQ 5 //how many seconds between reports
#define BALLOON_ID 11 //this needs to be different for each balloon.  DON'T FORGET!

TinyGPSPlus gps;

unsigned long myDate = 112214;
unsigned long myTime = 11223344;
double myLatitude = 38.874860;
double myLongitude = -104.408872;
double myAltitude = 6000;
double mySpeed = 50.2;
double myCourse = 94.2;
unsigned int mySats = 8;
double myHDOP = 1.04;
int myRail = 4095;
int myTemperature = 340;
int myPressure = 3205;
int myHumidity = 1023;
int myGround = 0;

void setup()
{
  //We're using the Due, let's actually use it
  analogReadResolution(12);
  
  //Set up the serial ports - the two USB ports connected to the computer are set for high-
  //speed operation to make things happen as quickly as possible
  Serial.begin(115200);
  Serial1.begin(4800);
  Serial2.begin(115200);
  Serial3.begin(4800);
  SerialUSB.begin(115200);
}

void loop()
{
  if(!DEMO_MODE) sendRealData();
  else sendDemoData();
}

void sendPEDGE()
{
  //Format: $PEDGE,<ID>,<GPSDate>,<GPSTime>,<GPSLat>,<GPSLon>,<GPSAlt>,<GPSSpd>,<GPSCourse>,<GPSNumSats>,<GPSHDOP>,<A1>,<A2>,<A3>,<A4>,<A5>*<CKSUM>
  //Build the primary string out, omitting the '$' and '*' since they're not used for the checksum calcs
  String sentence = "PEDGE,ID"+String(BALLOON_ID)+","+String(myDate)+","+String(myTime)+","+String(myLatitude,6)+","+String(myLongitude,6)+","+String(myAltitude)+","+String(mySpeed)+","+String(myCourse)+","+String(mySats)+","+String(myHDOP)+","+String(myRail)+","+String(myTemperature)+","+String(myPressure)+","+String(myHumidity)+","+String(myGround);
  
  //Calculate the checksum
  int XORVal, i; //variable declaration for the following calculation
  for(XORVal = 0, i = 0; i < sentence.length(); i++)
  {
    int c = (unsigned char)sentence[i];
    XORVal ^= c;
  }
  
  //Put it all together and send it
  if(!DEMO_MODE)
  {
    //send it out the radio serial port
    Serial2.print("$"); //the missing dollar sign
    Serial2.print(sentence);
    Serial2.print("*");
    Serial2.println(XORVal, HEX);
  }
  else
  {
    //send it out the standard USB port
    Serial.print("$"); //the missing dollar sign
    Serial.print(sentence);
    Serial.print("*");
    Serial.println(XORVal, HEX);
  }
}

//Just a demo for initial decoding development
void sendDemoData()
{
  if(GPS_DATA)
  {
    unsigned long startTime = millis(); //get the starting time from the millisecond timer
    while((millis() - startTime) < (REPORT_FREQ*1000)) //while we're waiting for our turn, send GPS data to TinyGPS++
    {
      if(Serial1.available())
      {
        gps.encode(Serial1.read());
      }
    }
    if(gps.location.isValid())
    {
      collectGPSData();
    }
    else
    {
      sendErrorMessage();
    }
  }
  else
  {
    delay(REPORT_FREQ*1000);
  }
  if(ANALOG_DATA) collectAnalogData();
  sendPEDGE();
  delay(REPORT_FREQ*1000);
}

void sendRealData()
{
  unsigned long startTime = millis(); //get the starting time from the millisecond timer
  while((millis() - startTime) < (REPORT_FREQ*1000)) //while we're waiting for our turn, send GPS data to TinyGPS++
  {
    if(Serial1.available())
    {
      gps.encode(Serial1.read());
    }
  }
  if(gps.location.isValid())
  {
    collectGPSData();
    collectAnalogData();
    sendPEDGE();
  }
  else
  {
    sendErrorMessage();
    collectAnalogData();
    sendPEDGE();
  }
}

void collectGPSData()
{
  myDate = gps.date.value();
  myTime = gps.time.value();
  myLatitude = gps.location.lat();
  myLongitude = gps.location.lng();
  myAltitude = gps.altitude.meters();
  mySpeed = gps.speed.knots();
  myCourse = gps.course.deg();
  mySats = gps.satellites.value();
  myHDOP = gps.hdop.value()/100.0;
}

void collectAnalogData()
{
  myRail = analogRead(A7);
  myTemperature = analogRead(A8);
  myPressure = analogRead(A9);
  myHumidity = analogRead(A10);
  myGround = analogRead(A11);
}

void sendErrorMessage()
{
  if(!DEMO_MODE)
  {
    //send it out the radio serial port
    Serial2.println("Something's wrong with the GPS.");
  }
  else
  {
    //send it out the standard USB port
    Serial.println("Something's wrong with the GPS.");
  }
}

*/
