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

Analog inputs aren't used on the ground units.  For now.
*/

#include <TinyGPS++.h>

//This function is required to overcome an error in the arduino core code for the Due.  Kinda sux, but that's how these things go.
void serialEventRun(void) {
  if (Serial.available()) serialEvent(); //if the user asks for something, that comes first
  if (Serial2.available()) serialEvent2(); //the RF data comes next in order of importance
  if (Serial1.available()) serialEvent1(); //then the local GPS
  if (Serial3.available()) serialEvent3(); //then the expansion port
  if (SerialUSB.available()) serialEventUSB(); //then the port that *should* be Tx only
}


TinyGPSPlus LocalGPS; // the parser for the GPS that's - duh - locally attached to the board (Serial1)
TinyGPSPlus RadioGPS; // the parser for the stream of data that's coming in over RF (Serial2)

//$PEDGE,<ID>,<GPSDate>,<GPSTime>,<GPSLat>,<GPSLon>,<GPSAlt>,<GPSSpd>,<GPSCourse>,<GPSNumSats>,<GPSHDOP>,<A1>,<A2>,<A3>,<A4>,<A5>*<CKSUM> (what's coming down from the balloon)
//$PEDGM,<GID>,<GPSDate>,<GPSTime>,<GPSLat>,<GPSLon>,<GPSAlt>,<GPSSpd>,<GPSCourse>,<Message>*<CKSUM> (a user message to be broadcast to other users)
//$PEDGP,<PID>,<BALDate>,<BALTime>,<PredictLat>,<PredictLon> (a predicted landing location message)

//parsing definitions for the balloon sentences
TinyGPSCustom balloonID (RadioGPS, "PEDGE", 1);
TinyGPSCustom balloonDate(RadioGPS, "PEDGE", 2);
TinyGPSCustom balloonTime(RadioGPS, "PEDGE", 3);
TinyGPSCustom balloonLat(RadioGPS, "PEDGE", 4);
TinyGPSCustom balloonLon(RadioGPS, "PEDGE", 5);
TinyGPSCustom balloonAlt(RadioGPS, "PEDGE", 6);
TinyGPSCustom balloonSpd(RadioGPS, "PEDGE", 7);
TinyGPSCustom balloonCse(RadioGPS, "PEDGE", 8);
TinyGPSCustom balloonNumSats(RadioGPS, "PEDGE", 9);
TinyGPSCustom balloonHDOP(RadioGPS, "PEDGE", 10);

//parsing definitions for the message sentences
TinyGPSCustom messageID (RadioGPS, "PEDGM", 1);
TinyGPSCustom messageDate (RadioGPS, "PEDGM", 2);
TinyGPSCustom messageTime (RadioGPS, "PEDGM", 3);
TinyGPSCustom messageLat (RadioGPS, "PEDGM", 4);
TinyGPSCustom messageLon (RadioGPS, "PEDGM", 5);
TinyGPSCustom messageAlt (RadioGPS, "PEDGM", 6);
TinyGPSCustom messageSpd (RadioGPS, "PEDGM", 7);
TinyGPSCustom messageCse (RadioGPS, "PEDGM", 8);
TinyGPSCustom messageTxt (RadioGPS, "PEDGM", 9);

//parsing definitions for the prediciton definitions
TinyGPSCustom predictID (RadioGPS, "PEDGP", 1);
TinyGPSCustom predictDate (RadioGPS, "PEDGP", 2);
TinyGPSCustom predictTime (RadioGPS, "PEDGP", 3);
TinyGPSCustom predictLat (RadioGPS, "PEDGP", 4);
TinyGPSCustom predictLon (RadioGPS, "PEDGP", 5);

#define DEMO_LOCAL_GPS 1 //set to 1 to simulate local GPS data
#define DEMO_RADIO_GPS 1 //set to 1 to simulate remote GPS data
#define GROUND_ID 1 //this needs to be different for each ground system.  DON"T FOGET!
#define CMD_TIMEOUT 5 //how long to stay in command mode before timing out 
#define POS_TIMEOUT 5 //how long to display my position before timing out

#define BALLOON_DATA 1
#define MESSAGE_DATA 2
#define PREDICT_DATA 3
#define IMMEADIATE 0

bool displayMessageMode = true; //Do you want to display a message or not?  
bool commandMode = false;
bool debugMode = true;

unsigned long myDate;
unsigned long myTime;
double myLatitude;
double myLongitude;
double myAltitude;
double mySpeed;
double myCourse;
unsigned int mySats;
double myHDOP;

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
  if(balloonTime.isUpdated())
  {
    if(debugMode) Serial.println("****Received position update from balloon " + String(balloonID.value()));
    outputNMEA(BALLOON_DATA);
  }
  
  if(messageTime.isUpdated())
  {
    if(debugMode) Serial.println("****Received message from ground station " + String(messageID.value()));
    parseAndDisplayMessage();
  }
  
  if(predictTime.isUpdated())
  {
    if(debugMode) Serial.println("****Received new predicted landing point for " + String(predictID.value()));
    outputNMEA(PREDICT_DATA); //really not that great of an idea to have separate functions for the balloon and the
  }
  
  if(commandMode)
  {
    handleCommand();
  }
}

void createUserMessage()
{
  Serial.setTimeout(30000);
  
  Serial.print("Msg>");
  String userMessage = Serial.readStringUntil('\n');
  
  //TODO: Look for commas and astrix in the string and REMOVE THEM!
  
  String sentence = "PEDGM,GID"+String(GROUND_ID)+","+String(LocalGPS.date.value())+","+String(LocalGPS.time.value())+","+String(LocalGPS.location.lat(),6)+","+String(LocalGPS.location.lng(),6)+","+String(LocalGPS.altitude.meters())+","+String(LocalGPS.speed.knots())+","+String(LocalGPS.course.deg())+","+userMessage;
  
  //Calculate the checksum
  int XORVal, i; //variable declaration for the following calculation
  for(XORVal = 0, i = 0; i < sentence.length(); i++)
  {
    int c = (unsigned char)sentence[i];
    XORVal ^= c;
  }
  
  //send it out the radio serial port
  Serial2.print("$"); //the missing dollar sign
  Serial2.print(sentence);
  Serial2.print("*");
  Serial2.println(XORVal, HEX);
  
  if(debugMode)
  {
    Serial.print("\nOutput String: ");
    Serial.print("$"); 
    Serial.print(sentence);
    Serial.print("*");
    Serial.println(XORVal, HEX);
  }
  
}

void displayMyPosition()
{
  if(debugMode) Serial.println("****Piping your position to the Serial3 port.");
  
  unsigned long startTime = millis(); //get the starting time from the millisecond timer
  while((millis() - startTime) < (POS_TIMEOUT*1000))
  {
    if(Serial1.available())
    {
      Serial3.write((char)Serial1.read());
    }
  }
  
  if(debugMode) Serial.println("****Done piping your location to the Serial3 port.");
}

void createPredictMessage()
{
  Serial.setTimeout(5000);
  Serial.print("Idn>");
  String sPredictID = Serial.readStringUntil('\n');
  Serial.print("Lat>");
  String sPredictLat = Serial.readStringUntil('\n');
  Serial.print("Lng>");
  String sPredictLng = Serial.readStringUntil('\n');
  Serial.print("Yay!");
  
  String sentence = "PEDGP,PID"+sPredictID+","+String(LocalGPS.date.value())+","+String(LocalGPS.time.value())+","+String(sPredictLat.toFloat(),6)+","+String(sPredictLng.toFloat(),6);
  
  //Calculate the checksum
  int XORVal, i; //variable declaration for the following calculation
  for(XORVal = 0, i = 0; i < sentence.length(); i++)
  {
    int c = (unsigned char)sentence[i];
    XORVal ^= c;
  }
  
  //send it out the radio serial port
  Serial2.print("$"); //the missing dollar sign
  Serial2.print(sentence);
  Serial2.print("*");
  Serial2.println(XORVal, HEX);
  
  if(debugMode)
  {
    Serial.print("\nOutput String: ");
    Serial.print("$"); 
    Serial.print(sentence);
    Serial.print("*");
    Serial.println(XORVal, HEX);
  }
  
  Serial.setTimeout(1000);
}

//This function is essentially the menu for the whole system
void handleCommand()
{
  unsigned long startTime = millis(); //get the starting time from the millisecond timer
  char cmd = ' '; //initialize the command variable
  
  Serial.print("Cmd>");
  
  while( ((millis() - startTime) < (CMD_TIMEOUT*1000)) && cmd == ' ') //this is the bit that takes care of command mode timeout, and only allows one command to be entered.
  { 
    if(Serial.available())
    {
      cmd = Serial.read();
    }
  }
  
  if(cmd == '?') //the help function
  {
    Serial.println("****M: Send user message.");
    Serial.println("****P: Send predict message.");
    Serial.println("****I: Place me on the map.");
    Serial.println("****C: Send a command.");
    Serial.println("****D: Toggle debug mode.");
  }
  
  else if (cmd == 'M') //the user message generation functiton
  {
    if(debugMode) Serial.println("\n****You want to send a message.");
    createUserMessage();
  }
  
  else if (cmd == 'P') //the predicted landing function
  {
    //if(debugMode) Serial.println("\n****You think you know the future.");
    createPredictMessage();
  }
  
  else if (cmd == 'I') //Handle the request to show my location on the map
  {
    if(debugMode) Serial.println("\n****displaying your location now");
    displayMyPosition();
  }
  
  else if (cmd == 'C') //send a command
  {
    if(debugMode) Serial.println("\n--->You want to be in control.  Not implemented.");
  }
  
  else if (cmd == 'D') //Handle the request to go into debug mode
  {
    if(debugMode)
    {
      Serial.println("\n*****You're turning off debug mode.");
      debugMode = false;
    }
    else
    {
      Serial.println("\n****You're turning on debug mode.");
      debugMode = true;
    }
  }
  
  else if(cmd == ' ') Serial.println("\n--->Command mode time out.");
  else Serial.println("\n--->Unrecognized command.  What are you doing?");
  
  commandMode = false;
}

void collectBalloonData()
{
  myDate = (unsigned long)(String(balloonDate.value()).toFloat());
  myTime = (unsigned long)(String(balloonTime.value()).toFloat());
  myLatitude = abs(String(balloonLat.value()).toFloat());
  myLongitude = abs(String(balloonLon.value()).toFloat());
  myAltitude = String(balloonAlt.value()).toFloat();
  mySpeed = String(balloonSpd.value()).toFloat();
  myCourse = String(balloonCse.value()).toFloat();
  mySats = (unsigned int)(String(balloonNumSats.value()).toInt());
  myHDOP = String(balloonHDOP.value()).toFloat();
}

void collectMessageData()
{
}

void collectPredictData()
{
}


void outputNMEA(int mode)
{
  if(mode==BALLOON_DATA) collectBalloonData();
  else if(mode==MESSAGE_DATA) collectMessageData();
  else if(mode==PREDICT_DATA) collectPredictData();
  
  if(debugMode)
  {
    Serial.println("Date: " + String(myDate));
    Serial.println("Time: " + String(myTime));
    Serial.println("Lat : " + String(myLatitude, 6));
    Serial.println("Lng : " + String(myLongitude, 6));
    Serial.println("Alt : " + String(myAltitude));
    Serial.println("Spd : " + String(mySpeed));
    Serial.println("Cse : " + String(myCourse));
    Serial.println("Sats: " + String(mySats));
    Serial.println("HDOP: " + String(myHDOP));
  }
  
  //Pad date here...
  String sMyDate;
  if(myDate > 99999) sMyDate = String(myDate);
  else sMyDate = "0" + String(myDate); 
  
  //Pad time here...
  String sMyTime;
  myTime = myTime / 100; //remove the trailign DeciSecond zeros from the time
  if(myTime > 99999) sMyTime = String(myTime);
  else if(myTime > 9999) sMyTime = "0" + String(myTime);
  else if(myTime > 999) sMyTime = "00" + String(myTime);
  else if(myTime > 99) sMyTime = "000" + String(myTime);
  else if(myTime > 9) sMyTime = "0000" + String(myTime);
  else if(myTime > 0) sMyTime = "00000" + String(myTime);
  else sMyTime = "000000";
  
  //Pad latitude here... not worring about latitudes less than 10, as we're not flying in Central America
  int myLatDeg;
  float myLatMin;
  String sMyLat;
  myLatDeg = int(myLatitude); //get just the degrees of the latitude
  myLatMin = (myLatitude - myLatDeg) * 60; // calculate minutes from the decimal remainder of the original
  if(myLatMin > 9) sMyLat = String(myLatDeg) + String(myLatMin,3);
  else sMyLat = String(myLatDeg) + "0" + String(myLatMin,3);
  
  //Pad longitude here... only worrying about something less than 100, in case we're east of Lincoln, NE
  int myLngDeg;
  float myLngMin;
  String sMyLngDeg;
  String sMyLngMin;
  String sMyLng;
  myLngDeg = int(myLongitude); 
  myLngMin = (myLongitude - myLngDeg) * 60;
  //Pad the degrees field here
  if(myLngDeg > 99) sMyLngDeg = String(myLngDeg);
  else sMyLngDeg = "0" + String(myLngDeg);
  //Then pad the degrees
  if(myLngMin > 9) sMyLngMin = String(myLngMin,3);
  else sMyLngMin = "0" + String(myLngMin,3);
  //Put it together
  sMyLng = sMyLngDeg + sMyLngMin;
  
  //Pad numSats here
  String sMySats;
  if(mySats > 9) sMySats = String(mySats);
  else sMySats = "0" + String(mySats);
  
  //Pad speed here
  String sMySpeed;
  if(mySpeed > 99) sMySpeed = String(mySpeed, 1);
  else if(mySpeed > 9) sMySpeed = "0" + String(mySpeed, 1);
  else if(mySpeed > 0) sMySpeed = "00" + String(mySpeed, 1);

  //Pad course here
  String sMyCourse;
  if(myCourse > 99) sMyCourse = String(myCourse, 1);
  else if(myCourse > 9) sMyCourse = "0" + String(myCourse, 1);
  else if(myCourse > 0) sMyCourse = "00" + String(myCourse, 1);

  
  //the GGA sentence
  String sentence = "GPGGA,"+sMyTime+","+sMyLat+",N,"+sMyLng+",W,1,"+sMySats+","+String(myHDOP)+","+String(myAltitude)+",M,30.0,M,,";
  
  //Calculate the checksum
  int XORVal, i; //variable declaration for the following calculation
  for(XORVal = 0, i = 0; i < sentence.length(); i++)
  {
    int c = (unsigned char)sentence[i];
    XORVal ^= c;
  }
  
  //send it out the computer port
  Serial3.print("$"); //the missing dollar sign
  Serial3.print(sentence);
  Serial3.print("*");
  Serial3.println(XORVal, HEX);
  
  if(debugMode)
  {
    Serial.print("\nOutput String: ");
    Serial.print("$"); 
    Serial.print(sentence);
    Serial.print("*");
    Serial.println(XORVal, HEX);
  }
  
  //The RMC sentence, hard coded for magnetic variation in CO
  sentence = "GPRMC,"+sMyTime+",A,"+sMyLat+",N,"+sMyLng+",W,"+sMySpeed+","+sMyCourse+","+sMyDate+",008.1,E";
  
  //Calculate the checksum
  XORVal, i; //variable declaration for the following calculation
  for(XORVal = 0, i = 0; i < sentence.length(); i++)
  {
    int c = (unsigned char)sentence[i];
    XORVal ^= c;
  }
  
  //send it out the computer port
  Serial3.print("$"); //the missing dollar sign
  Serial3.print(sentence);
  Serial3.print("*");
  Serial3.println(XORVal, HEX);
  
  if(debugMode)
  {
    Serial.print("\nOutput String: ");
    Serial.print("$"); 
    Serial.print(sentence);
    Serial.print("*");
    Serial.println(XORVal, HEX);
  }
}

void sendMessage()
{
  //send a message encoded in an NMEA wrapper with position info
}

void parseAndDisplayMessage()
{
  //when a message is received, handle pulling out the position info and displaying that on the map, if it's desired.  Also, display the message without the header and with a leading ---> so that it stands out.
}

//To do: parse PEDGE messages, enable placement of my position on the map, enable sending and receiving messages


/////////////////////////////////////////////////////////
//UART Event handlers - these get called at the end of each loop execution if there's data on the UART
/////////////////////////////////////////////////////////

void serialEvent() //Event handler / buffer flusher for the user interface port
{
  char cmd;
  
  //if there's more than one command character on the buffer, we don't really know what to do with it all
  if(!commandMode && Serial.available() > 1)
  {
    if(debugMode) Serial.println("****Something went wrong here - invalid command.  Flushing buffer.");
    while(Serial.available())
    {
      cmd = Serial.read();
    }
    return;
  }
  
  cmd = Serial.read();
  if(cmd == '*')
  {
    commandMode = true;
  }
  else Serial.println("---> Use * to enter command mode, but you knew that already.");
}

void serialEvent1() //Event handler / buffer flusher for the local GPS
{
  while(Serial1.available())
  {
    LocalGPS.encode(Serial1.read());
  }
}

void serialEvent2() //Event handler / buffer flusher for the RF port
{
  while(Serial2.available())
  {
    RadioGPS.encode(Serial2.read());
  }
}

void serialEvent3() //Event handler / buffer flusher for the expansion UART
{
  Serial.println("WTF... there's data on Serial3.");
  while(Serial3.available())
  {
    char cmd = Serial3.read();
  }
}

void serialEventUSB() //Event handler / buffer flusher for the native USB port (should only be output in this application, but we know that Delorme outputs data, so we may as well clear it)
{
  Serial.println("WTF... there's data on SerialUSB.");
  while(SerialUSB.available())
  {
    char cmd = SerialUSB.read();
  }
}




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
