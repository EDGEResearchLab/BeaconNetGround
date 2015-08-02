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
SerialUSB: Future expansion - note that it appears that the SerialUSB goes away when Serial is connected... 
Serial1: GPS (Rx only)
Serial2: 9xTend (Tx/Rx) and OpenLog (Tx only)
Serial3: Header NMEA output - intended to be Tx only

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

#define GROUND_ID 1 //this needs to be different for each ground system.  DON"T FOGET!
#define CMD_TIMEOUT 5 //how long to stay in command mode before timing out 
#define POS_TIMEOUT 5 //how long to display my position before timing out

//the constants used for switching the source of the re-constituted GPS data
#define BALLOON_DATA 1
#define MESSAGE_DATA 2
#define PREDICT_DATA 3
#define IMMEADIATE 0

#define KEEPALIVE_PERIOD 5 //number of seconds to wait before outputting a duplicate NMEA point for the balloon

bool displayMessageMode = true; //Do you want to display a message or not?  
bool commandMode = false;
bool debugMode = true;
bool keepaliveMode = true;

unsigned long myDate;
unsigned long myTime;
double myLatitude;
double myLongitude;
double myAltitude;
double mySpeed;
double myCourse;
unsigned int mySats;
double myHDOP;
String myID;

unsigned long mainTime;

void setup()
{ 
  //Set up the serial ports - the two USB ports connected to the computer are set for high-
  //speed operation to make things happen as quickly as possible
  Serial.begin(115200);
  Serial1.begin(4800);
  Serial2.begin(115200);
  Serial3.begin(115200);
  SerialUSB.begin(115200);
  
  myID = "GID"+String(GROUND_ID);
  
  mainTime = millis(); //capture the current reading of the millis timer for keepalive function
}

void loop()
{
  if(balloonTime.isUpdated())
  {
    if(debugMode) Serial.println("****Received position update from balloon " + String(balloonID.value()));
    outputNMEA(BALLOON_DATA);
    mainTime = millis();  //"reset" the timer for keepalive output
  }
  
  if(messageTime.isUpdated())
  {
    if(debugMode) Serial.println("****Received message from ground station " + String(messageID.value()));
    parseAndDisplayMessage();
    mainTime = millis(); //"reset" the timer for keepalive output
  }
  
  if(predictTime.isUpdated())
  {
    if(debugMode) Serial.println("****Received new predicted landing point for " + String(predictID.value()));
    outputNMEA(PREDICT_DATA, 5); 
    mainTime = millis(); //"reset" the timer for keepalive output
  }
  
  if(commandMode)
  {
    handleCommand();
    mainTime = millis(); //"reset" the timer for keepalive output
  }
  
  if((millis() - mainTime) > KEEPALIVE_PERIOD * 1000)  //if this happens, you haven't heard from the balloon in a while...
  {
    if(keepaliveMode)
    {
      double elapsedTime = (millis() - mainTime)/1000;
      if(debugMode) Serial.println("****KEEPALIVE MESSAGE GENERATED - NO UPDATE FROM BALLOON IN " + String(elapsedTime) + " SECONDS.");
      outputNMEA(BALLOON_DATA);
      mainTime = millis(); //"reset" the timer for keepalive output
    }
  }
}

void createUserMessage()
{
  Serial.setTimeout(30000);
  
  Serial.print("Msg>");
  String userMessage = Serial.readStringUntil('\r');
  
  //TODO: TEST THE FOLLOWING CODE (it would be better to just remove the offending characters, but we'll stick with an error for now):
  if((userMessage.indexOf('*') > 0) | (userMessage.indexOf(',') > 0))
  {
    Serial.print("\n ERROR: Strings must not contain commas or asterisks.  Try again.");
    return;
  }
  
  String sentence = "PEDGM,"+myID+","+String(LocalGPS.date.value())+","+String(LocalGPS.time.value())+","+String(LocalGPS.location.lat(),6)+","+String(LocalGPS.location.lng(),6)+","+String(LocalGPS.altitude.meters())+","+String(LocalGPS.speed.knots())+","+String(LocalGPS.course.deg())+","+userMessage;
  
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
  String sPredictID = Serial.readStringUntil('\r');
  Serial.print("Lat>");
  String sPredictLat = Serial.readStringUntil('\r');
  Serial.print("Lng>");
  String sPredictLng = Serial.readStringUntil('\r');
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
    Serial.println("\n****M: Send user message.");
    Serial.println("****P: Send predict message.");
    Serial.println("****I: Place me on the map.");
    Serial.println("****C: Send a command. (not supported)");
    Serial.println("****D: Toggle debug mode.");
    Serial.println("****K: Toggle keepalive mode.");
    Serial.println("****G: Change your station ID (beta)");
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
  
  else if (cmd == 'K') //Handle the request to toggle keepalive mode
  {
    if(keepaliveMode)
    {
      if(debugMode) Serial.println("\n*****You're turning off keepalive mode.");
      keepaliveMode = false;
    }
    else
    {
      if(debugMode) Serial.println("\n****You're turning on keepalive mode.");
      keepaliveMode = true;
    }
  }
  
  else if (cmd == 'G')
  {
    if(debugMode) Serial.println("\n****Changing station ID.  Current ID is: " + myID);
    changeStationID();
  }
  
  else if(cmd == ' ') Serial.println("\n--->Command mode time out.");
  else Serial.println("\n--->Unrecognized command.  What are you doing?");
  
  commandMode = false;
}

void changeStationID()
{
  Serial.setTimeout(10000);
  Serial.println("\nThis feature is in beta; there isn't any error checking.  Be careful!");
  Serial.print("Enter new station ID (or wait 10 seconds to set to default)> ");
  
  String newID = Serial.readStringUntil('\r');
  
  if(!newID.length()) 
  {
    Serial.println("\nID set to default.  ID is: " + myID);
    return;
  }
  
  myID = newID;
  
  Serial.println("\n ID Updated to: " + myID);
  
  Serial.setTimeout(1000);
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
  
  //hardcode these values, as they're not provided for messages...
  mySats = 9;
  myHDOP = 1.0;
}

void collectMessageData()
{
  myDate = (unsigned long)(String(messageDate.value()).toFloat());
  myTime = (unsigned long)(String(messageTime.value()).toFloat());
  myLatitude = abs(String(messageLat.value()).toFloat());
  myLongitude = abs(String(messageLon.value()).toFloat());
  myAltitude = String(messageAlt.value()).toFloat();
  mySpeed = String(messageSpd.value()).toFloat();
  myCourse = String(messageCse.value()).toFloat();
  mySats = (unsigned int)(String(balloonNumSats.value()).toInt());
  myHDOP = String(balloonHDOP.value()).toFloat();
}

void collectPredictData()
{
  myDate = (unsigned long)(String(predictDate.value()).toFloat());
  myTime = (unsigned long)(String(predictTime.value()).toFloat());
  myLatitude = abs(String(predictLat.value()).toFloat());
  myLongitude = abs(String(predictLon.value()).toFloat());
  
  //hardcode these values, as they're not provided for prerdicted landing sites...
  myAltitude = 1800.0;
  mySpeed = 0.0;
  myCourse = 0.0;
  mySats = 9;
  myHDOP = 1.0;
}

void outputNMEA(int mode, int repeats)
{
  for(int i=0; i<repeats; i++)
  {
    outputNMEA(mode);
  }
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

void parseAndDisplayMessage()
{
  Serial.println("\n----" + String(messageID.value()) + "> " + String(messageTxt.value()));
  
  if(debugMode)
  {
    Serial.println("Message received with the following position information included: ");
  }
  
  outputNMEA(MESSAGE_DATA, 5); 
}


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

