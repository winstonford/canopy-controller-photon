/////////////////////////////////////////////////////////////////////////
//
// Canopy Controller
//
// Version 1.0
// Emmanuel Avionics, Inc. Los Angeles Jan 10, 2017
//
// Files needed:
// x.ino
// lib1.cpp
// lib1.h
// Compiles on the web based IDE at https://build.particle.io
// Runs on the Particle Photon board
//
////////////////////////////////////////////////////////////////////////////
//
#include "Particle.h"
#include "MQTT.h"

#include "lib1.h"

SDL_Arduino_INA3221 ina3221;

// If defined, output directly to the serial port
#define DEBUG_SERIAL

// Time between samples in miliseconds
#define SAMPLETIME 1000

// INA3221 channels
#define CHAN1 1
#define CHAN2 2
#define CHAN3 3

// INA3221 boards (each board is hardware modified to define its channel)
#define BOARD0 0
#define BOARD1 1
#define BOARD2 2

// ADC channels for temperature measurement
#define ADC0 0
#define ADC1 1
#define ADC2 2

// INA3221 boards are numbered 0 thru 2
uint8_t board = 2;

// Loopcount is displayed so display always changes between frames
uint16_t loopcount;

// Constant to convert ADC counts to degrees Kelvin
const float ktemp = 100.0 * ((20.0 + 5.62) / 20.0) * (3.3 / 4095.0);


// replace these values with your own, you can get them from cayenne's website'
const String CLIENTID = "GET_IT_FROM_CAYENNE_WHEN_YOU_ADD_A_DEVICE";
const String USERNAME = "GET_IT_FROM_CAYENNE_WHEN_YOU_ADD_A_DEVICE";
const String PASSWORD = "GET_IT_FROM_CAYENNE_WHEN_YOU_ADD_A_DEVICE";

// for sending values to cayenne, one must send a publish to a topic+channel
// Source: See section "Send Sensor data" on https://mydevices.com/cayenne/docs/#bring-your-own-thing-api-mqtt-messaging-topics
// Example: v1/username/things/clientID/data/channel
// My example: "v1/" + USERNAME + "/things/" + CLIENTID + "/data/2"
// I will leave the channel (the 2 above) out so it can be configured by the code using this constant
const String cayenneSendSensorData = "v1/" + USERNAME + "/things/" + CLIENTID + "/data/";

// for receiving commands from cayenne, one must do the following:
// Source: See section "Receive Actuator command" on https://mydevices.com/cayenne/docs/#bring-your-own-thing-api-mqtt-messaging-topics
// 1- Receive Actuator command
// 2- Send Actuator Updated Value
// 3- Send command response
// 
// this translates into:
// 1- Receive Actuator command:
//    1.a -> subscribe to topic cmd+channel
//    1.b -> expect a publish from cayenne into it
//    1.c -> parse command sequence number
//
// Example from cayenne site on step 1:
// Receive Actuator Command on Channel 3
// => SUB v1/A1234B5678C/things/0123-4567-89AB-CDEF/cmd/3
// <= 2otoExGxnMJz0Jn,0
//
// 2- Send Actuator Updated Value
//    2.a -> publish current value of actuator to topic/channel
// Example from cayenne site on step 2:
// Send updated Actuator value on Channel 3
// => PUB v1/A1234B5678C/things/0123-4567-89AB-CDEF/digital/3
// 1
//
// 3- Send command response -> publish command response to topic response with result
// Example from cayenne site on step 3:
// Send a Command Response - OK
// => PUB v1/A1234B5678C/things/0123-4567-89AB-CDEF/response
// ok,2otoExGxnMJz0Jn
//

const String cayenneReceiveActuatorCommand = "v1/" + USERNAME + "/things/" + CLIENTID + "/cmd/";
const String cayenneSendCommandResponse = "v1/" + USERNAME + "/things/" + CLIENTID + "/response";

// example value
int value10 = 0;

void mqttCallback(char* topic, byte* payload, unsigned int length);

// ip of mqtt.mydevices.com
byte mqttServer[] = { 54, 87, 58, 187 };
MQTT mqttClient(mqttServer, 1883, mqttCallback);


void setup(void)
{
  Serial.begin(57600);
  Serial.println("SDA_Arduino_INA3221_Test");

  Serial.println("Measuring voltage and current with ina3221 ...");
// Initialize registers on each board
  ina3221.begin(0);
  ina3221.begin(1);
  ina3221.begin(2);
// Initialize FET drivers
  pinMode (D2, OUTPUT);
  pinMode (D3, OUTPUT);
  pinMode (D4, OUTPUT);
  pinMode (D5, OUTPUT);
  pinMode (D6, OUTPUT);

  mqttConnect();
  
// Set up internet switch control function
  Particle.function("switches",switchToggle);
}

void loop(void)
{

#ifdef DEBUG_SERIAL
  Serial.print(loopcount++); Serial.println("------------------------------");
#endif
  // The boards are numbered 0 thru 3, the channels on each board 1 thru 3
  // Voltage V is in volts, current I is in milliamps
  float V1 = 0;
  float V2 = 0;
  float V3 = 0;
  float V4 = 0;
  float V5 = 0;
  float V6 = 0;
  float V7 = 0;
  float V8 = 0;
  float V9 = 0;
  float I1 = 0;
  float I2 = 0;
  float I3 = 0;
  float I4 = 0;
  float I5 = 0;
  float I6 = 0;
  float I7 = 0;
  float I8 = 0;
  float I9 = 0;
  float T1 = 0;
  float T2 = 0;
  float T3 = 0;
  // String for holding the data output ready to publish
  String outstring;

  int i;

  // Turn on power Zener Diode circuit
  digitalWrite(D6, HIGH);
  // Allow current to stabilize
  delay(50);

  // Get voltages
  V1 = ina3221.getBusVoltage_V(BOARD0, CHAN1);
  V2 = ina3221.getBusVoltage_V(BOARD0, CHAN2);
  V3 = ina3221.getBusVoltage_V(BOARD0, CHAN3);
  V4 = ina3221.getBusVoltage_V(BOARD1, CHAN1);
  V5 = ina3221.getBusVoltage_V(BOARD1, CHAN2);
  V6 = ina3221.getBusVoltage_V(BOARD1, CHAN3);
  V7 = ina3221.getBusVoltage_V(BOARD2, CHAN1);
  V8 = ina3221.getBusVoltage_V(BOARD2, CHAN2);
  V9 = ina3221.getBusVoltage_V(BOARD2, CHAN3);

  // Get currents in mA and round to nearest integer
  I1 = round(ina3221.getCurrent_mA(BOARD0, CHAN1));
  I2 = round(ina3221.getCurrent_mA(BOARD0, CHAN2));
  I3 = round(ina3221.getCurrent_mA(BOARD0, CHAN3));
  I4 = round(ina3221.getCurrent_mA(BOARD1, CHAN1));
  I5 = round(ina3221.getCurrent_mA(BOARD1, CHAN2));
  I6 = round(ina3221.getCurrent_mA(BOARD1, CHAN3));
  I7 = round(ina3221.getCurrent_mA(BOARD2, CHAN1));
  I8 = round(ina3221.getCurrent_mA(BOARD2, CHAN2));
  I9 = round(ina3221.getCurrent_mA(BOARD2, CHAN3));

  // Get temperatures - 273.15 for C, × 9/5 - 459.67 for F
  T1 = (float)analogRead(ADC0) * ktemp * 9/5 - 459.67; 
  T2 = (float)analogRead(ADC1) * ktemp * 9/5 - 459.67;
  T3 = (float)analogRead(ADC2) * ktemp * 9/5 - 459.67;

  // Reset Zener line ready for next time
  digitalWrite(D6, LOW);

  // Prepare string for publishing.  Vars temp and voltage to 1 decimal place, current as an integer
  /*
  outstring = "";
  outstring = outstring + String(T1, 1) + ", ";
  outstring = outstring + String(T2, 1) + ", ";
  outstring = outstring + String(T3, 1) + ", ";
  outstring = outstring + String(V1, 1) + ", ";
  outstring = outstring + String(I1, 0) + ", ";
  outstring = outstring + String(V2, 1) + ", ";
  outstring = outstring + String(I2, 0) + ", ";
  outstring = outstring + String(V3, 1) + ", ";
  outstring = outstring + String(I3, 0) + ", ";
  outstring = outstring + String(V4, 1) + ", ";
  outstring = outstring + String(I4, 0) + ", ";
  outstring = outstring + String(V5, 1) + ", ";
  outstring = outstring + String(I5, 0) + ", ";
  outstring = outstring + String(V6, 1) + ", ";
  outstring = outstring + String(I6, 0) + ", ";
  outstring = outstring + String(V7, 1) + ", ";
  outstring = outstring + String(I7, 0) + ", ";
  outstring = outstring + String(V8, 1) + ", ";
  outstring = outstring + String(I8, 0) + ", ";
  outstring = outstring + String(V9, 1) + ", ";
  outstring = outstring + String(I9, 0) ;
*/


/* Test values 

    I1 = 1234;
    V1 = 18.7;
    I2 = 1234.9;
    V2 = 18.7;
    I3 = 1234;
    V3 = 18.7;
    I4 = 1234;
    V4 = 18.7;
    I5 = 1234;
    V5 = 18.7;
    I6 = 1234;
    V6 = 18.7;
    I7 = 1234;
    V7 = 18.7;
    I8 = 1234;
    V8 = 18.7;
    I9 = 1234;
    V9 = 18.7;
 */   

// Subject to Particle limit of 255 bytes / chars
// Tested Max per value is 5 digits: 3 digits, point, one decimal (example 123.4) 
// Settings: 
// Temps (in F)and Volts are set to one decimal place, which in practice, whether we use C or F, is 4 digits: 78.4
// Current (in milliamps) is set to zero decimal points which in practice is also 4 digits: 1597 Interesting that we save a char by mA, in amps is 5 digits 1.597

//    "\",\"c\":\"" + String("3") +  ->> c is NOT SUPPORTED BY GOOGLE SCRIPTS ARRGGGGG


//COMMENTED SINCE loop is executed every second. I suggest this publish to get its own function and timer ;)
  // Particle.publish("pg", 
  //   "{\"a\":\"" + String(T1, 1) + 
  //   "\",\"b\":\"" + String(T2, 1) + 
  //   "\",\"d\":\"" + String(T3, 1) + 
  //   "\",\"e\":\"" + String(V1, 1) + 
  //   "\",\"f\":\"" + String(I1, 0) + 
  //   "\",\"g\":\"" + String(V2, 1) + 
  //   "\",\"h\":\"" + String(I2, 0) + 
  //   "\",\"i\":\"" + String(V3, 1) + 
  //   "\",\"j\":\"" + String(I3, 0) + 
  //   "\",\"k\":\"" + String(V4, 1) + 
  //   "\",\"l\":\"" + String(I4, 0) + 
  //   "\",\"m\":\"" + String(V5, 1) + 
  //   "\",\"n\":\"" + String(I5, 0) + 
  //   "\",\"o\":\"" + String(V6, 1) + 
  //   "\",\"p\":\"" + String(I6, 0) + 
  //   "\",\"q\":\"" + String(V7, 1) + 
  //   "\",\"r\":\"" + String(I7, 0) + 
  //   "\",\"s\":\"" + String(V8, 1) + 
  //   "\",\"t\":\"" + String(I8, 0) + 
  //   "\",\"u\":\"" + String(V9, 1) + 
  //   "\",\"v\":\"" + String(I9, 0) + 
  //   "\"}",
  //   60, PRIVATE);
// delay(120000);


#ifdef DEBUG_SERIAL
  // If DEBUG is defined, output everything to the serial port
  Serial.println(outstring);
  Serial.print("V1 = "); Serial.print(V1); Serial.print(" I1 = "); Serial.println(I1);
  Serial.print("V2 = "); Serial.print(V2); Serial.print(" I2 = "); Serial.println(I2);
  Serial.print("V3 = "); Serial.print(V3); Serial.print(" I3 = "); Serial.println(I3);
  Serial.print("V4 = "); Serial.print(V4); Serial.print(" I4 = "); Serial.println(I4);
  Serial.print("V5 = "); Serial.print(V5); Serial.print(" I5 = "); Serial.println(I5);
  Serial.print("V6 = "); Serial.print(V6); Serial.print(" I6 = "); Serial.println(I6);
  Serial.print("V7 = "); Serial.print(V7); Serial.print(" I7 = "); Serial.println(I7);
  Serial.print("V8 = "); Serial.print(V8); Serial.print(" I8 = "); Serial.println(I8);
  Serial.print("V9 = "); Serial.print(V9); Serial.print(" I9 = "); Serial.println(I9);
  Serial.print(" T1 = "); Serial.println(T1);
  Serial.print(" T2 = "); Serial.println(T2);
  Serial.print(" T3 = "); Serial.println(T3);
#endif

  // Delay before next sample
#ifdef SAMPLETIME
  for(i = 0; i < 1; i++)
    delay(SAMPLETIME);
#endif

  mqttLoop();
  //this writes to a widget on channel 10
  mqttPublish(cayenneSendSensorData + "10", String(value10));
  value10 = value10 +1;
  delay(1000);

}

int switchToggle(String command) {
  if (command=="S1ON") {
    digitalWrite(D2,HIGH);
#ifdef DEBUG_SERIAL
    Serial.println("S1 set on");
#endif
    return 1;
  }
  else if (command=="S2ON") {
    digitalWrite(D3,HIGH);
#ifdef DEBUG_SERIAL
    Serial.println("S2 set on");
#endif
    return 2;
  }
  else if (command=="S3ON") {
    digitalWrite(D4,HIGH);
#ifdef DEBUG_SERIAL
    Serial.println("S3 set on");
#endif
    return 3;
  }
  else if (command=="S4ON") {
    digitalWrite(D5,HIGH);
#ifdef DEBUG_SERIAL
    Serial.println("S4 set on");
#endif
    return 4;
  }
  else if (command=="SOFF") {
    digitalWrite(D2,LOW);
    digitalWrite(D3,LOW);
    digitalWrite(D4,LOW);
    digitalWrite(D5,LOW);
#ifdef DEBUG_SERIAL
    Serial.println("All switches off");
#endif
    return 5;
  }
  else {
    return -1;
  }
}

/*******************************************************************************
 * Function Name  : mqttConnect
 * Description    : connects to the mqtt broker and subscribes to interesting topics
 * Return         : true if success, false otherwise
 *******************************************************************************/
bool mqttConnect()
{
  if (mqttClient.isConnected())
    return true;
  
  mqttClient.connect( CLIENTID, USERNAME, PASSWORD);

  if (mqttClient.isConnected()) {
    Particle.publish("MQTT connected", "Hurray!", PRIVATE);
  } else {
    Particle.publish("MQTT failed to connect", "", PRIVATE);
    return false;
  }

  // we need to subscribe to topics that this device is interested in  
  mqttSubscribeToInterestingTopics();

  return true;

}

// Cayenne: Receive Actuator command
// In order to receive a command for a given data channel, the device must subscribe to the “cmd” topic.
// Topic	                               PUB  SUB
// v1/username/things/clientID/cmd/channel	 	X    Payload will contain a command sequence number followed by the value.
//                                                    The Developer is responsible for managing the value format.
// (string) seq,value
// Source: https://mydevices.com/cayenne/docs/#bring-your-own-thing-api-mqtt-messaging-topics

/*******************************************************************************
 * Function Name  : mqttSubscribeToInterestingTopics
 * Description    : subscribes to interesting topics
 * Return         : none
 *******************************************************************************/
void mqttSubscribeToInterestingTopics()
{
  mqttClient.subscribe(cayenneReceiveActuatorCommand + "1");
  mqttClient.subscribe(cayenneReceiveActuatorCommand + "2");
}

/*******************************************************************************
 * Function Name  : mqttLoop
 * Description    : processes the mqtt events
 * Return         : true if success, false otherwise
 *******************************************************************************/
bool mqttLoop()
{
  if (mqttClient.isConnected()) {
    mqttClient.loop();
    return true;
  }

  return false;
}

/*******************************************************************************
 * Function Name  : mqttPublish
 * Description    : publishes the payload to the topic
 * Parameters     : String topic
                    String payload
 * Return         : true if success, false otherwise
 *******************************************************************************/
bool mqttPublish(String topic, String payload)
{
  if (mqttClient.isConnected()) {
    mqttClient.publish(topic, payload);
    return true;
  }

  return false;
}

/*******************************************************************************
 * Function Name  : mqttCallback
 * Description    : receives publishes sent by the mqtt broker
 * Parameters     : char* topic
                    byte* payload
                    unsigned int length
 * Return         : none
 *******************************************************************************/
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  String payloadStr(p);
  String topicStr(topic);

  // of course there would be much better ways of achieving this, but for now this can do just fine
  String channel = "";
  if (topicStr == cayenneReceiveActuatorCommand+"1") {
    channel = "1";
  } else if (topicStr == cayenneReceiveActuatorCommand+"2") {
    channel = "2";
  }

  if (channel == "") {
    Particle.publish( "ERROR", "unknown MQTT command received", PRIVATE);
    return;
  }

  Particle.publish( "MQTT command received", channel + " " + payloadStr, PRIVATE);

  // parse payload to extract the value to be set
  // Example: 2otoExGxnMJz0Jn,0
  // and we want 0
  int commaPosition = payloadStr.indexOf(",");
  String commandValue = payloadStr.substring(commaPosition+1);
  Particle.publish( "DEBUG value", commandValue, PRIVATE);
  
  // parse payload to extract the command sequence number
  // Example: 2otoExGxnMJz0Jn,0
  // and we want 2otoExGxnMJz0Jn
  String commandSequenceNumber = payloadStr.remove(commaPosition);
  Particle.publish( "DEBUG seqNumber", commandSequenceNumber, PRIVATE);


  // this executes step 2 of cayenne's handshake:
  // Send updated Actuator value on Channel 3
  // => PUB v1/A1234B5678C/things/0123-4567-89AB-CDEF/digital/3   <<-- I believe digital is a mistake in the docs
  // 1
  mqttPublish(cayenneSendSensorData + channel, commandValue);

 
  // this executes step 3 of cayenne's handshake:
  // Send a Command Response - OK
  // => PUB v1/A1234B5678C/things/0123-4567-89AB-CDEF/response
  // ok,2otoExGxnMJz0Jn
  mqttPublish(cayenneSendCommandResponse, "ok," + commandSequenceNumber);


  //**********************************************************************************************
  //**********************************************************************************************
  //**********************************************************************************************
  // IT IS HERE that you call the function that will set your relays on/off
  // channel is the relay number and commandValue contains 1 or 0 for on or off
  //**********************************************************************************************
  //**********************************************************************************************
  //**********************************************************************************************

  if (channel == "1") {
      if (commandValue=="0") {
        digitalWrite(D2,LOW);
      } else {
        digitalWrite(D2,HIGH);
      }
  }

  if (channel == "2") {
      if (commandValue=="0") {
        digitalWrite(D3,LOW);
      } else {
        digitalWrite(D3,HIGH);
      }
  }

}
// this is the end compadre, que mas? ;)
