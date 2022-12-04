/*
  jimmyburnworld, October 2022
  SolisModbus-MQTT Bridge using ESP8266 based controller
  Uses ModbusMaster and SoftwareSerial libraries for the RS485/ Modbus side of things
  ESP8266WiFi and ArduinoMqttCLient for the MQTT side
  https://github.com/jimmyburnworld/SolisModbus-MQTT
  Uses Solis inverter 'COM' port. Set inverter 'address' to 2 - check communications to BMS and Meter after doing this before proceeding
  to connect Arduino.
  COM port: 1 = +5Vcc, 2 = 0V com, 3 = 'A' 4 = 'B' - This might vary by model, your responsibility to check before connecting
  MAX485 GND needed to be connected to 'G' next to 3.3V pin to work.
  Used inverter +5V and 0V to power ESP8266 using 'Vin' and 'G' connections & jumpered to MAX485 Vcc & Gnd
  KNOWN ISSUES: Signed 16bit values and 32bit values do not work correctly.
  Thee signed 16bit values are unsigned because the ModbusMaster library only returns uint16_t type.
  32bit values are due to word/ byte swapping Endian-ness etc.
  Only included the variables for this project, feel free to add more or remove
  Search for RS485_MODBUS-Hybrid-BACoghlan-201811228-1854.pdf for protocol description and registers
*/

#include <ModbusMaster.h>
#include <ESP8266WiFi.h>
#include <ArduinoMqttClient.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

// *************************************************************************************

// WIFI settings
#define WIFI_SSID ""    // YOUR SSID goes here
#define WIFI_PASSWORD ""   // YOUR WIFI PASSWORD goes here

// MQTT Broker settings. Will work with IP or local name. Static IP is more reliable
#define MQTT_BROKER_URL  "beaglebone" //"xxx.xxx.xxx.xxx"; //"core-mosquitto";
#define MQTT_BROKER_PORT 1883         //your port here
#define MQTT_TOPIC "Solis/inverter"
#define MQTT_CLIENT_ID "SolisInv"
#define MQTT_USERNAME "your user"
#define MQTT_PASSWORD "your password"

// Pin definitions
#define LED_GR 15     // Green LED Pin
#define LED_RT 16     // Red LED Pin
#define MAX485_RX 12  // connect RX to D6 (ESP2866 D6), TX to D7 (ESP2866 D7)
#define MAX485_TX 13
#define MAX485_DE 14  // Use a MAX485 compatible transceiver, bridge DE and RE_NEG and connect to D5 on ESP8266

// *************************************************************************************

// receivePin, transmitPin, inverse_logic, bufSize, isrBufSize
SoftwareSerial S(MAX485_RX, MAX485_TX);

// instantiate ModbusMaster object
ModbusMaster node;

//*MQTT Initialisation here
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

// Callbacks for setting the MAX485 to send or receive
void preTransmission()
{
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_DE, 0);
}

const char broker[] = MQTT_BROKER_URL; 
int        port     = MQTT_BROKER_PORT; 
int count = 0;

#define REGISTERS_TO_READ 9
// Reg 3005 - Active power U32 - TBD: My inverter output power is not high enough to test if this value is read correctly
// Reg 3009 - Total energy U32 - TBD: Could not be tested
// Reg 3015 - Energy yesterday U16 - Manual says 3015 is Energy today
// Reg 3016 - Energy today U16 - Manual says 3015 is Energy yesterday
// Reg 3011 - Energy this month U32 - TBD: Could not be tested
// Reg 3013 - Energy last month U32 - TBD: Could not be tested
// Reg 3017 - Energy this year U32
// Reg 3019 - Energy last year U32
// Reg 3042 - Inverter temp U16

const long RegAddress[REGISTERS_TO_READ] = {3005, 3009, 3016, 3015, 3011, 3013, 3017, 3019, 3042};
const int  RegWords[REGISTERS_TO_READ]   = {   2,    2,    1,    1,    1,    1,    1,    1,    1};
const float RegScalar[REGISTERS_TO_READ] = { 1.0,  1.0,  0.1,  0.1,  1.0,  1.0,  1.0,  1.0,0.001};
float RegValue[REGISTERS_TO_READ];
uint8_t RegResult[REGISTERS_TO_READ];

// Initialise character arrays with JSON names
const char *RegName[REGISTERS_TO_READ] = 
                  {"PACTot",
                   "ETotal",
                   "EToday",
                   "EYesterday",
                   "EMonth",
                   "ELastMonth",
                   "EYear",
                   "ELastYear",
                   "InvTemperature",
                  };

void setup()
{
  pinMode(LED_GR, OUTPUT);
  pinMode(LED_RT, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(LED_GR, 0);    // LED off
  digitalWrite(LED_RT, 0);    // LED off
  digitalWrite(MAX485_DE, 0); // Init in receive mode

  // Modbus communication runs at 9600 baud
  S.begin(9600, SWSERIAL_8N1);

  //Serial Monitor
  Serial.begin(57600);

  // Modbus slave ID 2
  node.begin(2, S); 
  
  // Set callbacks for setting R/W direction on MAX485
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  //WiFi Connection here
  {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.print("Connecting");
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
      digitalWrite(LED_GR, millis() / 1000 % 2);
    }
    Serial.println();

    digitalWrite(LED_RT, 0);
    digitalWrite(LED_GR, 1);

    Serial.print("Connected, IP address: ");
    Serial.println(WiFi.localIP());
  }

  //MQTT Connection here

  // Each client must have a unique client ID
  mqttClient.setId(MQTT_CLIENT_ID);

  // You can provide a username and password for authentication
  mqttClient.setUsernamePassword(MQTT_USERNAME, MQTT_PASSWORD);
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    digitalWrite(LED_RT, 1);
    // FIXME: This is not ideal as it will prevent startup if the MQTT broker becomes active after the inverter
    // Since the inverter will shut down on sunset we will eventually establish a connection the next day
    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

}

void loop()
{
  int RegNo;

  // Update Wifi status indication 
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED_GR, 1);
  } else {
    digitalWrite(LED_GR, 0);
  }

  //*MQTT Loop
  // call poll() regularly to allow the library to send MQTT keep alives which
  // avoids being disconnected by the broker
  mqttClient.poll();

  // Read inverter registers
  bool error = false;
  for (RegNo = 0; RegNo < REGISTERS_TO_READ; RegNo++) {
    
    RegResult[RegNo] = node.readInputRegisters(RegAddress[RegNo], 10);  // We some extra register as otherwise the actual value is reported in RegResult[RegNo]
    if (RegResult[RegNo] == node.ku8MBSuccess) {
      if (RegWords[RegNo] == 1) {
        RegValue[RegNo] = (node.getResponseBuffer(0) * RegScalar[RegNo]);
      }
      if (RegWords[RegNo] == 2) {
        RegValue[RegNo] = (((node.getResponseBuffer(1) << 16) + node.getResponseBuffer(0)  ) * RegScalar[RegNo]);
      }

      yield();
      delay(500); // Frame interval is 300ms as per Ginlong docu. I found it more stable with 500ms
    } else {
      error = true;
      digitalWrite(LED_RT, 1);
      Serial.println("Error reading inverter registers");     

      StaticJsonDocument<200> jsonDoc;
      jsonDoc["Register"] = RegNo;
      jsonDoc["ErrorCode"] = RegValue[RegNo];
      mqttClient.beginMessage(MQTT_TOPIC);
      serializeJson(jsonDoc, Serial);
      serializeJson(jsonDoc, mqttClient);
      mqttClient.endMessage();
 
    }
  }
  
  Serial.println("Mqtt check");
  // Check error status and tranmit MQTT message
  if (!error) {
    Serial.println("MQTT transmission:");      

    // Create JSON
    StaticJsonDocument<200> jsonDoc;
    for (RegNo = 0; RegNo < REGISTERS_TO_READ; RegNo++)
    {
      jsonDoc[RegName[RegNo]] = RegValue[RegNo];
    }

    // Send JSON as MQTT an on the serial connection
    mqttClient.beginMessage(MQTT_TOPIC);
    serializeJson(jsonDoc, Serial);
    serializeJson(jsonDoc, mqttClient);
    mqttClient.endMessage();

    // indicate things are o.k.
    digitalWrite(LED_RT, 0);
  }

}
