
#include <ModbusMaster.h>
#include <ESP8266WiFi.h>
#include <ArduinoMqttClient.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

// receivePin, transmitPin, inverse_logic, bufSize, isrBufSize
// connect RX to D6 (ESP2866 D6), TX to D7 (ESP2866 D7)
SoftwareSerial S(12, 13);

#define LED_GR D8
#define LED_RT D0

//IP Address for MQTT Broker here. Will work with local name however more reliable with static IP
#define MQTT_BROKER_URL  "beaglebone" //"xxx.xxx.xxx.xxx"; //"core-mosquitto";
#define MQTT_BROKER_PORT 1883         //your port here
#define MQTT_TOPIC "Solis/inverter"

/*!
  Use a MAX485 compatible transceiver, bridge DE and RE_NEG and connect to D5 on ESP8266
*/
#define MAX485_DE      14

// instantiate ModbusMaster object
ModbusMaster node;

void preTransmission()
{
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_DE, 0);
}

//*MQTT Initialisation here
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);


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

long RegAddress[REGISTERS_TO_READ] = {3005, 3009, 3016, 3015, 3011, 3013, 3017, 3019, 3042};
int  RegWords[REGISTERS_TO_READ]   = {   2,    2,    1,    1,    1,    1,    1,    1,    1};
float RegScalar[REGISTERS_TO_READ] = { 1.0,  1.0,  0.1,  0.1,  1.0,  1.0,  1.0,  1.0,0.001};
float RegValue[REGISTERS_TO_READ];
uint8_t RegResult[REGISTERS_TO_READ];

// Initialise character arrays with JSON names
char *RegName[REGISTERS_TO_READ] = 
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
  digitalWrite(LED_GR, 0);
  digitalWrite(LED_RT, 0);

  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_DE, 0);

  // Modbus communication runs at 9600 baud
  S.begin(9600, SWSERIAL_8N1);

  //Serial Monitor
  Serial.begin(57600);

  // Modbus slave ID 2
  node.begin(2, S); 
  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  //WiFi Connection here
  {
    WiFi.begin("", "");

    Serial.print("Connecting");
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
      digitalWrite(LED_RT, millis() / 1000 % 2);
    }
    Serial.println();

    digitalWrite(LED_RT, 0);
    digitalWrite(LED_GR, 1);

    Serial.print("Connected, IP address: ");
    Serial.println(WiFi.localIP());
  }

  //MQTT Connection here

  // Each client must have a unique client ID
  mqttClient.setId("SolisInv");

  // You can provide a username and password for authentication
  mqttClient.setUsernamePassword("your user", "your password");

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

}

void loop()
{
  int RegNo;
  
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED_GR, 1);
  } else {
    digitalWrite(LED_GR, 0);
  }

  //*MQTT Loop
  // call poll() regularly to allow the library to send MQTT keep alives which
  // avoids being disconnected by the broker
  mqttClient.poll();

  bool error = false;
  for (RegNo = 0; RegNo < REGISTERS_TO_READ; RegNo++)
  {
    RegResult[RegNo] = 0;

    RegResult[RegNo] = node.readInputRegisters(RegAddress[RegNo], RegWords[RegNo]);
  
    if (RegResult[RegNo] == node.ku8MBSuccess)
    {
      if (RegWords[RegNo] == 1)
      {
        RegValue[RegNo] = (node.getResponseBuffer(0) * RegScalar[RegNo]);
      }
      if (RegWords[RegNo] == 2)
      {
        RegValue[RegNo] = (((node.getResponseBuffer(1) << 16) + node.getResponseBuffer(0)  ) * RegScalar[RegNo]);
      }

      // indicate things are o.k.
      digitalWrite(LED_RT, millis() / 1000 % 2);
      yield();
      delay(500); //Frame interval is 300ms
    } else {
      error = true;
      Serial.println("Error reading inverter registers");      
    }
  }

  // Check error status and tranmitt MQTT message
  if (!error) {
    Serial.println("MQTT transmission:");      
    mqttClient.beginMessage(MQTT_TOPIC);
      //mqttClient.print(RegValue[RegNo]);
    StaticJsonDocument<200> jsonDoc;
    for (RegNo = 0; RegNo < REGISTERS_TO_READ; RegNo++)
    {
      jsonDoc[RegName[RegNo]] = RegValue[RegNo];
    }
    serializeJson(jsonDoc, Serial);
    serializeJson(jsonDoc, mqttClient);
    mqttClient.endMessage();
  }
}
