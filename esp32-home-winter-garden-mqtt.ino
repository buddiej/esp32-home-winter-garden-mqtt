#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <stdlib.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>



#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include <Wire.h>
#include <Adafruit_INA219.h>

#include "_authentification.h"  /* credentials for WIFI and mqtt. Located in libraries folder */


/*****************************************************************************************/
/*                                    GENERAL DEFINE                                     */
/*****************************************************************************************/
#define TRUE  1
#define FALSE 0

#define STATE_OFF           0
#define STATE_ON            1

/*****************************************************************************************/
/*                                    PROJECT DEFINE                                     */
/*****************************************************************************************/

#define MQTT_PAYLOAD_MAX 250

/* Receive topics */
#define TOPIC_GET_WINTERGARDEN_DHT_MESSAGE   "wintergarden/get/dht"
#define TOPIC_GET_WINTERGARDEN_INA219        "wintergarden/get/ina219"


/* Send topics */
#define TOPIC_SET_MESSAGE_CANDLE            "wintergarden/set/candle"

#define DHTPIN 17     // Digital pin connected to the DHT sensor 
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment the type of sensor in use:
//#define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

#define DHT_HUMIDITY_OFFSET -8

#define MOS_FET_OUTPUT 16 // pin that controls the MOSFET


// setting PWM properties
const int freq = 1000;
const int ledChannel = 0;
const int resolution = 8;


/*****************************************************************************************/
/*                                     TYPEDEF ENUM                                      */
/*****************************************************************************************/


/*****************************************************************************************/
/*                                   TYPEDEF STRUCT                                      */
/*****************************************************************************************/
typedef struct DHT_SENSOR
{
  char name[12];
  int  version;
  int  sensor_id;
  float   max_value_temp;   
  float   min_value_temp;
  float   act_value_temp;
  float   resolution_temp;
  float   max_value_humidity;   
  float   min_value_humidity;
  float   act_value_humidity;
  float   resolution_humidity;
}T_DHT_SENSOR;




/*****************************************************************************************/
/*                                         VARIABLES                                     */
/*****************************************************************************************/
/* create an instance of WiFiClientSecure */
WiFiClient espClient;
PubSubClient client(espClient);


int mqttRetryAttempt = 0;
int wifiRetryAttempt = 0;
             
long lastMsg = 0;
long loop_2sec_counter = 0;
long loop_2sec_counterOld = 0;

DHT_Unified dht(DHTPIN, DHTTYPE);

T_DHT_SENSOR Dht_Sensor;


Adafruit_INA219 ina219;




static void receivedCallback(char* topic, byte* payload, unsigned int length);
static void mqttconnect(void);

#define DEBUG_MQTT_RECEIVER   Serial.print("Message received: ");  \
                              Serial.print(topic); \
                              Serial.print("\t"); \
                              Serial.print("payload: "); \
                              Serial.println(PayloadString);

/**************************************************************************************************
Function: ArduinoOta_Init()
Argument: void
return: void
**************************************************************************************************/
void ArduinoOta_Init(void)
{
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("esp32-home-winter-garden-mqtt");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
}                              
/*************************************************************************************************/
/**************************************************************************************************
Function: setup()
return: void
**************************************************************************************************/
/*************************************************************************************************/
void setup()
{
  sensor_t sensor_temp;
  sensor_t sensor_humidity;
  
  Serial.begin(115200);

  Serial.println(" ");
  Serial.println("######################################");
  Serial.println("# Program Home-winter-garden v0.1    #");
  Serial.println("######################################");
  Serial.println(__FILE__);
  Serial.println(" ");
  Serial.println("Starting ...");
  Serial.println(" ");

  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(1000);
    Serial.print(".");
    wifiRetryAttempt++;
    if (wifiRetryAttempt > 5) 
    {
      Serial.println("Restarting!");
      ESP.restart();
    }
  }

  ArduinoOta_Init();
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("IP address of server: ");
  Serial.println(serverHostname);
  /* set SSL/TLS certificate */
  /* configure the MQTT server with IPaddress and port */
  client.setServer(serverHostname, 1883);
  /* this receivedCallback function will be invoked
    when client received subscribed topic */
  client.setCallback(receivedCallback);

  /******************/
  /* user init here */
  /******************/

  /* Initialize device. */
  dht.begin();
  Serial.println(F("DHT22 Unified Sensor Example"));


  /* Get temperature sensor details. */
  dht.temperature().getSensor(&sensor_temp);
  /* Get humidity sensor details. */
  dht.humidity().getSensor(&sensor_humidity);

  memcpy(&Dht_Sensor.name, &sensor_temp.name,5);
  Dht_Sensor.version = sensor_temp.version;
  Dht_Sensor.sensor_id = sensor_temp.sensor_id;
  Dht_Sensor.max_value_temp = sensor_temp.max_value;
  Dht_Sensor.min_value_temp = sensor_temp.min_value;
  Dht_Sensor.resolution_temp = sensor_temp.resolution;
  Dht_Sensor.max_value_humidity = sensor_humidity.max_value;
  Dht_Sensor.min_value_humidity = sensor_humidity.min_value;
  Dht_Sensor.resolution_humidity = sensor_humidity.resolution;
  
  Serial.println(F("-------- DHT22 Sensor --------------"));
  Serial.print  (F("Sensor Type: ")); Serial.println(Dht_Sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(Dht_Sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(Dht_Sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(Dht_Sensor.max_value_temp); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(Dht_Sensor.min_value_temp); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(Dht_Sensor.resolution_temp); Serial.println(F("°C"));
  Serial.print  (F("Max Value:   ")); Serial.print(Dht_Sensor.max_value_humidity); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(Dht_Sensor.min_value_humidity); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(Dht_Sensor.resolution_humidity); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));



  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219.setCalibration_16V_400mA();

  Serial.println("Measuring voltage and current with INA219 ...");
  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(MOS_FET_OUTPUT, ledChannel);
 
  Serial.println("Setup finished ... ");
}

/*************************************************************************************************/
/**************************************************************************************************
Function: loop()
return: void
**************************************************************************************************/
/*************************************************************************************************/
void loop() 
{

  ArduinoOTA.handle();
  
  /* if client was disconnected then try to reconnect again */
  if (!client.connected()) 
  {
    mqttconnect();
  }
  /* this function will listen for incomming
  subscribed topic-process-invoke receivedCallback */
  client.loop();
 

  /* we increase counter every 5 secs we count until 5 secs reached to avoid blocking program if using delay()*/
  long now = millis();
  
  /* calling every 2 sec. */
  if (now - lastMsg > 2000)
  {
    /* store timer value */
    lastMsg = now;

    loop_2sec_counter++;
  }

  if(loop_2sec_counter != loop_2sec_counterOld)
  {
    if(loop_2sec_counter % 2 == 0)
    {
      /* call every 4 sec. */
      //Serial.println("1st call every 4 sec.");

      /* Get temperature event and print its value. */
      sensors_event_t event_temp;
      sensors_event_t event_humidity;

      dht.temperature().getEvent(&event_temp);
      if (isnan(event_temp.temperature)) 
      {
        Serial.println(F("Error reading temperature!"));
      }
      else 
      {
        Dht_Sensor.act_value_temp = event_temp.temperature;
        Serial.print(F("Temperature: "));
        Serial.print(Dht_Sensor.act_value_temp);
        Serial.println(F("°C"));
      }
      // Get humidity event and print its value.
      dht.humidity().getEvent(&event_humidity);
      if (isnan(event_humidity.relative_humidity)) 
      {
        Serial.println(F("Error reading humidity!"));
      }
      else 
      {
        /*correction of humidity*/
        event_humidity.relative_humidity = event_humidity.relative_humidity + DHT_HUMIDITY_OFFSET;
        Dht_Sensor.act_value_humidity = event_humidity.relative_humidity;
        Serial.print(F("Humidity: "));
        Serial.print(Dht_Sensor.act_value_humidity);
        Serial.println(F("%"));
      }
      
      
      /********************************************************************************************/
      /************************      HANDLING OF Send MQTT TOPICS for DTH *************************/ 
      /********************************************************************************************/
      /*200 Byte of RAM used for json Object */
      StaticJsonDocument<MQTT_PAYLOAD_MAX> doc;
      char Buffer[MQTT_PAYLOAD_MAX];
        
      char temperature[8];
      char humidity[8];
      char res_temp[8];
      char res_humidity[8];

      dtostrf(Dht_Sensor.act_value_temp,  6, 2, temperature);
      dtostrf(Dht_Sensor.act_value_humidity,  6, 2, humidity);
      dtostrf(Dht_Sensor.resolution_temp,  2, 2, res_temp);
      dtostrf(Dht_Sensor.resolution_humidity,  2, 2, res_humidity);

      /* Add values in the document */
      doc["temperature"] = temperature;
      doc["humidity"] = humidity;
      doc["name"] = Dht_Sensor.name;
      doc["version"] = Dht_Sensor.version;
      doc["sensor_id"] = Dht_Sensor.sensor_id;
      doc["max_value_temp"] = Dht_Sensor.max_value_temp;
      doc["min_value_temp"] = Dht_Sensor.min_value_temp;
      doc["res_temp"] = res_temp;
      doc["max_val_humi"] = Dht_Sensor.max_value_humidity;
      doc["min_val_humi"] = Dht_Sensor.min_value_humidity;
      doc["res_humi"] = res_humidity;
      /* serialize the content */
      serializeJson(doc, Buffer);
      /* publish the buffer content */
      client.publish(TOPIC_GET_WINTERGARDEN_DHT_MESSAGE, Buffer, false);
    }
    else
    {
      /* call every 4 sec. */
      //Serial.println("2nd call every 4 sec.");
      float shuntvoltage = 0;
      float busvoltage = 0;
      float current_mA = 0;
      float loadvoltage = 0;
      float power_mW = 0;
    
      shuntvoltage = ina219.getShuntVoltage_mV();
      busvoltage = ina219.getBusVoltage_V();
      current_mA = ina219.getCurrent_mA();
      power_mW = ina219.getPower_mW();
      loadvoltage = busvoltage + (shuntvoltage / 1000);

      Serial.println(F("-------- INA219 current Sensor --------------"));
      Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
      Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
      Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
      Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
      Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
      Serial.println(F("---------------------------------------------"));

      /********************************************************************************************/
      /************************      HANDLING OF Send MQTT TOPICS for INA 219  ********************/ 
      /********************************************************************************************/
      /*200 Byte of RAM used for json Object */
      StaticJsonDocument<MQTT_PAYLOAD_MAX> doc;
      char Buffer[MQTT_PAYLOAD_MAX];
        
      /* Add values in the document */
      doc["shuntvoltage"] = shuntvoltage;
      doc["busvoltage"] = busvoltage;
      doc["current_mA"] = current_mA;
      doc["loadvoltage"] = loadvoltage;
      doc["power_mW"] = power_mW;
                              
      
      /* serialize the content */
      serializeJson(doc, Buffer);
      /* publish the buffer content */
      client.publish(TOPIC_GET_WINTERGARDEN_INA219, Buffer, false);
    }
  }
 
  
    
  /* store counter value */
  loop_2sec_counterOld = loop_2sec_counter;
}

/**************************************************************************************************
Function: receivedCallback()
Argument: char* topic ; received topic
          byte* payload ; received payload
          unsigned int length ; received length
return: void
**************************************************************************************************/
void receivedCallback(char* topic, byte* payload, unsigned int length) 
{
  uint8_t Loc_Status;

  uint32_t Loc_pwmvalue;
  int out;

  StaticJsonDocument<MQTT_PAYLOAD_MAX> doc;

  char PayloadString[length + 1 ];
  /* convert payload in string */
  for(byte i=0;i<length;i++)
  {
    PayloadString[i] = payload[i];
  }
  PayloadString[length] = '\0';
  
  /* Debug */
  DEBUG_MQTT_RECEIVER


  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, PayloadString);

  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  long time = doc["time"];
  Serial.println(time);


  /********************************************************************************************/
  /********************      HANDLING OF Received MQTT TOPICS WITH JASON     ******************/ 
  /********************************************************************************************/
  
  /*+++++++++++++++++++++++++++++ Set control +++++++++++++++++++++++++++++++++++++++*/ 
 
  if(strcmp(topic, TOPIC_SET_MESSAGE_CANDLE)==0)
  {
    if(doc.containsKey("value")) 
    {
      Loc_pwmvalue = doc["value"];
      Serial.print("status candle received set: ");
      Serial.println(Loc_pwmvalue, DEC);

      /* set value */
      if(Loc_pwmvalue > 255)
      {
        Loc_pwmvalue = 255; /* limit to 5V*/
      }
      ledcWrite(ledChannel, Loc_pwmvalue);

      
    }
  }


}


/**************************************************************************************************
Function: mqttconnect()
Argument: void
return: void
**************************************************************************************************/
void mqttconnect(void)
{
  /* Loop until reconnected */
  while (!client.connected()) 
  {
    Serial.print("MQTT connecting ...");
    /* client ID */
    String clientId = "esp32-home-winter-garden-mqtt";
    /* connect now */
    if (client.connect(clientId.c_str(), serverUsername.c_str(), serverPassword.c_str()))
    {
      Serial.println("connected");

      /*********************/
      /* subscribe topic's */
      /*********************/
      client.subscribe(TOPIC_SET_MESSAGE_CANDLE);
    
    } 
    else 
    {
      Serial.print("failed, status code =");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
      /* Wait 5 seconds before retrying */
      delay(5000);
      mqttRetryAttempt++;
      if (mqttRetryAttempt > 5) 
      {
        Serial.println("Restarting!");
        ESP.restart();
      }
    }
  }
}
