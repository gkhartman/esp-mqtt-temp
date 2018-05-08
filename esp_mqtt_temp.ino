/*  File: esp_mqtt_temp.ino
 *  Author: Garret Hartman
 *  Created: Mar 11, 2018
 *  Last Modified: May 7, 2018
 *  Description: ESP8266 Wifi+MQTT Temperature Sensor.
 */

#include <ESP8266WiFi.h>        // WiFiClient
#include <DallasTemperature.h>
#include <OneWire.h>
#include <PubSubClient.h>       // PubSubClient (MQTT Client)

// Data wire is plugged into gpio pin 2 on the esp8266
#define ONE_WIRE_BUS_PIN 2
const char* SSID = "";
const char* WIFI_PASSWD = "";
const char* MQTT_HOST = "";
const int MQTT_PORT = 1883;
const char* MQTT_USER = "";
const char* MQTT_PASSWD = "";
const char* MQTT_CLIENT_ID = "esp-temperature-1";
const int8_t MAX_WIFI_RETRY_TICKS = 40;
const int8_t MAX_MQTT_RETRY_COUNT = 40;
const int DEEP_SLEEP_USECONDS = 6e8; // 10 minutes in microseconds

// Init temperature sensor init
OneWire oneWire(ONE_WIRE_BUS_PIN);
DallasTemperature sensors(&oneWire);

// Wifi+MQTT client init 
WiFiClient espWifiClient;
PubSubClient client(espWifiClient);

void setup_wifi() {
    
    delay(10);  // 10ms delay

    // We start by connecting to a WiFi network
    Serial.print("\nConnecting to: ");
    Serial.println(SSID);
    WiFi.begin(SSID, WIFI_PASSWD);
    
    // Print '.' every 500ms until wifi connection confirmed
    int8_t retryTicks = 0;
    while( WiFi.status() != WL_CONNECTED ) {
        retryTicks++;
        if(retryTicks > MAX_WIFI_RETRY_TICKS) {
            ESP.deepSleep(DEEP_SLEEP_USECONDS);
        }
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println( WiFi.localIP() );
}

// Note: reconnect function also performs initial connection
void reconnect() {
    int8_t mqttRetryCount = 0;
    // Loop until we're reconnected
    while( !client.connected() ) {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if( client.connect( MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWD ) ) {
            Serial.println("Connected To MQTT Broker!");
        } 
        else {
            mqttRetryCount++;
            if(mqttRetryCount > MAX_MQTT_RETRY_COUNT) {
                Serial.print("Exceeded max MQTT reconnect count... Going into deep sleep for ");
                Serial.print(DEEP_SLEEP_USECONDS);
                Serial.print("microseconds");
                ESP.deepSleep(DEEP_SLEEP_USECONDS);
            }
            // Wait 5 seconds before retrying
            Serial.println(" . Auto retry in 5 seconds.");
            Serial.print("Failed to connect to MQTT Broker, rc=");
            Serial.print(client.state());
            delay(5000);
        }
    }
}

void setup() {
    Serial.begin(115200); // set baud rate

    setup_wifi(); // Connect to wifi
    
    // Set up mqtt client
    client.setServer(MQTT_HOST, MQTT_PORT);

    // Connect to MQTT Broker
    reconnect(); 
    client.loop();
    
    // Set up sensor
    pinMode(ONE_WIRE_BUS_PIN, INPUT);
    sensors.begin();
    sensors.setResolution(12); // set sensor resolution to 12 bits
    
    // Fetch temp from sensor and publish reading to MQTT broker
    sensors.requestTemperatures();
    float temp = sensors.getTempCByIndex(0);
    Serial.println(temp);
    String topicStr = String("sensor/temperature1");
    int publish_succeeded = client.publish( topicStr.c_str(), String(temp).c_str(), TRUE );
    
    if(!publish_succeeded) {
        Serial.print("Publish failed: ");
        Serial.println(topicStr.c_str());
    }

    // Allow 5 seconds for publish to finish before powering down for deep sleep
    delay(5000); 

    Serial.print("Entering deep sleep for ");
    Serial.print(DEEP_SLEEP_USECONDS);
    Serial.println(" useconds.");
    ESP.deepSleep(DEEP_SLEEP_USECONDS);
}

void loop() { /* Device should deep sleep before hitting this loop. */ }
