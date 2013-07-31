
/*
  Grow Box
 */
 
#include <SPI.h>
#include <WiFi.h>
#include <HttpClient.h>
#include <Cosm.h>
#include <dht11.h>  // Library from http://playground.arduino.cc/Main/DHT11Lib

#define DEBUG_NETWORK   true

#define COSM_API_KEY    "xkHHcDNujJhL01JLBPBRB0exbr-SAKx5RjJ1TmVnYVVDaz0g" // Cosm API Key
#define COSM_FEED_ID    99616          // Cosm Feed ID

#define DHT11PIN 5

dht11 DHT11;

#define pumpControlPin  3 
#define switchPin       2

int lightSensorReading = 0;
int soilHumiditySensorReading = 0;
float temperatureReading = 0; // Celsius
float humidityReading = 0; // Relative Humidity Perceptage

int soilHumidityTresholdLow = 120;
int soilHumidityTresholdHigh = 500;
boolean isLowSoilHumidity = false;
boolean isHighSoilHumidity = false;

String soilHumidityLabel = "SoilHumidity,";

char ssid[] = "Nirvana";    //  your network SSID (name) 
char pass[] = "computer";   // your network password

//char ssid[] = "rga@SF";
//char pass[] = "";

int status = WL_IDLE_STATUS;

// initialize the library instance:
WiFiClient client;

// Coms key to upload data
char cosmKey[] = COSM_API_KEY;

// Define the strings for our datastream IDs
char sensorId0[] = "Light";
char sensorId1[] = "AirTemperature";
char sensorId2[] = "AirHumidity";
char sensorId3[] = "SoilHumidity";
char sensorId4[] = "Pump";

CosmDatastream dataStreams[] = {
  CosmDatastream(sensorId0, strlen(sensorId0), DATASTREAM_INT),
  CosmDatastream(sensorId1, strlen(sensorId1), DATASTREAM_FLOAT),
  CosmDatastream(sensorId2, strlen(sensorId2), DATASTREAM_FLOAT),
  CosmDatastream(sensorId3, strlen(sensorId3), DATASTREAM_INT),
  CosmDatastream(sensorId4, strlen(sensorId4), DATASTREAM_INT),
};
  
// Finally, wrap the datastreams into a feed
CosmFeed feed(99616, dataStreams, 5 /* number of datastreams */);

CosmClient cosmclient(client);

unsigned long lastConnectionTime = 0;          // last time you connected to the server, in milliseconds
boolean lastConnected = false;                 // state of the connection last time through the main loop
const unsigned long postingInterval = 1000; //delay between updates to Pachube.com

int pumpDataBuffer = 0;  // Holds whether the pump was on since the last data transmission

int potReading = 0;

void setup() {
  
  // start serial port:
  Serial.begin(9600);
  
  // Setup the pin modes
  pinMode(pumpControlPin, OUTPUT);
  pinMode(switchPin, INPUT);

  // attempt to connect to Wifi network:
  if (DEBUG_NETWORK) {
    
    while ( status != WL_CONNECTED) { 
      
      Serial.print("Setup(): Attempting to connect to SSID: ");
      Serial.println(ssid);
      
      if (strcmp(pass,"")!=0) {
        Serial.println("Password is not empty.");
        status = WiFi.begin(ssid, pass);
      } else {
        Serial.println("Password is empty.");
        status = WiFi.begin(ssid);
      }
      // wait 10 seconds for connection:
      delay(10000);
    }
    
    Serial.println("Setup(): Connected to wifi.");
    
    // Print network status
    printWifiStatus();
  
  }
}


void loop() {
  
  sensorLoop();
  pumpLoop();

  if (DEBUG_NETWORK) {
    // if there's incoming data from the net connection.
    // send it out the serial port.  This is for debugging
    // purposes only:
    if (client.available()) {
      char c = client.read();
      Serial.print(c);
    }
  
    // if there's no net connection, but there was one last time
    // through the loop, then stop the client:
    if (!client.connected() && lastConnected) {
      Serial.println();
      Serial.println("disconnecting.");
      client.stop();
    }
  }

  // if you're not connected, and ten seconds have passed since
  // your last connection, then connect again and send data:
  if(!client.connected() && (millis() - lastConnectionTime > postingInterval)) {
    
    if (DEBUG_NETWORK) {
      // Send data
      sendData();
    }
    
    // note the time that the connection was made or attempted:
    lastConnectionTime = millis();
  }
  // store the state of the connection for next time through
  // the loop:
  lastConnected = client.connected();
  
}

// this method makes a HTTP connection to the server:
void sendData() {
  
  printDebugInfo();
  
  Serial.println("sendData() --------------------");
  
  dataStreams[0].setInt(lightSensorReading);
  dataStreams[1].setFloat(temperatureReading);
  dataStreams[2].setFloat(humidityReading);
  dataStreams[3].setInt(soilHumiditySensorReading);
  dataStreams[4].setInt(pumpDataBuffer);
  
  Serial.println("Uploading to Cosm...");
  int ret = cosmclient.put(feed,cosmKey);
  Serial.print("cosmclient.put returned ");
  Serial.println(ret);  
  
  Serial.println("End sendData() ----------------");

}


void printWifiStatus() {
  
  Serial.println("#################");
  Serial.println("# Wifi Status");
  
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}


/********************
 * Sensors
 */

void sensorLoop()
{
  
  // Read the analog sensors
  lightSensorReading = analogRead(A0);
  soilHumiditySensorReading = analogRead(A1);  

  // Read the potentiometer
  potReading = analogRead(A2);
  
  // Set the tresholds
  soilHumidityTresholdLow = map(potReading,1,1024,0,300);
  
  // Check the tresholds
  isLowSoilHumidity = soilHumiditySensorReading < soilHumidityTresholdLow;
  isHighSoilHumidity = soilHumiditySensorReading > soilHumidityTresholdHigh;
  
  dht11Loop();
  
}

void dht11Loop()
{
  Serial.println("-- DHT11 Loop -------------------");

  int chk = DHT11.read(DHT11PIN);

  //Serial.print("Read sensor: ");
  switch (chk)
  {
    case DHTLIB_OK: 
                Serial.println("DHT11: OK"); 
                temperatureReading = DHT11.temperature;
                humidityReading = DHT11.humidity;
                break;
    case DHTLIB_ERROR_CHECKSUM: 
                Serial.println("DHT11: Checksum error"); 
                break;
    case DHTLIB_ERROR_TIMEOUT: 
                Serial.println("DHT11: Time out error"); 
                break;
    default: 
                Serial.println("DHT11: Unknown error"); 
                break;
  }

  delay(2000);
  
  Serial.println("-- END DHT11 Loop -------------------");
}

/********************
 * Pump
 */

const int pumpStateOff = 0;
const int pumpStateOn = 150;

unsigned long lastPumpControlTime = 0;             // Last time the pump was controlled, in milliseconds
unsigned long pumpControlInterval = 1000;           // Delay between pump control commnds, in milliseconds
const unsigned long pumpDelayInterval = 30000;   
const unsigned long pumpMonitorInterval = 1000;

boolean isPumping = false;                       // Is the pump pumping?
unsigned long lastPumpingTime = 0;               // Time when the pump started pumping
const unsigned long pumpingInterval = 5000;      // How long the pump should pump


void pumpLoop()
{
    
  if (isPumping) 
  {
    // If we have been pumping long enough
    if (millis() - lastPumpingTime > pumpingInterval)
    {
      Serial.println("Pumping for long enough");
      pumpOff();
    }
  } else {
    // Failsafe: if we are not pumping, turn the pump off.
    pumpOff();
  }
  
  if (millis() - lastPumpControlTime > pumpControlInterval) 
  {
    Serial.println("pumpLoop(): command pump");
    printDebugInfo();
    
    if (isLowSoilHumidity && !isPumping) 
    {
      pumpControlInterval = pumpDelayInterval;
      pumpOn();
    } 
    else 
    {
      pumpControlInterval = pumpMonitorInterval;
    }
    
    lastPumpControlTime = millis();
  }
  
}

void pumpOn() 
{
  Serial.println("pumpOn()");
  isPumping = true;
  lastPumpingTime = millis();
  pumpDataBuffer = 1;
  analogWrite(pumpControlPin,pumpStateOn);
}

void pumpOff() 
{
  //Serial.println("pumpOff()");
  analogWrite(pumpControlPin,pumpStateOff);
  isPumping = false;
  lastPumpingTime = 0;
}


void printDebugInfo() 
{
  if (true) {
    
  Serial.println("-- Debug Info --------------------");
  
  Serial.print("Light: ");
  Serial.println(lightSensorReading); 
  
  Serial.print("Humidity (%): ");
  Serial.println((float)humidityReading, 2);

  Serial.print("Temperature (oC): ");
  Serial.println((float)temperatureReading, 2);
  
  Serial.print("Soil Humidity: ");
  Serial.println(soilHumiditySensorReading);
  
  Serial.print("Is Low Soil Humidity?: ");
  Serial.println(isLowSoilHumidity);
  
  Serial.print("isPumping: ");
  Serial.println(isPumping);
  
  Serial.print("pumpControlInterval: ");
  Serial.println(pumpControlInterval);
  
  Serial.print("pumpDataBuffer: ");
  Serial.println(pumpDataBuffer);
  
  Serial.print("Potentiometer: ");
  Serial.println(potReading);
  
  Serial.print("soilHumidityTresholdLow: ");
  Serial.println(soilHumidityTresholdLow);
  
  Serial.println("-- End Debug Info -----------------");
  }
}
 

