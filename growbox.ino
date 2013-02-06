/*
  Grow Box
 */
 
#include <SPI.h>
#include <WiFi.h>

#define DEBUG_NETWORK   true

#define APIKEY          "xkHHcDNujJhL01JLBPBRB0exbr-SAKx5RjJ1TmVnYVVDaz0g" // Cosm API Key
#define FEEDID          99616            // Cosm Feed ID
#define USERAGENT       "Bonsai Box"     // Cosm User Agent (Project Name)

#define pumpControlPin  3 
#define switchPin       2

int lightSensorReading = 0;
int soilHumiditySensorReading = 0;

int soilHumidityTresholdLow = 120;
int soilHumidityTresholdHigh = 500;
boolean isLowSoilHumidity = false;
boolean isHighSoilHumidity = false;

String soilHumidityLabel = "SoilHumidity,";

char ssid[] = "Nirvana";    //  your network SSID (name) 
char pass[] = "computer";   // your network password

int status = WL_IDLE_STATUS;

// initialize the library instance:
WiFiClient client;
// if you don't want to use DNS (and reduce your sketch size)
// use the numeric IP instead of the name for the server:
IPAddress server(216,52,233,122);      // numeric IP for api.pachube.com
//char server[] = "api.pachube.com";   // name address for pachube API

unsigned long lastConnectionTime = 0;          // last time you connected to the server, in milliseconds
boolean lastConnected = false;                 // state of the connection last time through the main loop
const unsigned long postingInterval = 10*1000; //delay between updates to Pachube.com

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
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(ssid);
      status = WiFi.begin(ssid, pass);
      // wait 10 seconds for connection:
      delay(10000);
    }
  
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
      sendData(lightSensorReading,soilHumiditySensorReading);
    }
    
    // note the time that the connection was made or attempted:
    lastConnectionTime = millis();
  }
  // store the state of the connection for next time through
  // the loop:
  lastConnected = client.connected();
  
}

// this method makes a HTTP connection to the server:
void sendData(int lightData, int humidityData) {
  
  Serial.println("sendData() --------------------");
  Serial.println("Connecting...");
  
  // if there's a successful connection:
  if (client.connect(server, 80)) {
    
    Serial.println("Connected.");
    Serial.println("Sending data...");
    // send the HTTP PUT request:
    client.print("PUT /v2/feeds/");
    client.print(FEEDID);
    client.println(".csv HTTP/1.1");
    client.println("Host: api.pachube.com");
    client.print("X-PachubeApiKey: ");
    client.println(APIKEY);
    client.print("User-Agent: ");
    client.println(USERAGENT);

    int thisLength = String("Light,").length() + 1 + getLength(lightData) + 1;
    thisLength += soilHumidityLabel.length() + 1 + getLength(humidityData) + 1;
    thisLength += String("Pump,").length() + 1 + getLength(pumpDataBuffer);
    
    Serial.print("Length=");
    Serial.println(thisLength);
    
    client.print("Content-Length: ");
    client.println(thisLength);

    // last pieces of the HTTP PUT request:
    client.println("Content-Type: text/csv");
    client.println("Connection: close");
    client.println();

    // here's the actual content of the PUT request:
    client.print("Light,");           // 6
    client.println(lightData);        // 3 + 1
    client.print(soilHumidityLabel);  // 9
    client.println(humidityData);     // 3 + 1
    client.print("Pump,");            // 5
    client.println(pumpDataBuffer);   // 1
    pumpDataBuffer = 0;
    
    Serial.println("Done sending data.");

  } 
  else 
  {
    // if you couldn't make a connection:
    Serial.println("Connection failed.");
    Serial.println("Disconnecting.");
    client.stop();
  }

}


// This method calculates the number of digits in the
// sensor reading.  Since each digit of the ASCII decimal
// representation is a byte, the number of digits equals
// the number of bytes:

int getLength(int someValue) {
  // there's at least one byte:
  int digits = 1;
  // continually divide the value by ten, 
  // adding one to the digit count for each
  // time you divide, until you're at 0:
  int dividend = someValue /10;
  while (dividend > 0) {
    dividend = dividend /10;
    digits++;
  }
  // return the number of digits:
  return digits;
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
 * Sensor
 */

void sensorLoop()
{
  
  // Read the analog sensors
  lightSensorReading = analogRead(A0);
  soilHumiditySensorReading = analogRead(A1);  
  
  // Check the tresholds
  isLowSoilHumidity = soilHumiditySensorReading < soilHumidityTresholdLow;
  isHighSoilHumidity = soilHumiditySensorReading > soilHumidityTresholdHigh;

  // Read the potentiometer
  potReading = analogRead(A2);
  
  // Set the tresholds
  soilHumidityTresholdLow = map(potReading,1,1024,0,300);
  
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
const unsigned long pumpingInterval = 10000;      // How long the pump should pump


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
  Serial.println("-- Debug Info --------------------");
  Serial.print("Light: ");
  Serial.print(lightSensorReading); 
  Serial.print(" - Soil Humidity: ");
  Serial.print(soilHumiditySensorReading);
  Serial.print(" - is Low Soil Humidity?: ");
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
}
 

