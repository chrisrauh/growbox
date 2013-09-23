
#include <WiFi.h>

int redPin = 5;
int greenPin = 9;
int bluePin = 6;

char ssid[] = "Nirvana";         // SSID of your network  
char pass[] = "computer";        // password of your WPA Network
int wifiStatus = WL_IDLE_STATUS; // the Wifi radio's status

void setup()
{
  Serial.begin(57600);

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);  
 
  analogWrite(5, 255);
  analogWrite(6, 255);
  analogWrite(9, 255);
  
  
  // attempt to connect to Wifi network:
  while ( wifiStatus != WL_CONNECTED) { 
    
    Serial.print("Attempting to connect to network, SSID: ");
    Serial.println(ssid);
    setColorNum(4);
        
    wifiStatus = WiFi.begin(ssid, pass);
    
    Serial.print("Wifi status:");
    Serial.println(wifiStatus);

    if (wifiStatus != WL_CONNECTED) {
      Serial.println("Wifi NOT connected");
      Serial.println("Waiting 10 seconds to try again...");
      setColorNum(1);
      delay(2000);
    }
    
  }
  
  Serial.println("Wifi connected!");
  setColorNum(2);
  delay(1000);
  
}

void loop()
{
  for(int i = 0; i<360; i++){
    //convert 0-360 angle to radian (needed for sin function)
    float rad = DEG_TO_RAD * i;

    //calculate sin of angle as number between 0 and 255
    int sinOut = constrain((sin(rad) * 128) + 128, 0, 255); 

    analogWrite(redPin, 255-sinOut);
    analogWrite(greenPin, 255);
    analogWrite(bluePin, 255-sinOut);

    delay(10);
  }
}


void setColorNum(int c) {
  switch (c) {
    case 0:
      Serial.println("Black(Off)");
      setColor(0,0,0); // Black
      break;
    case 1: 
      Serial.println("Red");
      setColor(255, 0, 0);  // red
      break;
    case 2:
      Serial.println("Green");
      setColor(0, 255, 0);  // green
      break;
    case 3:
      Serial.println("Blue");
      setColor(0, 0, 255);  // blue
      break;
    case 4:
      Serial.println("Yellow");
      setColor(255, 255, 0);  // yellow
      break;
    case 5:
      Serial.println("Purple");
      setColor(255, 0, 255);  // purple
      break;
    case 6:
      Serial.println("Aqua");
      setColor(0, 255, 255);  // aqua
      break;
  }
}

void setColor(int red, int green, int blue)
{
  analogWrite(redPin, 255-red);
  analogWrite(greenPin, 255-green);
  analogWrite(bluePin, 255-blue);  
}
