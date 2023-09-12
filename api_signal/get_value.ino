#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

const char* ssid = "SINNAR";
const char* password = "12345678";

//Your Domain name with URL path or IP address with path
const char* serverName = "http://192.168.40.1/parameters/satellite";
//http://192.168.40.1/parameters/satellite
//http://www.randomnumberapi.com/api/v1.0/randomredditnumber
// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
String sensorReadings;
float sensorReadingsArr[0];


void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
 
  Serial.println("Timer set to 5 seconds (timerDelay variable), it will take 5 seconds before publishing the first reading.");
}

void loop() {
  //Send an HTTP POST request every 10 minutes
   if (WiFi.status() == WL_CONNECTED) { //Check WiFi connection status
 
    HTTPClient http;  //Declare an object of class HTTPClient
 
    http.begin(serverName);  //Specify request destination
    int httpCode = http.GET();                                  //Send the request
 
    if (httpCode > 0) { //Check the returning code
 
      String payload = http.getString();   //Get the request response payload
 //     Serial.println(payload);             //Print the response payload
      
     char json[310];
     payload.toCharArray(json,310);
     StaticJsonDocument<200> doc;
     deserializeJson(doc,json);

     int quality = doc["satellite"]["quality"];

     Serial.println(String(quality));
     
      
    
//      for (int i = 0; i < keys.length(); i++) {
//        JSONVar value = myObject[keys[i]];
//        Serial.print(keys[i]);
//        Serial.print(" = ");
//        Serial.println(value);
//        sensorReadingsArr[i] = int(value);
//        
//      }
//      Serial.println("1 = ");
//      Serial.println(sensorReadingsArr[0]);
//      int x = sensorReadingsArr[0];
//      Serial.print(x);
      
    }
 
    http.end();   //Close connection
    
  }
 
  delay(3000);    
}
