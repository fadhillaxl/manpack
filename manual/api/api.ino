#include <Arduino_JSON.h>
#include "SPIFFS.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Stepper.h>

const char* ssid = "Ihub61";
const char* password = "Ihub2019";

const int stepsPerRevolution = 2000;

Stepper myStepper(stepsPerRevolution, 2,15);

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

// Json Variable to Hold Sensor Readings
JSONVar readings;

void initSPIFFS() {
  if (!SPIFFS.begin()) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("");
  Serial.println(WiFi.localIP());
}

String getAccReadings() {
  
  float contoh = 90.00;
  readings["accX"] = String(contoh);
  readings["accY"] = String(contoh);
  readings["accZ"] = String(contoh);
  String accString = JSON.stringify (readings);
  return accString;
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  initWiFi();
  initSPIFFS();
    myStepper.setSpeed(60);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });

      server.on("/left", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "OK");
    Serial.println("kanan");
    myStepper.step(-stepsPerRevolution);
  });

   server.on("/right", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "OK");
    Serial.println("kiri");
    myStepper.step(stepsPerRevolution);
  });

  server.serveStatic("/", SPIFFS, "/");

    // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);

  server.begin();
  
}

void loop() {
  // put your main code here, to run repeatedly:
//  events.send(getAccReadings().c_str(),"getAccReadings");
}
