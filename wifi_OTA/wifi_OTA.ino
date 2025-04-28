#include <WiFi.h>
#include <ArduinoOTA.h>

// Separate secrets file (just like in WiFi101 examples)
#include "arduino_secrets.h" 
// In arduino_secrets.h:
// const char* SECRET_SSID = "your-ssid";
// const char* SECRET_PASS = "your-password";
// const char* OTA_PASSWORD = "your-ota-password";

void setup() {
  Serial.begin(9600);
  Serial.println("\nBooting...");

  WiFi.mode(WIFI_STA);           // Better connection stability
  WiFi.begin(SECRET_SSID, SECRET_PASS);

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    retries++;
    if (retries > 20) {  // ~10 seconds timeout
      Serial.println("\nConnection Failed! Rebooting...");
      delay(2000);
      ESP.restart();
    }
  }

  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Set OTA Hostname
  ArduinoOTA.setHostname("esp32-drone");

  // Set OTA password protection
  ArduinoOTA.setPassword(OTA_PASSWORD);

  // Optional callbacks for better OTA feedback
  ArduinoOTA.onStart([]() {
    Serial.println("Start updating...");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nUpdate finished!");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  
  Serial.println("Ready for OTA!");
}

void loop() {
  ArduinoOTA.handle();
}
