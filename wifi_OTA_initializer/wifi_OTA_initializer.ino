#include <WiFi.h>
#include <ArduinoOTA.h>

const char* SSID    = "The Honk";
const char* PASS    = "NoPasswordRequired4u";
const char* OTA_PWD = "MySecureOTApass";

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("\nBooting...");

  // Connect to Wi-Fi
  WiFi.begin(SSID, PASS);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print('.');
  }
  Serial.println("\nWi-Fi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Configure OTA
  ArduinoOTA.setHostname("esp32-nano-ota");
  ArduinoOTA.setPassword(OTA_PWD);
  ArduinoOTA.setPort(3232);     // explicit, though 3232 is default
  ArduinoOTA.begin();
  Serial.println("OTA Ready");
}

void loop() {
  // Blink LED
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink >= 500) {
    lastBlink = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // Handle OTA requests
  ArduinoOTA.handle();
}
