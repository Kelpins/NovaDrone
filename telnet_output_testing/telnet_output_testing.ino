#include <WiFi.h>
#include <ArduinoOTA.h>

const char* ssid = "Mainframe ID.1248X";
const char* password = "novadrone";

// 192.168.76.215 -- use this ip in putty to connect to telnet server
WiFiServer wifiServer(23);   // TCP server on port 23 (Telnet)
WiFiClient client;

unsigned long previousMillis = 0;
const long interval = 1000; // Print every 1 second

void setup() {
  Serial.begin(115200);
  delay(500);  // Give time for Serial Monitor to open
  Serial.println();
  Serial.println("Booting...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to Wi-Fi");
  
  unsigned long startAttemptTime = millis();
  const unsigned long timeout = 15000; // 15 seconds timeout

  // Try to connect until timeout
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWi-Fi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to Wi-Fi. Restarting...");
    delay(2000);
    ESP.restart();  // Restart and try again
  }

  // Setup OTA
  ArduinoOTA.setHostname("esp32-nano-ota");
  ArduinoOTA.begin();
  Serial.println("OTA Ready");

  wifiServer.begin(); // Start TCP server
  wifiServer.setNoDelay(true); // For faster updates
}

void loop() {
  ArduinoOTA.handle();

  if (wifiServer.hasClient()) {
    if (!client || !client.connected()) {
      if (client) client.stop();
      client = wifiServer.available();
      Serial.println("New WiFi client connected");
      client.println("ESP32 Connected to Serial over WiFi!");
    } else {
      WiFiClient newClient = wifiServer.available();
      newClient.stop();
    }
  }

  if (client && client.connected()) {
    // Forward data ESP32 -> WiFi
    if (Serial.available()) {
      client.write(Serial.read());
    }
    // Forward data WiFi -> ESP32
    if (client.available()) {
      Serial.write(client.read());
    }
  }

  // ===== CUSTOM PRINT SECTION =====
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    unsigned long secondsSinceBoot = millis() / 1000;
    
    String message = "Time since boot: " + String(secondsSinceBoot) + "s";
    Serial.println(message);

    if (client && client.connected()) {
      client.println(message);
    }
  }
}