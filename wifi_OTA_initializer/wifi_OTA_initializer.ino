#include <WiFi.h>
#include <ArduinoOTA.h>

const char* ssid = "SIMOMIFI_G0U0_2.4GHz";
const char* password = "DwAJi54E";
// 192.168.76.215 -- use this ip in putty to connect to telnet server

WiFiServer wifiServer(23);   // TCP server on port 23 (Telnet)
WiFiClient client;

int otaVersion = 1; // Manually increment this before each OTA upload

unsigned long previousMillis = 0;
const long interval = 1000; // Print every 1 second

void setup() {
  Serial.begin(9600);
  
  // Sets WiFi mode to Station (STA) mode where the ESP32 connects to an existing WiFi network
  // as a client, rather than creating its own access point (AP mode)
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.println("Connecting to Wi-Fi...");
  int wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    wl_status_t current_status = WiFi.status(); // Get current status
    delay(500);
    Serial.print(".");
    Serial.print("("); Serial.print(current_status); Serial.print(")"); // Print status code
    wifi_attempts++;
    if (wifi_attempts > 20) { // ~10 second timeout
        Serial.println("\nFailed to connect after 10 seconds. Retrying...");
        WiFi.disconnect(true); // Disconnect before retry
        delay(1000); // Wait a second before retrying
        WiFi.begin(ssid, password); // Try connecting again
        wifi_attempts = 0; // Reset counter
    }
  }

  Serial.println("\nWi-Fi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  delay(100); // Short delay to ensure Serial buffer flushes

  ArduinoOTA.setHostname("esp32-nano-ota");
  ArduinoOTA.begin();
  Serial.println("OTA Initialized. Hostname: esp32-nano-ota");
  delay(100); // Short delay

  wifiServer.begin(); // Start TCP server
  wifiServer.setNoDelay(true); // For faster updates
  Serial.println("Telnet Server Started. Setup complete.");
  delay(100); // Short delay
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
    
    String message = "Version: " + String(otaVersion) + " - Time since connection: " + String(secondsSinceBoot) + "s";
    Serial.println(message);

    if (client && client.connected()) {
      client.println(message);
    }
  }
}
