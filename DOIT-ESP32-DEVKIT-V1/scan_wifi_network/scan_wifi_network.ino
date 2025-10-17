#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nScanning available Wi-Fi networks...");
  
  int numNetworks = WiFi.scanNetworks();
  if (numNetworks == 0) {
    Serial.println("No Wi-Fi networks found.");
  } else {
    Serial.print(numNetworks);
    Serial.println(" Wi-Fi networks found.");
    
    for (int i = 0; i < numNetworks; i++) {
      Serial.print(i+1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(" dBm)");
      Serial.print(" - ");
      Serial.println(WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "Open" : "Encrypted");
    }
  }
}

void loop() {
  
}
