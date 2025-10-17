#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include <BluetoothSerial.h>
#include <WebServer.h>

// Servo angles
#define SWITCH_ON_ANGLE_18 100
#define SWITCH_OFF_ANGLE_18 150
#define SWITCH_ON_ANGLE_5  80
#define SWITCH_OFF_ANGLE_5 25

// Servo pins
#define servo18Pin 13
#define servo5Pin  12

// WiFi credentials
const char* ssid     = "YOUR WIFI SSID";
const char* password = "YOUR WIFI PASSWORD";

// Static IP configuration
IPAddress staticIP(192, 168, 1, 99);
IPAddress gateway(192, 168, 1, 254);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(8, 8, 8, 8);

// Global objects
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "time.google.com", 20700, 60000); // 5h45m offset for Nepalese Time
WebServer server(80);
Servo servo18, servo5;
BluetoothSerial SerialBT;

// States
bool is18WOn = false;
bool is5WOn = false;
int alarmHour = 6;
int alarmMinute = 30;

// Timing variables
unsigned long lastIPPrint = 0;
unsigned long lastTimeCheck = 0;

// ----------- Utility Functions ----------------

void connectToWiFi() {
  WiFi.disconnect();
  WiFi.config(staticIP, gateway, subnet, dns);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\nConnected! IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi failed, retrying in 10s...");
  }
}

// Reusable function (attach → move → wait → detach)
void moveServo(Servo &servo, int pin, int angle) {
  servo.attach(pin, 500, 2500);
  servo.write(angle);
  delay(30);  
  servo.release();
}

void on(int n) {
  if (n == 18) {
    moveServo(servo18, servo18Pin, SWITCH_ON_ANGLE_18);
    is18WOn = true;
  } else if (n == 5) {
    moveServo(servo5, servo5Pin, SWITCH_ON_ANGLE_5);
    is5WOn = true;
  }
}

void off(int n) {
  if (n == 18) {
    moveServo(servo18, servo18Pin, SWITCH_OFF_ANGLE_18);
    is18WOn = false;
  } else if (n == 5) {
    moveServo(servo5, servo5Pin, SWITCH_OFF_ANGLE_5);
    is5WOn = false;
  }
}

void checkTimeAndControlLights() {
  if (!timeClient.update() && !timeClient.forceUpdate()) return;

  int currentHour = timeClient.getHours();
  int currentMinute = timeClient.getMinutes();

  Serial.printf("Time: %02d:%02d\n", currentHour, currentMinute);

  if (currentHour == alarmHour && currentMinute == alarmMinute) {
    on(18);
    on(5);
  }
}

void handleCommand(int state, bool isBluetooth) {
  Serial.printf("Command: %d\n", state);

  switch (state) {
    case 11: on(18); off(5); break;
    case 10: off(18); break;
    case 21: on(5); off(18); break;
    case 20: off(5); break;
    case 99:
      if (isBluetooth) {
        SerialBT.printf("Alarm: %02d:%02d, 18W: %s, 5W: %s, IP: %s\n",
          alarmHour, alarmMinute,
          is18WOn ? "ON" : "OFF",
          is5WOn ? "ON" : "OFF",
          WiFi.localIP().toString().c_str()
        );
      }
      Serial.printf("Alarm: %02d:%02d, 18W: %s, 5W: %s, IP: %s\n",
        alarmHour, alarmMinute,
        is18WOn ? "ON" : "OFF",
        is5WOn ? "ON" : "OFF",
        WiFi.localIP().toString().c_str()
      );
      break;
    default:
      Serial.println("Invalid command");
      break;
  }
}

// ----------- Web Handlers -----------

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><title>ESP32 Light Control</title>";
  html += "<style>body{font-family:Arial;padding:20px}form{margin:10px 0}button{padding:8px}</style>";
  html += "</head><body><h1>ESP32 Light Control</h1>";
  html += "<p>Alarm: " + String(alarmHour) + ":" + (alarmMinute < 10 ? "0" : "") + String(alarmMinute) + "</p>";
  html += "<p>18W Light: " + String(is18WOn ? "ON" : "OFF") + "</p>";
  html += "<p>5W Light: " + String(is5WOn ? "ON" : "OFF") + "</p>";
  html += "<form method='POST' action='/setAlarm'>Set Alarm: ";
  html += "<input type='number' name='hour' min='0' max='23' value='" + String(alarmHour) + "'> : ";
  html += "<input type='number' name='minute' min='0' max='59' value='" + String(alarmMinute) + "'>";
  html += "<input type='submit' value='Set'></form>";
  html += "<h2>Light Control</h2>";
  html += "<form method='POST' action='/control'><button name='cmd' value='11'>18W On, 5W Off</button></form>";
  html += "<form method='POST' action='/control'><button name='cmd' value='10'>18W Off</button></form>";
  html += "<form method='POST' action='/control'><button name='cmd' value='21'>5W On, 18W Off</button></form>";
  html += "<form method='POST' action='/control'><button name='cmd' value='20'>5W Off</button></form>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleControl() {
  if (server.hasArg("cmd")) {
    int cmd = server.arg("cmd").toInt();
    handleCommand(cmd, false);
    server.send(200, "text/html", "<p>Command: " + String(cmd) + "</p><a href='/'>Back</a>");
  } else {
    server.send(400, "text/html", "<p>Missing command.</p><a href='/'>Back</a>");
  }
}

void handleSetAlarm() {
  if (server.hasArg("hour") && server.hasArg("minute")) {
    int newHour = server.arg("hour").toInt();
    int newMinute = server.arg("minute").toInt();
    if (newHour >= 0 && newHour <= 23 && newMinute >= 0 && newMinute <= 59) {
      alarmHour = newHour;
      alarmMinute = newMinute;
      server.send(200, "text/html", "<p>Alarm set to " + String(alarmHour) + ":" +
                  (alarmMinute < 10 ? "0" : "") + String(alarmMinute) + "</p><a href='/'>Back</a>");
    } else {
      server.send(400, "text/html", "<p>Invalid values.</p><a href='/'>Back</a>");
    }
  } else {
    server.send(400, "text/html", "<p>Missing parameters.</p><a href='/'>Back</a>");
  }
}

// ----------- Setup & Loop -----------

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Light_Control");
  connectToWiFi();

  pinMode(servo18Pin, OUTPUT);
  pinMode(servo5Pin, OUTPUT);

  timeClient.begin();

  // Setup web routes
  server.on("/", handleRoot);
  server.on("/setAlarm", handleSetAlarm);
  server.on("/control", handleControl);
  server.begin();

  Serial.println("System Ready!");
}

void loop() {
  // WiFi reconnect check
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
    delay(10000); // prevent hammering
  }

  // Print IP every 30s
  if (millis() - lastIPPrint >= 30000) {
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    lastIPPrint = millis();
  }

  // Check alarm every 1s
  if (millis() - lastTimeCheck >= 1000) {
    checkTimeAndControlLights();
    lastTimeCheck = millis();
  }

  // Handle Bluetooth
  if (SerialBT.available()) {
    int command = SerialBT.parseInt();
    handleCommand(command, true);
  }

  // Handle Web requests
  server.handleClient();
}
