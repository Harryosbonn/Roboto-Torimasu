/*
 * ESP32 GPS Beacon
 * Broadcasts GPS coordinates via WiFi UDP for robot person-following
 * 
 * Wiring:
 *   ESP32 GPIO16 (RX2) <- GPS TX
 *   ESP32 GPIO17 (TX2) -> GPS RX
 *   ESP32 3.3V -> GPS VCC
 *   ESP32 GND -> GPS GND
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <TinyGPS++.h>

// === CONFIGURATION ===
const char* WIFI_SSID = "YOUR_WIFI_SSID";      // Change this!
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";  // Change this!
const char* ROBOT_IP = "10.86.157.230";        // Robot's IP address
const int UDP_PORT = 5005;

// GPS Serial
#define GPS_RX 16
#define GPS_TX 17
HardwareSerial GPSSerial(2);

// Objects
TinyGPSPlus gps;
WiFiUDP udp;

// Timing
unsigned long lastBroadcast = 0;
const int BROADCAST_INTERVAL_MS = 500; // Send position twice per second

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 GPS Beacon ===");
  
  // Init GPS Serial
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS Serial initialized");
  
  // Connect to WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! IP: ");
  Serial.println(WiFi.localIP());
  
  // Start UDP
  udp.begin(UDP_PORT);
  Serial.println("UDP beacon ready");
}

void loop() {
  // Read GPS data
  while (GPSSerial.available() > 0) {
    char c = GPSSerial.read();
    gps.encode(c);
  }
  
  // Broadcast position at interval
  if (millis() - lastBroadcast > BROADCAST_INTERVAL_MS) {
    lastBroadcast = millis();
    broadcastPosition();
  }
}

void broadcastPosition() {
  // Build JSON packet
  String json = "{";
  json += "\"type\":\"beacon\",";
  
  if (gps.location.isValid()) {
    json += "\"lat\":" + String(gps.location.lat(), 6) + ",";
    json += "\"lon\":" + String(gps.location.lng(), 6) + ",";
    json += "\"alt\":" + String(gps.altitude.meters(), 1) + ",";
    json += "\"hdop\":" + String(gps.hdop.hdop(), 2) + ",";
    json += "\"sats\":" + String(gps.satellites.value()) + ",";
    json += "\"valid\":true";
  } else {
    json += "\"lat\":0,\"lon\":0,\"alt\":0,\"hdop\":99,\"sats\":0,";
    json += "\"valid\":false";
  }
  
  json += "}";
  
  // Send UDP packet to robot
  udp.beginPacket(ROBOT_IP, UDP_PORT);
  udp.print(json);
  udp.endPacket();
  
  // Debug output
  Serial.print("TX: ");
  Serial.println(json);
}
