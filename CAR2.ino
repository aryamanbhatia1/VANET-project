/* Vehicle ESP32
   - Reads GPS from NEO-6M on UART2 (GPIO16=RX2, GPIO17=TX2)
   - Sends JSON packet (id, lat, lon, speed) via ESP-NOW broadcast
   - Receives alert messages from RSU via esp_now recv callback and prints to Serial
*/

#include <WiFi.h>
#include <esp_now.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>

#define VEHICLE_ID "CAR2"   // CHANGE to "CAR2" for second car
#define SEND_INTERVAL_MS 2000

HardwareSerial gpsSerial(2); // use UART2
TinyGPSPlus gps;
unsigned long lastSend = 0;

// --------------------------- ESP-NOW Receive Callback ---------------------------
// Updated for new ESP-NOW API
void onReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  const uint8_t *mac = info->src_addr; // MAC from info struct
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  char msg[200];
  int l = min(len, (int)sizeof(msg)-1);
  memcpy(msg, incomingData, l);
  msg[l] = '\0';

  Serial.printf("ALERT RECEIVED from %s: %s\n", macStr, msg);
}

// --------------------------- Send GPS Position ---------------------------
void sendPosition() {
  if (!gps.location.isValid()) return; // skip if no fix

  StaticJsonDocument<200> doc;
  doc["id"] = VEHICLE_ID;
  doc["lat"] = gps.location.lat();
  doc["lon"] = gps.location.lng();
  doc["speed"] = gps.speed.kmph();

  char buf[200];
  size_t n = serializeJson(doc, buf);

  esp_err_t res = esp_now_send(NULL, (uint8_t*)buf, n); // broadcast
  if (res == ESP_OK) {
    Serial.printf("Sent pos %s: %.6f, %.6f, spd=%.1f\n", VEHICLE_ID,
                  gps.location.lat(), gps.location.lng(), gps.speed.kmph());
  } else {
    Serial.printf("esp_now_send failed: %d\n", res);
  }
}

// --------------------------- Setup ---------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
  Serial.println("Vehicle starting...");

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onReceive);  // ✅ new function signature used
  Serial.println("ESP-NOW initialized. Waiting for GPS fix...");
}

// --------------------------- Loop ---------------------------
void loop() {
  while (gpsSerial.available()) gps.encode(gpsSerial.read());
  if (millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis();
    sendPosition();
  }
  delay(10);
}
