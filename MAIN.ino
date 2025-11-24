/* RSU (Central ESP32)
   - Uses ESP-NOW to receive JSON position packets from vehicle nodes
   - Computes distances and if >= CONGESTION_THRESHOLD vehicles are
     within CONGESTION_DISTANCE_M, broadcasts an alert to all nodes.
*/

#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoJson.h>

#define MAX_VEHICLES 12
#define CONGESTION_DISTANCE_M 30.0   // meters
#define CONGESTION_THRESHOLD 2       // number of vehicles to trigger alert

struct VehicleEntry {
  char id[16];
  double lat;
  double lon;
  unsigned long lastSeen; // millis()
};

VehicleEntry vehicles[MAX_VEHICLES];
int vcount = 0;

// --------------------------- Haversine Function ---------------------------
float haversine(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0; // Earth radius in meters
  double dLat = (lat2 - lat1) * DEG_TO_RAD;
  double dLon = (lon2 - lon1) * DEG_TO_RAD;
  double a = sin(dLat/2.0)*sin(dLat/2.0) +
             cos(lat1*DEG_TO_RAD)*cos(lat2*DEG_TO_RAD) *
             sin(dLon/2.0)*sin(dLon/2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return R * c;
}

// --------------------------- Send Alert ---------------------------
void sendBroadcastAlert(const char* txt) {
  esp_err_t res = esp_now_send(NULL, (uint8_t*)txt, strlen(txt));
  if (res == ESP_OK) {
    Serial.println("Alert broadcasted: " + String(txt));
  } else {
    Serial.println("Alert send failed: " + String(res));
  }
}

// --------------------------- ESP-NOW Receive Callback ---------------------------
// UPDATED to new ESP-NOW API format
void onReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  const uint8_t *mac = info->src_addr; // Get MAC from struct
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  char jsonBuf[256];
  int copyLen = min(len, (int)sizeof(jsonBuf)-1);
  memcpy(jsonBuf, incomingData, copyLen);
  jsonBuf[copyLen] = '\0';

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, jsonBuf);
  if (err) {
    Serial.println("JSON parse error");
    return;
  }

  const char* vid = doc["id"];
  double lat = doc["lat"];
  double lon = doc["lon"];
  double spd = doc["speed"]; // optional

  Serial.printf("RSU got from %s (%s): %.6f , %.6f  speed: %.1f\n", vid, macStr, lat, lon, spd);

  // update or add vehicle entry
  bool updated = false;
  for (int i = 0; i < vcount; i++) {
    if (strcmp(vehicles[i].id, vid) == 0) {
      vehicles[i].lat = lat;
      vehicles[i].lon = lon;
      vehicles[i].lastSeen = millis();
      updated = true;
      break;
    }
  }

  if (!updated && vcount < MAX_VEHICLES) {
    strncpy(vehicles[vcount].id, vid, sizeof(vehicles[vcount].id) - 1);
    vehicles[vcount].id[sizeof(vehicles[vcount].id) - 1] = '\0';
    vehicles[vcount].lat = lat;
    vehicles[vcount].lon = lon;
    vehicles[vcount].lastSeen = millis();
    vcount++;
  }

  // prune old entries (>10s)
  unsigned long now = millis();
  for (int i = 0; i < vcount; i++) {
    if (now - vehicles[i].lastSeen > 10000UL) {
      for (int j = i; j < vcount - 1; j++) vehicles[j] = vehicles[j + 1];
      vcount--;
      i--;
    }
  }

  // check pairwise distances
  for (int i = 0; i < vcount; i++) {
    int nearby = 1;
    for (int j = 0; j < vcount; j++) {
      if (i == j) continue;
      double d = haversine(vehicles[i].lat, vehicles[i].lon, vehicles[j].lat, vehicles[j].lon);
      if (d <= CONGESTION_DISTANCE_M) nearby++;
    }
    if (nearby >= CONGESTION_THRESHOLD) {
      char alertMsg[128];
      snprintf(alertMsg, sizeof(alertMsg), "CONGESTION_NEAR_%s;Take_Alternate_Route", vehicles[i].id);
      sendBroadcastAlert(alertMsg);
      break;
    }
  }
}

// --------------------------- Setup ---------------------------
void setup() {
  Serial.begin(115200);
  delay(100);

  WiFi.mode(WIFI_STA); // required before esp_now_init
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onReceive); // new API compatible
  Serial.println("RSU started and waiting for vehicle packets...");
}

// --------------------------- Loop ---------------------------
void loop() {
  delay(200);
}
