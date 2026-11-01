#include <Arduino.h>
#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("POWER_TEST_START");
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  const bool ok = WiFi.softAP("CAS-Power-Test", "CASpower01", 11, false, 1);
  Serial.printf("POWER_TEST_AP=%s IP=%s\n", ok ? "OK" : "FAILED", WiFi.softAPIP().toString().c_str());
}

void loop() {
  Serial.printf("POWER_TEST_ALIVE ms=%lu heap=%lu clients=%d\n",
                (unsigned long)millis(), (unsigned long)ESP.getFreeHeap(), WiFi.softAPgetStationNum());
  delay(1000);
}
