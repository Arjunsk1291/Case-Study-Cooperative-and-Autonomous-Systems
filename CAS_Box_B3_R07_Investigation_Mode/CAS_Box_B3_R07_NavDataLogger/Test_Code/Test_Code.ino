#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>

const char* ssid = "Box Test";
const char* password = "12345678";

WebServer server(80);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_SSD1306 display(128, 32, &Wire, -1);
TinyGPSPlus gps;

const int trigPins[] = {23, 32, 5, 4, 13};
const int echoPins[] = {26, 25, 14, 18, 19};
const char* usNames[] = {"45_Deg", "Front", "Back", "Left", "Right"};

// Ultrasonic diagnostic data. Raw = latest attempt; filtered = median of up to 5 valid echoes.
static const uint8_t US_COUNT = 5;
static const uint8_t US_WINDOW = 5;
static const uint32_t US_QUIET_GAP_MS = 25;
int usRaw[US_COUNT] = {-1, -1, -1, -1, -1};
int usFiltered[US_COUNT] = {-1, -1, -1, -1, -1};
bool usValid[US_COUNT] = {false, false, false, false, false};
uint32_t usLastGoodMs[US_COUNT] = {0, 0, 0, 0, 0};
uint32_t usTimeouts[US_COUNT] = {0, 0, 0, 0, 0};
int usHistory[US_COUNT][US_WINDOW] = {};
uint8_t usHistoryCount[US_COUNT] = {0, 0, 0, 0, 0};
uint8_t usHistoryNext[US_COUNT] = {0, 0, 0, 0, 0};
int usIndex = 0;
uint32_t usLastPingMs = 0;

float imuX = 0, imuY = 0, imuZ = 0;
bool lidarActive = false;
int lidarBytes = 0;

int readUS(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(3);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  const unsigned long duration = pulseIn(echo, HIGH, 15000);
  if (duration == 0) return -1;
  const int cm = static_cast<int>((duration + 29) / 58);
  return (cm > 0 && cm <= 300) ? cm : -1;
}

int medianUS(uint8_t i) {
  const uint8_t n = usHistoryCount[i];
  if (!n) return -1;
  int sorted[US_WINDOW];
  for (uint8_t k = 0; k < n; ++k) sorted[k] = usHistory[i][k];
  for (uint8_t k = 1; k < n; ++k) {
    const int value = sorted[k];
    int j = k - 1;
    while (j >= 0 && sorted[j] > value) { sorted[j + 1] = sorted[j]; --j; }
    sorted[j + 1] = value;
  }
  return sorted[n / 2];
}

void updateUS(uint8_t i) {
  const int cm = readUS(trigPins[i], echoPins[i]);
  usRaw[i] = cm;
  usValid[i] = cm >= 0;
  if (cm >= 0) {
    usHistory[i][usHistoryNext[i]] = cm;
    usHistoryNext[i] = (usHistoryNext[i] + 1) % US_WINDOW;
    if (usHistoryCount[i] < US_WINDOW) ++usHistoryCount[i];
    usFiltered[i] = medianUS(i);
    usLastGoodMs[i] = millis();
  } else {
    ++usTimeouts[i];
  }
  usLastPingMs = millis();
}

const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><style>
body{font-family:sans-serif;background:#111;color:#eee;padding:15px;}
h2{color:#4fc3f7;}
p{margin:6px 0;}
b{color:#aaa;}
table{border-collapse:collapse;width:100%;font-size:14px;}td,th{text-align:left;padding:7px 4px;border-bottom:1px solid #333;}th{color:#aaa;font-weight:600}.ok{color:#70e0a0}.warn{color:#ffbd69}.small{font-size:11px;color:#999}
</style></head><body>
<h2>Sensor Diagnostics</h2>
<p><b>BNO055 IMU:</b> X: <span id="ix">-</span> Y: <span id="iy">-</span> Z: <span id="iz">-</span></p>
<p><b>GPS:</b> <span id="gps">-</span></p>
<p><b>LiDAR:</b> <span id="lidar">-</span></p>
<p><b>Ultrasonic diagnostics:</b> raw = latest ping; median = median of recent valid pings.</p>
<table><thead><tr><th>Sensor</th><th>Raw</th><th>Median</th><th>Status</th></tr></thead><tbody id="us"><tr><td colspan="4">Waiting for readings…</td></tr></tbody></table>
<script>
function poll() {
  fetch('/data').then(r => r.json()).then(d => {
    document.getElementById('ix').innerText = d.ix.toFixed(1);
    document.getElementById('iy').innerText = d.iy.toFixed(1);
    document.getElementById('iz').innerText = d.iz.toFixed(1);
    document.getElementById('gps').innerText = d.gpsValid ? (d.lat.toFixed(6) + ', ' + d.lng.toFixed(6)) : 'Waiting for fix';
    document.getElementById('lidar').innerText = d.lidar ? ('UART Active (' + d.lidarBytes + ' bytes)') : 'No UART data';
    let usHtml = '';
    for (let i = 0; i < d.usRaw.length; i++) {
      const raw = d.usValid[i] ? d.usRaw[i] + ' cm' : 'NO ECHO';
      const med = d.usFiltered[i] < 0 ? '-' : d.usFiltered[i] + ' cm';
      const age = d.usAgeMs[i] < 0 ? 'never' : (d.usAgeMs[i] < 1000 ? d.usAgeMs[i] + ' ms' : (d.usAgeMs[i] / 1000).toFixed(1) + ' s');
      const status = d.usValid[i] ? '<span class="ok">Fresh</span>' : '<span class="warn">Timeout</span>';
      usHtml += '<tr><td>' + d.names[i] + '</td><td>' + raw + '</td><td>' + med + '</td><td>' + status + '<div class="small">last good: ' + age + ' · timeouts: ' + d.usTimeouts[i] + '</div></td></tr>';
    }
    document.getElementById('us').innerHTML = usHtml;
  }).finally(() => setTimeout(poll, 100));
}
poll();
</script>
</body></html>
)rawliteral";

void handleData() {
  String json = "{";
  json += "\"ix\":" + String(imuX) + ",\"iy\":" + String(imuY) + ",\"iz\":" + String(imuZ) + ",";
  json += "\"gpsValid\":" + String(gps.location.isValid() ? 1 : 0) + ",";
  json += "\"lat\":" + String(gps.location.lat(), 6) + ",\"lng\":" + String(gps.location.lng(), 6) + ",";
  json += "\"lidar\":" + String(lidarActive ? 1 : 0) + ",\"lidarBytes\":" + String(lidarBytes) + ",";
  json += "\"usRaw\":[";
  for (int i = 0; i < US_COUNT; i++) { if (i) json += ","; json += String(usRaw[i]); }
  json += "],\"usFiltered\":[";
  for (int i = 0; i < US_COUNT; i++) { if (i) json += ","; json += String(usFiltered[i]); }
  json += "],\"usValid\":[";
  for (int i = 0; i < US_COUNT; i++) { if (i) json += ","; json += usValid[i] ? "true" : "false"; }
  json += "],\"usAgeMs\":[";
  for (int i = 0; i < US_COUNT; i++) { if (i) json += ","; json += (usLastGoodMs[i] == 0) ? String(-1) : String(millis() - usLastGoodMs[i]); }
  json += "],\"usTimeouts\":[";
  for (int i = 0; i < US_COUNT; i++) { if (i) json += ","; json += String(usTimeouts[i]); }
  json += "],\"names\":[";
  for (int i = 0; i < 5; i++) {
    json += "\"" + String(usNames[i]) + "\"";
    if (i < 4) json += ",";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 27, 33);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);

  for (int i = 0; i < 5; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  Wire.begin(21, 22);
  bno.begin();

  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("Sensors Init OK");
    display.println("Connect to WiFi");
    display.display();
  }

  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();

  Serial.println();
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("Password: ");
  Serial.println(password);
  Serial.print("URL: http://");
  Serial.println(IP);

  server.on("/", []() { server.send_P(200, "text/html", webpage); });
  server.on("/data", handleData);
  server.begin();
}

void loop() {
  server.handleClient();

  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }

  // IMU updated every loop (fast, non-blocking)
  sensors_event_t event;
  bno.getEvent(&event);
  imuX = event.orientation.x;
  imuY = event.orientation.y;
  imuZ = event.orientation.z;

  // LiDAR check
  lidarBytes = Serial1.available();
  lidarActive = lidarBytes > 0;
  while (Serial1.available()) Serial1.read();

  // Round-robin one sensor per loop pass. Quiet gap reduces cross-talk between modules.
  if (millis() - usLastPingMs >= US_QUIET_GAP_MS) {
    updateUS(usIndex);
    usIndex = (usIndex + 1) % US_COUNT;
  }
}