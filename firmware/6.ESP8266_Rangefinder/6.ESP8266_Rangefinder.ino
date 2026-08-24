#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// WiFi hotspot configuration
const char* ssid = "ESP8266-Distance-Meter";
const char* password = "12345678";

// Using RX/TX pins (GPIO1 and GPIO3)
#define TRIG_PIN 1   // GPIO1 (TX pin)
#define ECHO_PIN 3   // GPIO3 (RX pin)

// Web server
ESP8266WebServer server(80);

// Distance variables
float distance_cm = 0.0;
unsigned long lastMeasurement = 0;
const unsigned long MEASURE_INTERVAL = 100;
bool measurementError = false;
unsigned long measurementCount = 0;

// Ultrasonic measurement function
float measureDistance() {
  // Send trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Measure echo time
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
  
  if (duration == 0) {
    return -1.0;
  }
  
  // Calculate distance
  float distance = duration * 0.0343 / 2;
  
  // Valid range check
  if (distance > 400.0 || distance < 2.0) {
    return -1.0;
  }
  
  return distance;
}

// Clean HTML page - white background, black text, only distance
const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Distance Meter</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: 'Courier New', monospace;
            background: white;
            min-height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
        }
        
        .container {
            text-align: center;
        }
        
        .distance {
            font-size: 200px;
            font-weight: bold;
            color: black;
            font-family: 'Courier New', monospace;
        }
        
        .unit {
            font-size: 48px;
            color: black;
            margin-left: 10px;
        }
        
        @keyframes blink {
            0% { opacity: 1; }
            50% { opacity: 0.6; }
            100% { opacity: 1; }
        }
        
        .update {
            animation: blink 0.2s ease;
        }
        
        @media (max-width: 600px) {
            .distance {
                font-size: 100px;
            }
            .unit {
                font-size: 32px;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="distance">
            <span id="value">0.0</span><span class="unit">cm</span>
        </div>
    </div>

    <script>
        function updateDistance() {
            fetch('/data')
                .then(response => response.json())
                .then(data => {
                    const valueSpan = document.getElementById('value');
                    
                    if (data.error) {
                        valueSpan.innerHTML = '--';
                    } else {
                        valueSpan.innerHTML = data.distance.toFixed(1);
                        
                        // Add animation effect
                        valueSpan.classList.add('update');
                        setTimeout(() => valueSpan.classList.remove('update'), 200);
                    }
                })
                .catch(error => {
                    document.getElementById('value').innerHTML = '--';
                });
        }
        
        // Update every 300ms
        setInterval(updateDistance, 300);
        updateDistance();
    </script>
</body>
</html>
)rawliteral";

// Handle root path
void handleRoot() {
  server.send(200, "text/html", htmlPage);
}

// Handle data request
void handleData() {
  String json = "{";
  
  if (measurementError || distance_cm < 0) {
    json += "\"error\":\"Out of range\"";
    json += ",\"distance\":0";
  } else {
    json += "\"error\":null";
    json += ",\"distance\":" + String(distance_cm, 2);
  }
  
  json += ",\"count\":" + String(measurementCount);
  json += "}";
  
  server.send(200, "application/json", json);
}

// Handle not found
void handleNotFound() {
  server.send(404, "text/plain", "404: Not Found");
}

void setup() {
  // Note: Serial.begin() not called to avoid RX/TX conflict
  
  // Initialize pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  
  // Create WiFi hotspot
  WiFi.mode(WIFI_AP);
  
  // Configure AP parameters
  IPAddress local_ip(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  
  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP(ssid, password);
  
  // Setup web server
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.onNotFound(handleNotFound);
  
  server.begin();
}

void loop() {
  server.handleClient();
  
  // Periodic distance measurement
  unsigned long currentMillis = millis();
  if (currentMillis - lastMeasurement >= MEASURE_INTERVAL) {
    float measuredDistance = measureDistance();
    
    if (measuredDistance > 0) {
      distance_cm = measuredDistance;
      measurementError = false;
      measurementCount++;
    } else {
      measurementError = true;
    }
    
    lastMeasurement = currentMillis;
  }
  
  delay(10);
}