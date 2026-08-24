#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <IRrecv.h>
#include <IRutils.h>

#define IR_RECEIVE_PIN D3

String lastKey = "";
String currentKey = "";

const char* apSSID = "ESP8266_IR_Display";
const char* apPassword = "12345678";
const IPAddress apIP(192, 168, 4, 1);

ESP8266WebServer server(80);
IRrecv irrecv(IR_RECEIVE_PIN);
decode_results results;

String keyMap(uint32_t code) {
  switch(code) {
    case 0x16: return "1";
    case 0x19: return "2";
    case 0x0D: return "3";
    case 0x0C: return "4";
    case 0x18: return "5";
    case 0x5E: return "6";
    case 0x08: return "7";
    case 0x1C: return "8";
    case 0x5A: return "9";
    case 0x52: return "0";
    case 0x42: return "*";
    case 0x4A: return "#";
    case 0x46: return "UP";
    case 0x15: return "DOWN";
    case 0x40: return "OK";
    case 0x44: return "LEFT";
    case 0x43: return "RIGHT";
    default: return "";
  }
}

String htmlPage() {
  return R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>IR Remote</title>
<style>
  *{margin:0;padding:0;box-sizing:border-box;}
  body{background:#fff;font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;min-height:100vh;display:flex;justify-content:center;align-items:center;padding:20px;}
  .card{max-width:500px;width:100%;text-align:center;}
  h1{color:#000;font-size:24px;font-weight:500;margin-bottom:40px;padding-bottom:15px;border-bottom:2px solid #000;display:inline-block;}
  .display{background:#fff;border:2px solid #000;border-radius:20px;padding:60px 20px;margin-bottom:30px;}
  .key{font-size:140px;color:#000;font-weight:600;line-height:1;}
  .hint{color:#666;font-size:14px;margin-top:20px;padding-top:15px;border-top:1px solid #eee;}
  .status{display:inline-block;padding:5px 12px;background:#f5f5f5;color:#333;font-size:12px;border-radius:20px;margin-top:15px;}
  @media(max-width:480px){.key{font-size:100px;}.display{padding:40px 20px;}}
</style>
</head>
<body>
<div class="card">
<h1>IR Remote Display</h1>
<div class="display">
<div class="key" id="keyValue">—</div>
</div>
<div class="hint">
<div>Point remote at receiver</div>
<div class="status">● Ready</div>
</div>
</div>
<script>
function fetchKey(){
  fetch('/key').then(r=>r.text()).then(d=>{
    if(d&&d!=='-')document.getElementById('keyValue').innerText=d;
  }).catch(e=>console.error(e));
}
setInterval(fetchKey,200);
fetchKey();
</script>
</body>
</html>
)rawliteral";
}

void handleRoot() { server.send(200, "text/html", htmlPage()); }
void handleKey() { server.send(200, "text/plain", currentKey.length() > 0 ? currentKey : "-"); }

void setup() {
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(apSSID, apPassword);
  
  irrecv.enableIRIn();
  
  server.on("/", handleRoot);
  server.on("/key", handleKey);
  server.begin();
}

void loop() {
  server.handleClient();
  
  if (irrecv.decode(&results)) {
    String key = keyMap(results.command);
    if (key == "") key = keyMap(results.value & 0xFF);
    
    if (key != "" && key != lastKey) {
      currentKey = key;
      lastKey = key;
      delay(50);
    }
    irrecv.resume();
  }
}