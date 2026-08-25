#include <EEPROM.h>
#include "webserver.h"

const char* password = "12345678";

int Servo_PROGRAM = 0;
int GPIO_ID = 100;
String ival;

ESP8266WebServer server(80);

const RobotDeviceInfo kRobotDeviceInfo = {
    "QuadBot-E",
    "1.0.0",
    "v1"
};

static String g_deviceName = "Robot";
static int g_currentActionId = PROGRAM_NONE;
static bool g_robotBusy = false;
static bool g_stopRequested = false;
static int g_stopTargetProgram = PROGRAM_STANDBY;
static bool g_debugUiEnabled = false;

static const RobotActionInfo kRobotActions[] = {
    {PROGRAM_STANDBY, "standby", "待机"},
    {PROGRAM_FORWARD, "forward", "前进"},
    {PROGRAM_BACKWARD, "backward", "后退"},
    {PROGRAM_LEFTSHIFT, "leftshift", "左移"},
    {PROGRAM_RIGHTSHIFT, "rightshift", "右移"},
    {PROGRAM_TURNLEFT, "turnleft", "左转"},
    {PROGRAM_TURNRIGHT, "turnright", "右转"},
    {PROGRAM_LIE, "lie", "躺下"},
    {PROGRAM_HELLO, "hello", "打招呼"},
    {PROGRAM_FIGHTING, "fighting", "战斗"},
    {PROGRAM_PUSHUP, "pushup", "俯卧撑"},
    {PROGRAM_SLEEP, "sleep", "睡眠"},
    {PROGRAM_DANCE1, "dance1", "舞步1"},
    {PROGRAM_DANCE2, "dance2", "舞步2"},
    {PROGRAM_DANCE3, "dance3", "舞步3"},
    {PROGRAM_SIT, "sit", "坐下"},
    {PROGRAM_BOW, "bow", "鞠躬"},
    {PROGRAM_SHAKE, "shake", "摇摆"},
    {PROGRAM_AVOID, "avoid", "避障"},
    {PROGRAM_CENTER, "center", "中位"},
    {PROGRAM_ZERO, "zero", "归零"}
};

// ── Single-page UI stored in Flash (PROGMEM) ─────────────────────────────────
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>QuadBot Debug</title>
<style>
:root{
  --bg:#090c12;--panel:#10141d;--panel-2:#141a26;--border:#212939;--border-soft:#1a212e;
  --text:#e6ebf2;--text-dim:#8492a6;--text-faint:#576073;
  --cyan:#2dd4ee;--cyan-dim:#0e2530;
  --amber:#f5b942;--amber-dim:#2e2411;
  --violet:#b18cf5;--violet-dim:#231a35;
  --red:#f2685c;--red-dim:#2c1416;
  --green:#3ddc97;
  --mono:ui-monospace,"Cascadia Code","Segoe UI Mono",Consolas,monospace;
  --sans:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Arial,sans-serif;
}
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
body{font-family:var(--sans);background:var(--bg);color:var(--text);min-height:100vh;overflow:auto;background-image:radial-gradient(circle at 15% 0%,#101825 0%,var(--bg) 45%)}
.page{min-height:100vh;display:flex;flex-direction:column}
.bar{position:sticky;top:0;z-index:10;display:flex;align-items:center;justify-content:space-between;gap:8px;padding:14px 16px;background:rgba(9,12,18,.92);backdrop-filter:blur(6px);border-bottom:1px solid var(--border)}
.brand{display:flex;align-items:center;gap:9px}
.brand-dot{width:8px;height:8px;border-radius:50%;background:var(--green);box-shadow:0 0 8px var(--green)}
.title{font-size:1rem;font-weight:700;letter-spacing:.01em}
.title small{display:block;font-size:.68rem;font-weight:600;color:var(--text-faint);letter-spacing:.08em;text-transform:uppercase;margin-top:1px}
.bar-actions{display:flex;gap:8px}
.tool-btn{border:1px solid var(--border);border-radius:9px;padding:8px 12px;font-size:.8rem;font-weight:600;cursor:pointer;color:var(--text);background:var(--panel-2);touch-action:manipulation}
.tool-btn.alt{color:var(--text-dim)}
.main{flex:1;display:flex;flex-direction:column;gap:12px;padding:14px;padding-bottom:max(20px,env(safe-area-inset-bottom));max-width:520px;margin:0 auto;width:100%}
.panel{background:var(--panel);border:1px solid var(--border-soft);border-radius:16px;padding:14px}
.panel-title{font-size:.72rem;font-weight:700;color:var(--text-faint);margin-bottom:10px;text-transform:uppercase;letter-spacing:.09em}
.grid{display:grid;gap:9px}
.grid3{grid-template-columns:repeat(3,minmax(0,1fr))}
.btn{min-height:54px;width:100%;border:1px solid var(--border);border-radius:12px;font-size:.86rem;font-weight:600;cursor:pointer;color:var(--text);background:var(--panel-2);touch-action:manipulation;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:2px;transition:transform .08s ease,background .12s ease}
.btn:active{transform:scale(.96);background:#1b2331}
.btn .glyph{font-size:1.05rem;line-height:1;font-family:var(--mono)}
.btn.on-cyan{color:var(--cyan)}
.btn.on-amber{color:var(--amber)}
.btn.on-violet{color:var(--violet)}
.btn.on-red{color:var(--red);background:var(--red-dim);border-color:rgba(242,104,92,.35)}
.dpad{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:9px}
.dpad .fwd{grid-column:2}
.dpad .bwd{grid-column:2}
.status{display:flex;align-items:center;justify-content:space-between;padding:13px 16px}
.status-left{display:flex;align-items:center;gap:10px}
.dot{width:9px;height:9px;border-radius:50%;background:var(--text-faint);flex-shrink:0}
.dot.busy{background:var(--amber);animation:pulse 1s ease-in-out infinite}
@keyframes pulse{0%,100%{box-shadow:0 0 0 0 rgba(245,185,66,.55)}50%{box-shadow:0 0 0 5px rgba(245,185,66,0)}}
.status-label{font-size:.68rem;font-weight:700;color:var(--text-faint);text-transform:uppercase;letter-spacing:.08em}
.status-name{font-family:var(--mono);font-size:.92rem;font-weight:600;color:var(--text);margin-top:1px}
@media (prefers-reduced-motion:reduce){.dot.busy{animation:none}}
.overlay{position:fixed;inset:0;z-index:30;display:flex;align-items:flex-end;justify-content:center;background:rgba(4,6,10,.78);opacity:0;pointer-events:none;transition:opacity .16s ease}
.overlay.show{opacity:1;pointer-events:auto}
.sheet{width:min(560px,100%);max-height:88vh;background:var(--panel);border-radius:18px 18px 0 0;border:1px solid var(--border);border-bottom:none;display:flex;flex-direction:column}
.sheet-head{display:flex;align-items:center;justify-content:space-between;padding:14px 16px;border-bottom:1px solid var(--border-soft)}
.sheet-title{font-size:.9rem;font-weight:700}
.sheet-body{padding:14px;overflow:auto}
.close-btn{border:none;background:none;color:var(--text-dim);font-size:1.4rem;line-height:1;cursor:pointer}
table{width:100%;border-collapse:collapse;font-size:.82rem}
th,td{padding:8px 6px;text-align:center;border-bottom:1px solid var(--border-soft)}
th{color:var(--text-faint);font-weight:600;font-size:.72rem;text-transform:uppercase;letter-spacing:.05em}
.group td{padding-top:14px;padding-bottom:6px;text-align:left;color:var(--cyan);font-size:.74rem;font-weight:700;border-bottom:none;text-transform:uppercase;letter-spacing:.06em}
input[type=number]{width:64px;padding:6px 4px;border-radius:8px;border:1px solid var(--border);background:var(--bg);color:var(--text);text-align:center;font-family:var(--mono)}
.set-btn{border:1px solid #1e4633;border-radius:8px;background:var(--green);color:#052e1c;padding:7px 11px;font-size:.76rem;font-weight:700;cursor:pointer}
.wide-btn{width:100%;margin-top:14px;padding:12px;border:1px solid var(--border);border-radius:10px;background:var(--panel-2);color:var(--text);font-size:.86rem;font-weight:600;cursor:pointer}
.hint{font-size:.76rem;color:var(--text-faint);margin-top:9px;line-height:1.5}
.prog-row{display:flex;gap:8px;margin-bottom:10px}
.prog-select{flex:1;min-width:0;padding:9px 8px;border-radius:9px;border:1px solid var(--border);background:var(--bg);color:var(--text);font-size:.84rem}
.prog-pause{width:70px;padding:9px 6px;border-radius:9px;border:1px solid var(--border);background:var(--bg);color:var(--text);font-family:var(--mono);text-align:center}
.prog-list{display:flex;flex-direction:column;gap:6px;margin-bottom:10px;max-height:240px;overflow:auto}
.prog-step{display:flex;align-items:center;gap:8px;padding:8px 10px;background:var(--panel-2);border:1px solid var(--border-soft);border-radius:9px;font-size:.84rem}
.prog-step.active{border-color:var(--cyan);background:var(--cyan-dim)}
.prog-step .idx{color:var(--text-faint);font-family:var(--mono);font-size:.78rem;width:16px;flex-shrink:0}
.prog-step .name{flex:1}
.prog-step .pause{color:var(--text-faint);font-family:var(--mono);font-size:.78rem;flex-shrink:0}
.prog-step .rm{border:none;background:none;color:var(--text-faint);font-size:1.15rem;cursor:pointer;line-height:1;padding:0 2px;flex-shrink:0}
.prog-empty{color:var(--text-faint);font-size:.82rem;padding:6px 2px}
.prog-controls{display:grid;grid-template-columns:1fr 1fr auto;gap:8px}
</style>
</head>
<body>
<div class="page">
  <div class="bar">
    <div class="brand">
      <div class="brand-dot"></div>
      <div class="title">QuadBot<small>Control Console</small></div>
    </div>
    <div class="bar-actions">
      <button class="tool-btn" onclick="openCal()">Calibration</button>
      <button class="tool-btn alt" onclick="hideDebug()">Hide</button>
    </div>
  </div>

  <div class="main">
    <div class="panel status">
      <div class="status-left">
        <div id="statusDot" class="dot"></div>
        <div>
          <div class="status-label">Status</div>
          <div id="statusName" class="status-name">--</div>
        </div>
      </div>
    </div>

    <div class="panel">
      <div class="panel-title">Movement</div>
      <div class="dpad">
        <button class="btn on-cyan" onclick="pm(6)"><span class="glyph">&#8630;</span>Turn Left</button>
        <button class="btn on-cyan fwd" onclick="pm(2)"><span class="glyph">&#9650;</span>Forward</button>
        <button class="btn on-cyan" onclick="pm(7)"><span class="glyph">&#8631;</span>Turn Right</button>
        <button class="btn on-cyan" onclick="pm(4)"><span class="glyph">&#9664;</span>Left Shift</button>
        <button class="btn on-cyan bwd" onclick="pm(3)"><span class="glyph">&#9660;</span>Backward</button>
        <button class="btn on-cyan" onclick="pm(5)"><span class="glyph">&#9654;</span>Right Shift</button>
      </div>
    </div>

    <div class="panel">
      <div class="panel-title">Actions</div>
      <div class="grid grid3">
        <button class="btn on-amber" onclick="pm(1)">Standby</button>
        <button class="btn on-amber" onclick="pm(9)">Hello</button>
        <button class="btn on-amber" onclick="pm(11)">Push Up</button>
        <button class="btn on-amber" onclick="pm(8)">Lie</button>
        <button class="btn on-amber" onclick="pm(10)">Fighting</button>
        <button class="btn on-amber" onclick="pm(12)">Sleep</button>
        <button class="btn on-violet" onclick="pm(13)">Dance 1</button>
        <button class="btn on-violet" onclick="pm(14)">Dance 2</button>
        <button class="btn on-violet" onclick="pm(15)">Dance 3</button>
      </div>
    </div>

    <div class="panel">
      <div class="panel-title">Personality</div>
      <div class="grid grid3">
        <button class="btn on-violet" onclick="pm(16)">Sit</button>
        <button class="btn on-violet" onclick="pm(17)">Bow</button>
        <button class="btn on-violet" onclick="pm(18)">Shake</button>
      </div>
    </div>

    <div class="panel">
      <div class="panel-title">Autonomy</div>
      <div class="grid grid3">
        <button class="btn on-cyan" onclick="pm(21)">Avoid Mode</button>
        <button class="btn on-red" onclick="stopRobot()">Stop</button>
      </div>
      <div class="hint">Avoid Mode needs the firmware built with ENABLE_ULTRASONIC=1, which permanently repurposes the USB serial pins for the sensor -- calibrate.py/send_action.py/live_mirror.py won't work at all on that build, not just while this is running. Reflash with ENABLE_ULTRASONIC=0 to get serial tools back.</div>
    </div>

    <div class="panel">
      <div class="panel-title">Movement Programmer</div>
      <div class="prog-row">
        <select id="progAction" class="prog-select"></select>
        <input type="number" id="progPause" class="prog-pause" min="0" max="5000" step="100" value="300">
        <button class="tool-btn" onclick="addStep()">Add</button>
      </div>
      <div id="progList" class="prog-list"></div>
      <div class="prog-controls">
        <button class="btn on-cyan" id="progPlayBtn" onclick="playRoutine()">Play</button>
        <button class="btn on-red" onclick="stopRoutine()">Stop</button>
        <button class="tool-btn alt" onclick="clearRoutine()">Clear</button>
      </div>
      <div class="hint">Chains existing moves into a sequence. Pause is how long to wait, after each move actually finishes, before starting the next. Saved automatically in this browser.</div>
    </div>
  </div>
</div>

<div id="overlay" class="overlay" onclick="bgClose(event)">
  <div class="sheet">
    <div class="sheet-head">
      <div class="sheet-title">Calibration</div>
      <button class="close-btn" onclick="closeCal()">&times;</button>
    </div>
    <div class="sheet-body">
      <table>
        <thead>
          <tr><th>Pin</th><th>Part</th><th>Axis</th><th>Offset</th><th></th></tr>
        </thead>
        <tbody>
          <tr class="group"><td colspan="5">Right Front</td></tr>
          <tr><td>D14</td><td>Foot</td><td>Swing</td><td><input type="number" id="s0" min="-30" max="30" value="0"></td><td><button class="set-btn" onclick="save(0)">SET</button></td></tr>
          <tr><td>D12</td><td>Leg</td><td>Lift</td><td><input type="number" id="s1" min="-30" max="30" value="0"></td><td><button class="set-btn" onclick="save(1)">SET</button></td></tr>

          <tr class="group"><td colspan="5">Right Rear</td></tr>
          <tr><td>D13</td><td>Leg</td><td>Lift</td><td><input type="number" id="s2" min="-30" max="30" value="0"></td><td><button class="set-btn" onclick="save(2)">SET</button></td></tr>
          <tr><td>D15</td><td>Foot</td><td>Swing</td><td><input type="number" id="s3" min="-30" max="30" value="0"></td><td><button class="set-btn" onclick="save(3)">SET</button></td></tr>

          <tr class="group"><td colspan="5">Left Front</td></tr>
          <tr><td>D16</td><td>Foot</td><td>Swing</td><td><input type="number" id="s4" min="-30" max="30" value="0"></td><td><button class="set-btn" onclick="save(4)">SET</button></td></tr>
          <tr><td>D5</td><td>Leg</td><td>Lift</td><td><input type="number" id="s5" min="-30" max="30" value="0"></td><td><button class="set-btn" onclick="save(5)">SET</button></td></tr>

          <tr class="group"><td colspan="5">Left Rear</td></tr>
          <tr><td>D4</td><td>Leg</td><td>Lift</td><td><input type="number" id="s6" min="-30" max="30" value="0"></td><td><button class="set-btn" onclick="save(6)">SET</button></td></tr>
          <tr><td>D2</td><td>Foot</td><td>Swing</td><td><input type="number" id="s7" min="-30" max="30" value="0"></td><td><button class="set-btn" onclick="save(7)">SET</button></td></tr>
        </tbody>
      </table>
      <button class="wide-btn" onclick="pm(100)">Go To Zero Pose</button>
      <div class="hint">Offset range: -30 to +30 is recommended for quick calibration.</div>
    </div>
  </div>
</div>

<script>
function pm(v){ fetch('/controller?pm=' + v, { cache: 'no-store' }); }
function stopRobot(){ fetch('/api/v1/stop', { method: 'POST', cache: 'no-store' }); }
function closeCal(){ document.getElementById('overlay').classList.remove('show'); }
function bgClose(e){ if(e.target.id === 'overlay') closeCal(); }
function save(id){
  var value = document.getElementById('s' + id).value;
  fetch('/save?key=' + id + '&value=' + value, { cache: 'no-store' });
}
function hideDebug(){ window.location = '/debug/close'; }
function openCal(){
  fetch('/setting1', { cache: 'no-store' })
    .then(function(r){ return r.json(); })
    .then(function(values){
      for (var i = 0; i < 8; i++) {
        document.getElementById('s' + i).value = values[i] || 0;
      }
      document.getElementById('overlay').classList.add('show');
    });
}
function pollStatus(){
  fetch('/api/v1/state', { cache: 'no-store' })
    .then(function(r){ return r.json(); })
    .then(function(json){
      var data = json.data;
      if (!data) return;
      document.getElementById('statusName').textContent = data.current_action_label + (data.busy ? ' (moving)' : '');
      document.getElementById('statusDot').classList.toggle('busy', !!data.busy);
    })
    .catch(function(){});
}
pollStatus();
setInterval(pollStatus, 1000);

var ROUTINE_KEY = 'quadbot_routine_v1';
var routineSteps = [];
try { routineSteps = JSON.parse(localStorage.getItem(ROUTINE_KEY) || '[]'); } catch (e) { routineSteps = []; }
var routinePlaying = false;
var routineCancel = false;
var routineCurrentIndex = -1;

function loadActionOptions(){
  fetch('/api/v1/info', { cache: 'no-store' })
    .then(function(r){ return r.json(); })
    .then(function(json){
      var actions = (json.data && json.data.actions) || [];
      var sel = document.getElementById('progAction');
      sel.innerHTML = '';
      actions.forEach(function(a){
        if (a.name === 'center' || a.name === 'zero') return;
        var opt = document.createElement('option');
        opt.value = a.name;
        opt.textContent = a.name;
        sel.appendChild(opt);
      });
    })
    .catch(function(){});
}

function saveRoutine(){ try { localStorage.setItem(ROUTINE_KEY, JSON.stringify(routineSteps)); } catch (e) {} }

function renderRoutine(){
  var list = document.getElementById('progList');
  list.innerHTML = '';
  if (routineSteps.length === 0) {
    list.innerHTML = '<div class="prog-empty">No steps yet -- add one above.</div>';
    return;
  }
  routineSteps.forEach(function(step, i){
    var row = document.createElement('div');
    row.className = 'prog-step' + (routinePlaying && i === routineCurrentIndex ? ' active' : '');
    var idx = document.createElement('span'); idx.className = 'idx'; idx.textContent = (i + 1);
    var name = document.createElement('span'); name.className = 'name'; name.textContent = step.action;
    var pause = document.createElement('span'); pause.className = 'pause'; pause.textContent = '+' + step.pause + 'ms';
    var rm = document.createElement('button'); rm.className = 'rm'; rm.innerHTML = '&times;';
    rm.onclick = function(){ removeStep(i); };
    row.appendChild(idx); row.appendChild(name); row.appendChild(pause); row.appendChild(rm);
    list.appendChild(row);
  });
}

function addStep(){
  var action = document.getElementById('progAction').value;
  var pause = parseInt(document.getElementById('progPause').value, 10) || 0;
  if (!action) return;
  routineSteps.push({ action: action, pause: pause });
  saveRoutine();
  renderRoutine();
}

function removeStep(i){
  if (routinePlaying) return;
  routineSteps.splice(i, 1);
  saveRoutine();
  renderRoutine();
}

function clearRoutine(){
  if (routinePlaying) return;
  routineSteps = [];
  saveRoutine();
  renderRoutine();
}

function sleep(ms){ return new Promise(function(r){ setTimeout(r, ms); }); }

function waitUntilIdle(){
  return new Promise(function(resolve){
    (function poll(){
      if (routineCancel) { resolve(); return; }
      fetch('/api/v1/state', { cache: 'no-store' })
        .then(function(r){ return r.json(); })
        .then(function(json){
          var busy = json.data && json.data.busy;
          if (routineCancel || !busy) { resolve(); return; }
          setTimeout(poll, 150);
        })
        .catch(function(){ resolve(); });
    })();
  });
}

function playRoutine(){
  if (routinePlaying || routineSteps.length === 0) return;
  routinePlaying = true;
  routineCancel = false;
  document.getElementById('progPlayBtn').textContent = 'Playing...';
  (async function(){
    for (var i = 0; i < routineSteps.length; i++) {
      if (routineCancel) break;
      routineCurrentIndex = i;
      renderRoutine();
      var step = routineSteps[i];
      await fetch('/api/v1/action', {
        method: 'POST', cache: 'no-store',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ action: step.action })
      });
      await sleep(200);
      await waitUntilIdle();
      if (routineCancel) break;
      await sleep(step.pause);
    }
    routinePlaying = false;
    routineCurrentIndex = -1;
    document.getElementById('progPlayBtn').textContent = 'Play';
    renderRoutine();
  })();
}

function stopRoutine(){
  routineCancel = true;
  fetch('/api/v1/stop', { method: 'POST', cache: 'no-store' });
}

loadActionOptions();
renderRoutine();
</script>
</body>
</html>
)rawliteral";

static const char DEBUG_LOCKED_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>QuadBot Debug Locked</title>
<style>
body{margin:0;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Arial,sans-serif;background:#090c12;background-image:radial-gradient(circle at 15% 0%,#101825 0%,#090c12 45%);color:#e6ebf2;min-height:100vh;display:flex;align-items:center;justify-content:center;padding:20px}
.card{width:min(420px,100%);background:#10141d;border:1px solid #1a212e;border-radius:16px;padding:22px}
h1{margin:0 0 12px;font-size:1.1rem;color:#2dd4ee}
p{margin:0 0 10px;line-height:1.55;color:#8492a6;font-size:.92rem}
code{background:#090c12;border:1px solid #212939;padding:2px 6px;border-radius:6px;font-family:ui-monospace,"Cascadia Code","Segoe UI Mono",Consolas,monospace;color:#e6ebf2}
</style>
</head>
<body>
  <div class="card">
    <h1>Debug UI Disabled</h1>
    <p>The legacy debug web page is currently hidden.</p>
    <p>Open <code>/debug/open</code> when you want to use it, or keep using the <code>/api/v1</code> endpoints.</p>
  </div>
</body>
</html>
)rawliteral";

// ── EEPROM helpers ────────────────────────────────────────────────────────────

void writeKeyValue(int8_t key, int8_t value)
{
    EEPROM.write(key, value);
    EEPROM.commit();
}

// BUG FIX: was always returning 0 instead of the stored value
int8_t readKeyValue(int8_t key)
{
    return (int8_t)EEPROM.read(key);
}

const RobotActionInfo* getRobotActions(size_t& count)
{
    count = sizeof(kRobotActions) / sizeof(kRobotActions[0]);
    return kRobotActions;
}

const RobotActionInfo* findRobotActionById(int actionId)
{
    size_t count = 0;
    const RobotActionInfo* actions = getRobotActions(count);

    for (size_t i = 0; i < count; i++) {
        if (actions[i].id == actionId) {
            return &actions[i];
        }
    }

    return nullptr;
}

const RobotActionInfo* findRobotActionByName(const String& actionName)
{
    size_t count = 0;
    const RobotActionInfo* actions = getRobotActions(count);

    for (size_t i = 0; i < count; i++) {
        if (actionName.equalsIgnoreCase(actions[i].name)) {
            return &actions[i];
        }
    }

    return nullptr;
}

const char* getActionName(int actionId)
{
    const RobotActionInfo* action = findRobotActionById(actionId);
    return action != nullptr ? action->name : "idle";
}

const char* getActionLabel(int actionId)
{
    const RobotActionInfo* action = findRobotActionById(actionId);
    return action != nullptr ? action->label : "空闲";
}

String getDeviceName()
{
    return g_deviceName;
}

void setCurrentAction(int actionId, bool busy)
{
    g_currentActionId = actionId;
    g_robotBusy = busy;
}

void clearCurrentAction()
{
    g_currentActionId = PROGRAM_NONE;
    g_robotBusy = false;
}

int getCurrentActionId()
{
    return g_currentActionId;
}

const char* getCurrentActionName()
{
    return getActionName(g_currentActionId);
}

bool isRobotBusy()
{
    return g_robotBusy;
}

void requestRobotStop(int targetProgram)
{
    g_stopRequested = true;
    g_stopTargetProgram = targetProgram;
}

void clearRobotStopRequest()
{
    g_stopRequested = false;
    g_stopTargetProgram = PROGRAM_STANDBY;
}

bool isRobotStopRequested()
{
    return g_stopRequested;
}

int getRobotStopTargetProgram()
{
    return g_stopTargetProgram;
}

static bool ensureDebugUiEnabled()
{
    if (g_debugUiEnabled) {
        return true;
    }

    server.send(403, "text/plain", "DEBUG_UI_DISABLED");
    return false;
}

static void sendJsonResponse(int statusCode, bool ok, int code, const char* message, const String& data)
{
    String payload = "{\"ok\":";
    payload += ok ? "true" : "false";
    payload += ",\"code\":";
    payload += String(code);
    payload += ",\"message\":\"";
    payload += message;
    payload += "\"";
    if (data != "") {
        payload += ",\"data\":";
        payload += data;
    }
    payload += "}";

    server.send(statusCode, "application/json", payload);
}

static void sendJsonError(int statusCode, int code, const char* message)
{
    sendJsonResponse(statusCode, false, code, message, "");
}

static String buildCalibrationJsonArray()
{
    String offsets = "[";
    for (int i = 0; i < ALLSERVOS; i++) {
        if (i > 0) {
            offsets += ",";
        }
        offsets += String((int8_t)EEPROM.read(i));
    }
    offsets += "]";
    return offsets;
}

static String buildActionsJsonArray()
{
    String actions = "[";
    size_t count = 0;
    const RobotActionInfo* actionList = getRobotActions(count);

    for (size_t i = 0; i < count; i++) {
        if (i > 0) {
            actions += ",";
        }

        actions += "{\"id\":";
        actions += String(actionList[i].id);
        actions += ",\"name\":\"";
        actions += actionList[i].name;
        actions += "\",\"label\":\"";
        actions += actionList[i].label;
        actions += "\"}";
    }

    actions += "]";
    return actions;
}

static bool extractJsonInt(const String& body, const char* key, int& value)
{
    String token = "\"";
    token += key;
    token += "\"";

    int keyPos = body.indexOf(token);
    if (keyPos < 0) {
        return false;
    }

    int colonPos = body.indexOf(':', keyPos + token.length());
    if (colonPos < 0) {
        return false;
    }

    int start = colonPos + 1;
    while (start < body.length() && isspace(body[start])) {
        start++;
    }

    int end = start;
    if (end < body.length() && body[end] == '-') {
        end++;
    }

    while (end < body.length() && isdigit(body[end])) {
        end++;
    }

    if (start == end) {
        return false;
    }

    value = body.substring(start, end).toInt();
    return true;
}

static bool extractJsonString(const String& body, const char* key, String& value)
{
    String token = "\"";
    token += key;
    token += "\"";

    int keyPos = body.indexOf(token);
    if (keyPos < 0) {
        return false;
    }

    int colonPos = body.indexOf(':', keyPos + token.length());
    if (colonPos < 0) {
        return false;
    }

    int firstQuote = body.indexOf('"', colonPos + 1);
    if (firstQuote < 0) {
        return false;
    }

    int secondQuote = body.indexOf('"', firstQuote + 1);
    if (secondQuote < 0) {
        return false;
    }

    value = body.substring(firstQuote + 1, secondQuote);
    return true;
}

static bool resolveActionRequest(int& actionId)
{
    actionId = PROGRAM_NONE;
    String body = server.arg("plain");
    int parsedId = PROGRAM_NONE;
    String actionName;

    if (body != "") {
        if (extractJsonInt(body, "id", parsedId)) {
            actionId = parsedId;
        } else if (extractJsonString(body, "action", actionName)) {
            const RobotActionInfo* action = findRobotActionByName(actionName);
            if (action != nullptr) {
                actionId = action->id;
            }
        }
    }

    if (actionId == PROGRAM_NONE && server.hasArg("id")) {
        actionId = server.arg("id").toInt();
    }

    if (actionId == PROGRAM_NONE && server.hasArg("action")) {
        const RobotActionInfo* action = findRobotActionByName(server.arg("action"));
        if (action != nullptr) {
            actionId = action->id;
        }
    }

    return findRobotActionById(actionId) != nullptr;
}

static bool parseCalibrationOffsets(int8_t offsets[ALLSERVOS])
{
    String body = server.arg("plain");
    if (body == "") {
        return false;
    }

    int keyPos = body.indexOf("\"offsets\"");
    if (keyPos < 0) {
        return false;
    }

    int startBracket = body.indexOf('[', keyPos);
    int endBracket = body.indexOf(']', startBracket);
    if (startBracket < 0 || endBracket < 0 || endBracket <= startBracket) {
        return false;
    }

    String values = body.substring(startBracket + 1, endBracket);
    values.trim();

    for (int i = 0; i < ALLSERVOS; i++) {
        int commaPos = values.indexOf(',');
        String item;

        if (commaPos >= 0) {
            item = values.substring(0, commaPos);
            values = values.substring(commaPos + 1);
        } else {
            item = values;
            values = "";
        }

        item.trim();
        if (item == "") {
            return false;
        }

        int value = item.toInt();
        if (value < -124 || value > 124) {
            return false;
        }

        offsets[i] = (int8_t)value;
    }

    return values.indexOf(',') < 0 && values == "";
}

static bool parseServoCommand(int& servoId, int& servoValue)
{
    servoId = -1;
    servoValue = 0;

    String body = server.arg("plain");
    if (body != "") {
        extractJsonInt(body, "servo", servoId);
        extractJsonInt(body, "value", servoValue);
    }

    if (servoId < 0 && server.hasArg("servo")) {
        servoId = server.arg("servo").toInt();
    }

    if (!server.hasArg("value") && body == "") {
        return false;
    }

    if (body == "" && server.hasArg("value")) {
        servoValue = server.arg("value").toInt();
    }

    if (servoId < 0 || servoId >= ALLSERVOS) {
        return false;
    }

    if (servoValue < PWMRES_Min || servoValue > PWMRES_Max) {
        return false;
    }

    return true;
}

static bool resolveStopMode(int& targetProgram)
{
    targetProgram = PROGRAM_STANDBY;
    String mode;
    String body = server.arg("plain");

    if (body != "") {
        extractJsonString(body, "mode", mode);
    }

    if (mode == "" && server.hasArg("mode")) {
        mode = server.arg("mode");
    }

    if (mode == "" || mode.equalsIgnoreCase("standby")) {
        targetProgram = PROGRAM_STANDBY;
        return true;
    }

    if (mode.equalsIgnoreCase("center")) {
        targetProgram = PROGRAM_CENTER;
        return true;
    }

    if (mode.equalsIgnoreCase("zero")) {
        targetProgram = PROGRAM_ZERO;
        return true;
    }

    return false;
}

// ── WiFi / EEPROM init ────────────────────────────────────────────────────────

void webinit()
{
    uint8_t mac[WL_MAC_ADDR_LENGTH];
    WiFi.softAPmacAddress(mac);
    String macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) + String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
    macID.toUpperCase();

    String AP_NameString = "Robot - " + macID;
    g_deviceName = AP_NameString;
    char AP_NameChar[AP_NameString.length() + 1];
    memset(AP_NameChar, 0, AP_NameString.length() + 1);
    for (int i = 0; i < (int)AP_NameString.length(); i++)
        AP_NameChar[i] = AP_NameString.charAt(i);

    WiFi.softAP(AP_NameChar, password);

    EEPROM.begin(512);
    delay(10);
}

// ── HTTP handlers ─────────────────────────────────────────────────────────────

void handleClient()
{
    server.handleClient();
}

// Main page: serve PROGMEM HTML directly (no String allocation)
void handleIndex()
{
    if (!g_debugUiEnabled) {
        server.send_P(200, "text/html", DEBUG_LOCKED_HTML);
        return;
    }

    server.send_P(200, "text/html", INDEX_HTML);
}

// Returns calibration offsets as JSON array [s0, s1, ..., s7]
// Index i = servo i = EEPROM key i
void handleSetting1()
{
    if (!ensureDebugUiEnabled()) {
        return;
    }

    String content = "[";
    for (int i = 0; i < 8; i++) {
        if (i > 0) content += ",";
        content += String((int8_t)EEPROM.read(i));
    }
    content += "]";
    server.send(200, "application/json", content);
}

static void handleApiInfo()
{
    String data = "{";
    data += "\"device_name\":\"" + getDeviceName() + "\",";
    data += "\"model\":\"";
    data += kRobotDeviceInfo.model;
    data += "\",\"firmware_version\":\"";
    data += kRobotDeviceInfo.firmwareVersion;
    data += "\",\"protocol_version\":\"";
    data += kRobotDeviceInfo.protocolVersion;
    data += "\",\"wifi_mode\":\"AP\",";
    data += "\"servo_count\":";
    data += String(ALLSERVOS);
    data += ",\"actions\":";
    data += buildActionsJsonArray();
    data += "}";

    sendJsonResponse(200, true, 0, "OK", data);
}

static void handleApiState()
{
    int requestedActionId = Servo_PROGRAM;
    bool avoidEnabled = (getCurrentActionId() == PROGRAM_AVOID) || (requestedActionId == PROGRAM_AVOID);
    String data = "{";
    data += "\"device_name\":\"" + getDeviceName() + "\",";
    data += "\"current_action_id\":";
    data += String(getCurrentActionId());
    data += ",\"current_action\":\"";
    data += getCurrentActionName();
    data += "\",\"current_action_label\":\"";
    data += getActionLabel(getCurrentActionId());
    data += "\",\"requested_action_id\":";
    data += String(requestedActionId);
    data += ",\"requested_action\":\"";
    data += getActionName(requestedActionId);
    data += "\",\"busy\":";
    data += isRobotBusy() ? "true" : "false";
    data += ",\"avoid_enabled\":";
    data += avoidEnabled ? "true" : "false";
    data += ",\"stop_requested\":";
    data += isRobotStopRequested() ? "true" : "false";
    data += ",\"stop_target\":\"";
    data += getActionName(getRobotStopTargetProgram());
    data += "\",\"debug_ui_enabled\":";
    data += g_debugUiEnabled ? "true" : "false";
    data += ",\"wifi_mode\":\"AP\"}";

    sendJsonResponse(200, true, 0, "OK", data);
}

static void handleApiAction()
{
    int actionId = PROGRAM_NONE;
    if (!resolveActionRequest(actionId)) {
        sendJsonError(400, 4001, "invalid action");
        return;
    }

    Servo_PROGRAM = actionId;
    setCurrentAction(actionId, false);

    String data = "{";
    data += "\"id\":";
    data += String(actionId);
    data += ",\"action\":\"";
    data += getActionName(actionId);
    data += "\",\"label\":\"";
    data += getActionLabel(actionId);
    data += "\"}";

    sendJsonResponse(200, true, 0, "accepted", data);
}

static void handleApiCalibrationGet()
{
    String data = "{\"offsets\":";
    data += buildCalibrationJsonArray();
    data += "}";

    sendJsonResponse(200, true, 0, "OK", data);
}

static void handleApiCalibrationPost()
{
    int8_t offsets[ALLSERVOS];
    if (!parseCalibrationOffsets(offsets)) {
        sendJsonError(400, 4002, "invalid calibration payload");
        return;
    }

    for (int i = 0; i < ALLSERVOS; i++) {
        writeKeyValue(i, offsets[i]);
    }

    String data = "{\"offsets\":";
    data += buildCalibrationJsonArray();
    data += "}";

    sendJsonResponse(200, true, 0, "saved", data);
}

static void handleApiServo()
{
    int servoId = -1;
    int servoValue = 0;
    if (!parseServoCommand(servoId, servoValue)) {
        sendJsonError(400, 4003, "invalid servo payload");
        return;
    }

    GPIO_ID = servoId;
    ival = String(servoValue);

    String data = "{";
    data += "\"servo\":";
    data += String(servoId);
    data += ",\"value\":";
    data += String(servoValue);
    data += "}";

    sendJsonResponse(200, true, 0, "accepted", data);
}

static void handleApiStop()
{
    int targetProgram = PROGRAM_STANDBY;
    if (!resolveStopMode(targetProgram)) {
        sendJsonError(400, 4004, "invalid stop mode");
        return;
    }

    Servo_PROGRAM = PROGRAM_NONE;
    requestRobotStop(targetProgram);

    String data = "{";
    data += "\"mode\":\"";
    data += getActionName(targetProgram);
    data += "\",\"target_program\":";
    data += String(targetProgram);
    data += "}";

    sendJsonResponse(200, true, 0, "stop requested", data);
}

static void handleDebugUiOpen()
{
    g_debugUiEnabled = true;
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
}

static void handleDebugUiClose()
{
    g_debugUiEnabled = false;
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
}

// Save calibration offset: key=servo index (0-7), value=offset (-124~124)
// key=100 resets all offsets to 0
void handleSave()
{
    if (!ensureDebugUiEnabled()) {
        return;
    }

    String key   = server.arg("key");
    String value = server.arg("value");

    int8_t keyInt   = (int8_t)key.toInt();
    int8_t valueInt = (int8_t)value.toInt();

    delay(50);

    if (keyInt == 100) {
        for (int i = 0; i < 8; i++) writeKeyValue(i, 0);
    } else if (valueInt >= -124 && valueInt <= 124) {
        writeKeyValue(keyInt, valueInt);
    }

    delay(10);
    server.send(200, "text/plain", "OK");
}

// Robot motion control: pm=program number, or servo=index&value=angle
void handleController()
{
    if (!ensureDebugUiEnabled()) {
        return;
    }

    String pm    = server.arg("pm");
    String servo = server.arg("servo");

    if (pm != "") {
        int requestedProgram = pm.toInt();
        if (findRobotActionById(requestedProgram) == nullptr) {
            server.send(400, "text/plain", "INVALID_ACTION");
            return;
        }
        Servo_PROGRAM = requestedProgram;
    }

    if (servo != "") {
        int requestedServo = servo.toInt();
        String value = server.arg("value");

        if (requestedServo < 0 || requestedServo >= ALLSERVOS || value == "") {
            server.send(400, "text/plain", "INVALID_SERVO");
            return;
        }

        GPIO_ID = requestedServo;
        ival    = value;
    }

    server.send(200, "text/plain", "OK");
}

// ── Route registration ────────────────────────────────────────────────────────

void enableWebServer()
{
    server.on("/",           HTTP_GET, handleIndex);
    server.on("/debug/open", HTTP_GET, handleDebugUiOpen);
    server.on("/debug/close", HTTP_GET, handleDebugUiClose);
    server.on("/controller", HTTP_GET, handleController);
    server.on("/save",       HTTP_GET, handleSave);
    server.on("/setting1",   HTTP_GET, handleSetting1);
    server.on("/api/v1/info",        HTTP_GET, handleApiInfo);
    server.on("/api/v1/state",       HTTP_GET, handleApiState);
    server.on("/api/v1/action",      HTTP_POST, handleApiAction);
    server.on("/api/v1/calibration", HTTP_GET, handleApiCalibrationGet);
    server.on("/api/v1/calibration", HTTP_POST, handleApiCalibrationPost);
    server.on("/api/v1/servo",       HTTP_POST, handleApiServo);
    server.on("/api/v1/stop",        HTTP_POST, handleApiStop);

    server.begin();
}
