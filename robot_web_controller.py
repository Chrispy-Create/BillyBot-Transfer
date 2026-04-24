#!/usr/bin/env python3
"""
BillyBot web controller.

Runs a Flask web app that:
  - Requires login (username + password, change defaults in CREDENTIALS below)
  - Publishes drive commands to /cmd_vel_raw (picked up by the safety node)
  - Publishes linear actuator commands to /actuator_cmd
  - Shows live GPS coordinates from /gps/fix
  - Has a Start SLAM / Stop All button

Architecture note
─────────────────
This node publishes to /cmd_vel_raw.
The billybot_safety node subscribes to /cmd_vel_raw and republishes to /cmd_vel
(blocking forward motion when ultrasonic sensors detect an obstacle).
The motor controller subscribes to /cmd_vel.

Security
────────
Change the credentials below (or set BILLYBOT_ADMIN_PASSWORD env var) before
deploying.  Also set a unique BILLYBOT_SECRET_KEY env var.
"""

import hashlib
import os
import subprocess
import threading
import logging
import socket
from functools import wraps

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from sensor_msgs.msg import NavSatFix

from flask import (Flask, render_template_string, request, jsonify,
                   session, redirect, url_for)

# ─── Credentials ─────────────────────────────────────────────────────────────
# To change the password, replace the hash value below with:
#   python3 -c "import hashlib; print(hashlib.sha256(b'your_new_password').hexdigest())"
# Or set the BILLYBOT_ADMIN_PASSWORD environment variable at startup.

def _hash(pw: str) -> str:
    return hashlib.sha256(pw.encode()).hexdigest()

_default_password = os.environ.get('BILLYBOT_ADMIN_PASSWORD', 'BillyBot2024!')
CREDENTIALS = {
    'admin': _hash(_default_password)
}

# ─── Globals ─────────────────────────────────────────────────────────────────
g_ros_node: 'WebControllerNode | None' = None
g_ros_process: subprocess.Popen | None = None
g_gps_data: dict = {'lat': None, 'lon': None, 'valid': False}

# ─── ROS 2 node ──────────────────────────────────────────────────────────────

class WebControllerNode(Node):
    def __init__(self):
        super().__init__('web_controller_node')

        # Drive commands go to /cmd_vel_raw; safety node forwards to /cmd_vel
        self.drive_pub = self.create_publisher(Twist, 'cmd_vel_raw', 10)

        # Linear actuator (extend=1, retract=-1, stop=0)
        self.actuator_pub = self.create_publisher(Int32, 'actuator_cmd', 10)

        # GPS subscription — stores latest fix in the global for Flask to read
        self.create_subscription(NavSatFix, 'gps/fix', self._gps_cb, 10)

    def _gps_cb(self, msg: NavSatFix):
        global g_gps_data
        if msg.status.status >= 0:   # STATUS_FIX or better
            g_gps_data = {
                'lat':   round(msg.latitude,  6),
                'lon':   round(msg.longitude, 6),
                'valid': True
            }
        else:
            g_gps_data = {'lat': None, 'lon': None, 'valid': False}

    def publish_twist(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x  = float(linear_x)
        msg.angular.z = float(angular_z)
        self.drive_pub.publish(msg)

    def publish_actuator(self, cmd: int):
        msg = Int32()
        msg.data = int(cmd)
        self.actuator_pub.publish(msg)


def _ros_thread():
    global g_ros_node
    rclpy.init()
    g_ros_node = WebControllerNode()
    rclpy.spin(g_ros_node)
    g_ros_node.destroy_node()
    rclpy.shutdown()


# ─── Utilities ───────────────────────────────────────────────────────────────

def get_local_ip() -> str:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        return s.getsockname()[0]
    except Exception:
        return '127.0.0.1'
    finally:
        s.close()


# ─── Flask app ────────────────────────────────────────────────────────────────

app = Flask(__name__)
app.secret_key = os.environ.get('BILLYBOT_SECRET_KEY', 'billybot-change-me-abc123xyz')

logging.getLogger('werkzeug').setLevel(logging.ERROR)


def require_login(f):
    @wraps(f)
    def decorated(*args, **kwargs):
        if 'user' not in session:
            return redirect(url_for('login_page'))
        return f(*args, **kwargs)
    return decorated


# ─── HTML templates ───────────────────────────────────────────────────────────

LOGIN_HTML = """
<!DOCTYPE html>
<html>
<head>
  <title>BillyBot — Login</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: sans-serif; background: #282c34; color: #abb2bf;
           display: flex; align-items: center; justify-content: center;
           height: 100vh; margin: 0; }
    .card { background: #333842; padding: 40px; border-radius: 12px;
            width: 320px; text-align: center; }
    h1   { color: #61afef; margin-bottom: 28px; font-size: 1.6rem; }
    input { width: 100%; padding: 10px; margin: 8px 0; box-sizing: border-box;
            border-radius: 6px; border: 1px solid #555; background: #282c34;
            color: #abb2bf; font-size: 1rem; }
    button { width: 100%; padding: 12px; margin-top: 16px; background: #61afef;
             color: white; border: none; border-radius: 8px; font-size: 1rem;
             cursor: pointer; }
    button:hover { background: #528bbf; }
    .error { color: #e06c75; margin-top: 12px; }
  </style>
</head>
<body>
  <div class="card">
    <h1>BillyBot</h1>
    <form method="post">
      <input name="username" type="text"     placeholder="Username" required autofocus>
      <input name="password" type="password" placeholder="Password" required>
      <button type="submit">Log In</button>
    </form>
    {% if error %}<p class="error">{{ error }}</p>{% endif %}
  </div>
</body>
</html>
"""

MAIN_HTML = """
<!DOCTYPE html>
<html>
<head>
  <title>BillyBot Controller</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: sans-serif; text-align: center; margin-top: 30px;
           background: #282c34; color: #abb2bf; }
    h1   { color: #61afef; }

    .btn        { font-size: 20px; padding: 13px 26px; margin: 6px; border-radius: 10px;
                  cursor: pointer; border: none; color: white; }
    .drive-btn  { background: #61afef; }
    .ros-btn    { background: #98c379; }
    .stop-btn   { background: #e06c75; }
    .act-btn    { background: #c678dd; }
    .speed-btn  { background: #444; opacity: 0.5; width: 56px; font-size: 16px; padding: 10px 0; }
    .speed-btn.active { opacity: 1; background: #61afef; }
    .logout-btn { background: #555; font-size: 14px; padding: 6px 14px;
                  border-radius: 6px; border: none; color: #abb2bf; cursor: pointer;
                  float: right; margin: 0 12px; }

    .section { border: 1px solid #444; border-radius: 10px; padding: 14px 16px;
               margin: 16px auto; width: 85%; max-width: 620px; }
    .section h3 { margin: 0 0 12px; }

    #gps-box  { font-size: 1rem; margin-bottom: 8px; }
    #map-link { color: #61afef; text-decoration: none; font-size: 0.9rem; }
    #map-link:hover { text-decoration: underline; }
    #user-bar { text-align: right; margin-bottom: 4px; font-size: 0.85rem; color: #6c7986; }
  </style>
</head>
<body tabindex="0">

  <div id="user-bar">
    Logged in as <strong>{{ username }}</strong>
    <form style="display:inline" method="post" action="/logout">
      <button class="logout-btn" type="submit">Log out</button>
    </form>
  </div>

  <h1>BillyBot Controller</h1>

  <!-- Speed -->
  <div style="margin-bottom:16px">
    Speed:
    <button class="btn speed-btn" data-speed="1">1</button>
    <button class="btn speed-btn" data-speed="2">2</button>
    <button class="btn speed-btn active" data-speed="3">3</button>
    <button class="btn speed-btn" data-speed="4">4</button>
    <button class="btn speed-btn" data-speed="5">5</button>
  </div>

  <!-- ROS controls -->
  <div>
    <button class="btn ros-btn"  onclick="rosCmd('start_slam')">▶ Start SLAM</button>
    <button class="btn stop-btn" onclick="rosCmd('stop_all')">⏹ Stop All</button>
  </div>

  <!-- GPS -->
  <div class="section">
    <h3 style="color:#e5c07b">GPS</h3>
    <div id="gps-box">Waiting for GPS fix…</div>
    <a id="map-link" href="#" target="_blank" style="display:none">📍 View on map</a>
  </div>

  <!-- Actuator -->
  <div class="section">
    <h3 style="color:#c678dd">Platform</h3>
    <button class="btn act-btn"  onclick="sendActuator(1)">⬆ Extend</button>
    <button class="btn act-btn"  onclick="sendActuator(-1)">⬇ Retract</button>
    <button class="btn stop-btn" onclick="sendActuator(0)">⏹ Stop</button>
  </div>

  <!-- Drive -->
  <div class="section">
    <h3 style="color:#abb2bf">Movement</h3>
    <div>
      <button class="btn drive-btn"
        onmousedown="press('w')" onmouseup="release()"
        ontouchstart="press('w')" ontouchend="release()" onmouseleave="release()">
        W Forward
      </button>
    </div>
    <div>
      <button class="btn drive-btn"
        onmousedown="press('a')" onmouseup="release()"
        ontouchstart="press('a')" ontouchend="release()" onmouseleave="release()">
        A Left
      </button>
      <button class="btn drive-btn"
        onmousedown="press('s')" onmouseup="release()"
        ontouchstart="press('s')" ontouchend="release()" onmouseleave="release()">
        S Backward
      </button>
      <button class="btn drive-btn"
        onmousedown="press('d')" onmouseup="release()"
        ontouchstart="press('d')" ontouchend="release()" onmouseleave="release()">
        D Right
      </button>
    </div>
    <div>
      <button class="btn drive-btn"
        onmousedown="press('q')" onmouseup="release()"
        ontouchstart="press('q')" ontouchend="release()" onmouseleave="release()">
        Q Turn ↺
      </button>
      <button class="btn drive-btn"
        onmousedown="press('e')" onmouseup="release()"
        ontouchstart="press('e')" ontouchend="release()" onmouseleave="release()">
        E Turn ↻
      </button>
    </div>
  </div>

  <script>
    const speedMap = {1: 0.4, 2: 0.55, 3: 0.7, 4: 0.85, 5: 1.0};
    let currentSpeed = speedMap[3];
    let driveInterval = null;
    let curLin = 0, curAng = 0;

    // ── Speed buttons ─────────────────────────────────────────────────────
    document.querySelectorAll('.speed-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        document.querySelectorAll('.speed-btn').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        currentSpeed = speedMap[btn.dataset.speed];
      });
    });

    // ── Drive helpers ─────────────────────────────────────────────────────
    function sendTwist(lin, ang) {
      fetch('/send', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({linear: lin, angular: ang})
      });
    }

    function press(key) {
      let lin = 0, ang = 0;
      switch (key) {
        case 'w': lin =  currentSpeed;       break;
        case 's': lin = -currentSpeed;       break;
        case 'a': ang =  currentSpeed;       break;
        case 'd': ang = -currentSpeed;       break;
        case 'q': ang =  currentSpeed * 1.5; break;
        case 'e': ang = -currentSpeed * 1.5; break;
      }
      curLin = lin; curAng = ang;
      sendTwist(lin, ang);
      if (!driveInterval) {
        driveInterval = setInterval(() => sendTwist(curLin, curAng), 200);
      }
    }

    function release() {
      clearInterval(driveInterval);
      driveInterval = null;
      curLin = 0; curAng = 0;
      sendTwist(0, 0);
    }

    // ── Keyboard ──────────────────────────────────────────────────────────
    document.body.focus();
    const DRIVE_KEYS = new Set(['w','a','s','d','q','e']);
    document.addEventListener('keydown', e => {
      if (e.repeat) return;
      const k = e.key.toLowerCase();
      if (DRIVE_KEYS.has(k)) press(k);
    });
    document.addEventListener('keyup', e => {
      if (DRIVE_KEYS.has(e.key.toLowerCase())) release();
    });

    // ── ROS / actuator ────────────────────────────────────────────────────
    function rosCmd(cmd) {
      fetch('/' + cmd, {method: 'POST'});
    }

    function sendActuator(cmd) {
      fetch('/actuator', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({command: cmd})
      });
    }

    // ── GPS polling ───────────────────────────────────────────────────────
    function updateGPS() {
      fetch('/gps_data')
        .then(r => r.json())
        .then(data => {
          const box  = document.getElementById('gps-box');
          const link = document.getElementById('map-link');
          if (data.valid) {
            box.textContent = 'Lat: ' + data.lat + '  |  Lon: ' + data.lon;
            link.href = 'https://www.openstreetmap.org/?mlat=' + data.lat +
                        '&mlon=' + data.lon + '&zoom=18';
            link.style.display = 'inline';
          } else {
            box.textContent = 'No GPS fix';
            link.style.display = 'none';
          }
        })
        .catch(() => {});
    }
    updateGPS();
    setInterval(updateGPS, 2000);
  </script>
</body>
</html>
"""

# ─── Routes ───────────────────────────────────────────────────────────────────

@app.route('/login', methods=['GET', 'POST'])
def login_page():
    if request.method == 'POST':
        username = request.form.get('username', '').strip()
        password = request.form.get('password', '')
        if username in CREDENTIALS and CREDENTIALS[username] == _hash(password):
            session['user'] = username
            return redirect(url_for('index'))
        return render_template_string(LOGIN_HTML, error='Invalid username or password')
    return render_template_string(LOGIN_HTML, error=None)


@app.route('/logout', methods=['POST'])
def logout():
    session.pop('user', None)
    return redirect(url_for('login_page'))


@app.route('/')
@require_login
def index():
    return render_template_string(MAIN_HTML, username=session['user'])


@app.route('/send', methods=['POST'])
@require_login
def send_command():
    data = request.get_json()
    lin  = float(data.get('linear',  0.0))
    ang  = float(data.get('angular', 0.0))
    if g_ros_node:
        g_ros_node.publish_twist(lin, ang)
    return jsonify(success=True)


@app.route('/actuator', methods=['POST'])
@require_login
def actuator_command():
    data = request.get_json()
    cmd  = int(data.get('command', 0))
    if g_ros_node:
        g_ros_node.publish_actuator(cmd)
    return jsonify(success=True)


@app.route('/gps_data')
@require_login
def gps_data():
    return jsonify(g_gps_data)


@app.route('/start_slam', methods=['POST'])
@require_login
def start_slam():
    global g_ros_process
    if g_ros_process:
        g_ros_process.terminate()
    g_ros_process = subprocess.Popen(
        ['ros2', 'launch', 'billybot_bringup', 'slam.launch.py']
    )
    print('Starting SLAM…')
    return jsonify(success=True)


@app.route('/stop_all', methods=['POST'])
@require_login
def stop_all():
    global g_ros_process
    if g_ros_process:
        g_ros_process.terminate()
        g_ros_process = None
    if g_ros_node:
        g_ros_node.publish_twist(0.0, 0.0)
        g_ros_node.publish_actuator(0)
    print('Stopping all ROS processes and motors')
    return jsonify(success=True)


# ─── Entry point ──────────────────────────────────────────────────────────────

if __name__ == '__main__':
    ros_thread = threading.Thread(target=_ros_thread, daemon=True)
    ros_thread.start()

    local_ip = get_local_ip()
    print(f'\n  BillyBot controller → http://{local_ip}:5000')
    print(f'  On the BillyBot WiFi → http://10.42.0.1:5000\n')

    app.run(host='0.0.0.0', port=5000)
