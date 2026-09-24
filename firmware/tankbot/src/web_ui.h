#pragma once
// Web control page (from tank-bot-esp32), patched with hold-to-drive keep-alive.
#include <pgmspace.h>
const char MAIN_page[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>TankBot Control</title>
  <style>
    * {
      margin: 0;
      padding: 0;
      box-sizing: border-box;
    }

    html, body {
      font-family: Arial, sans-serif;
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
      height: 100vh;
      width: 100vw;
      overflow: hidden;
      position: fixed;
      touch-action: none;
    }

    body {
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      padding: 20px;
    }

    .container {
      background: white;
      border-radius: 20px;
      padding: 30px;
      box-shadow: 0 20px 60px rgba(0,0,0,0.3);
      max-width: 400px;
      width: 100%;
      max-height: 95vh;
      overflow-y: auto;
      touch-action: pan-y;
      position: relative;
    }

    h1 {
      text-align: center;
      color: #333;
      margin-bottom: 30px;
      font-size: 2em;
    }

    .speed-control {
      margin-bottom: 30px;
      text-align: center;
    }

    .speed-label {
      font-size: 1.2em;
      color: #555;
      margin-bottom: 10px;
      display: block;
    }

    .speed-value {
      font-size: 1.5em;
      color: #667eea;
      font-weight: bold;
      margin: 10px 0;
    }

    .speed-slider {
      width: 100%;
      height: 8px;
      border-radius: 5px;
      background: #ddd;
      outline: none;
      -webkit-appearance: none;
    }

    .speed-slider::-webkit-slider-thumb {
      -webkit-appearance: none;
      appearance: none;
      width: 25px;
      height: 25px;
      border-radius: 50%;
      background: #667eea;
      cursor: pointer;
    }

    .speed-slider::-moz-range-thumb {
      width: 25px;
      height: 25px;
      border-radius: 50%;
      background: #667eea;
      cursor: pointer;
      border: none;
    }

    .controls {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 10px;
      margin-top: 20px;
    }

    .btn {
      background: #667eea;
      color: white;
      border: none;
      border-radius: 10px;
      padding: 20px;
      font-size: 1.2em;
      cursor: pointer;
      transition: all 0.3s;
      user-select: none;
      -webkit-user-select: none;
      touch-action: manipulation;
    }

    .btn:active {
      background: #5568d3;
      transform: scale(0.95);
    }

    .btn:disabled {
      background: #ccc;
      cursor: not-allowed;
    }

    .btn-forward {
      grid-column: 2;
    }

    .btn-left {
      grid-column: 1;
      grid-row: 2;
    }

    .btn-stop {
      grid-column: 2;
      grid-row: 2;
      background: #e74c3c;
    }

    .btn-stop:active {
      background: #c0392b;
    }

    .btn-right {
      grid-column: 3;
      grid-row: 2;
    }

    .btn-backward {
      grid-column: 2;
      grid-row: 3;
    }

    .status {
      text-align: center;
      margin-top: 20px;
      padding: 10px;
      background: #f0f0f0;
      border-radius: 10px;
      color: #666;
    }

    .settings-btn {
      position: absolute;
      top: 15px;
      right: 15px;
      background: #888;
      color: white;
      border: none;
      border-radius: 50%;
      width: 40px;
      height: 40px;
      font-size: 1.5em;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      transition: all 0.3s;
    }

    .settings-btn:active {
      background: #666;
      transform: scale(0.95);
    }

    .control-toggle-btn {
      position: absolute;
      top: 15px;
      left: 15px;
      background: #667eea;
      color: white;
      border: none;
      border-radius: 50%;
      width: 40px;
      height: 40px;
      font-size: 1em;
      font-weight: bold;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      transition: all 0.3s;
    }

    .control-toggle-btn:active {
      background: #5568d3;
      transform: scale(0.95);
    }

    .joystick-container {
      display: none;
      position: relative;
      width: 200px;
      height: 200px;
      margin: 30px auto;
      background: #f0f0f0;
      border-radius: 50%;
      box-shadow: inset 0 0 20px rgba(0,0,0,0.1);
      touch-action: none;
      user-select: none;
      -webkit-user-select: none;
    }

    .joystick-container.active {
      display: block;
    }

    .joystick-knob {
      position: absolute;
      width: 80px;
      height: 80px;
      background: #667eea;
      border-radius: 50%;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
      cursor: grab;
      box-shadow: 0 4px 10px rgba(0,0,0,0.3);
      transition: background 0.2s;
      touch-action: none;
      user-select: none;
      -webkit-user-select: none;
    }

    .joystick-knob:active {
      cursor: grabbing;
      background: #5568d3;
    }

    .controls.hide {
      display: none;
    }

    .modal {
      display: none;
      position: fixed;
      z-index: 1000;
      left: 0;
      top: 0;
      width: 100%;
      height: 100%;
      background-color: rgba(0,0,0,0.5);
      align-items: center;
      justify-content: center;
    }

    .modal.show {
      display: flex;
    }

    .modal-content {
      background: white;
      padding: 30px;
      border-radius: 20px;
      max-width: 400px;
      width: 90%;
      position: relative;
    }

    .close-btn {
      position: absolute;
      top: 10px;
      right: 15px;
      font-size: 2em;
      color: #888;
      cursor: pointer;
      border: none;
      background: none;
      line-height: 1;
    }

    .close-btn:hover {
      color: #333;
    }

    .trim-control {
      margin-top: 20px;
    }

    @media (max-width: 480px) {
      .container {
        padding: 20px;
      }

      h1 {
        font-size: 1.5em;
      }

      .btn {
        padding: 15px;
        font-size: 1em;
      }

      .settings-btn {
        width: 35px;
        height: 35px;
        font-size: 1.2em;
      }

      .control-toggle-btn {
        width: 35px;
        height: 35px;
        font-size: 1em;
      }
    }
  </style>
</head>
<body>
  <div class="container">
    <button class="control-toggle-btn" id="controlToggleBtn" title="Toggle Joystick">JS</button>
    <button class="settings-btn" id="settingsBtn">&#9881;</button>

    <h1>TankBot</h1>

    <div class="speed-control">
      <label class="speed-label">Speed Control</label>
      <div class="speed-value" id="speedDisplay">Medium</div>
      <input type="range" min="1" max="3" value="2" class="speed-slider" id="speedSlider">
    </div>

    <!-- Joystick Control -->
    <div class="joystick-container" id="joystickContainer">
      <div class="joystick-knob" id="joystickKnob"></div>
    </div>

    <!-- Button Controls -->
    <div class="controls" id="buttonControls">
      <button class="btn btn-forward" id="btnForward">^<br>Forward</button>
      <button class="btn btn-left" id="btnLeft">&lt;<br>Left</button>
      <button class="btn btn-stop" id="btnStop">X<br>Stop</button>
      <button class="btn btn-right" id="btnRight">&gt;<br>Right</button>
      <button class="btn btn-backward" id="btnBackward">v<br>Backward</button>
    </div>

    <div class="status" id="status">Ready</div>
  </div>

  <!-- Settings Modal -->
  <div class="modal" id="settingsModal">
    <div class="modal-content">
      <button class="close-btn" id="closeModal">&times;</button>
      <h2 style="margin-top: 0; color: #333;">Settings</h2>

      <div class="trim-control">
        <label class="speed-label">Steering Trim</label>
        <div class="speed-value" id="trimDisplay">Center</div>
        <input type="range" min="-20" max="20" value="0" class="speed-slider" id="trimSlider">
        <div style="font-size: 0.8em; color: #888; margin-top: 5px;">Left &larr; | &rarr; Right</div>
        <div style="font-size: 0.85em; color: #666; margin-top: 15px; line-height: 1.4;">
          Adjust this slider to compensate for uneven track tension. If your robot drifts left, move the slider right, and vice versa.
        </div>
      </div>
    </div>
  </div>

  <script>
    const speedSlider = document.getElementById('speedSlider');
    const speedDisplay = document.getElementById('speedDisplay');
    const trimSlider = document.getElementById('trimSlider');
    const trimDisplay = document.getElementById('trimDisplay');
    const status = document.getElementById('status');
    const settingsBtn = document.getElementById('settingsBtn');
    const settingsModal = document.getElementById('settingsModal');
    const closeModal = document.getElementById('closeModal');
    const controlToggleBtn = document.getElementById('controlToggleBtn');
    const joystickContainer = document.getElementById('joystickContainer');
    const joystickKnob = document.getElementById('joystickKnob');
    const buttonControls = document.getElementById('buttonControls');

    const speedNames = ['', 'Slow', 'Medium', 'Fast'];
    let joystickMode = false;
    let joystickActive = false;
    let joystickInterval = null;

    // Settings modal controls
    settingsBtn.addEventListener('click', function() {
      settingsModal.classList.add('show');
    });

    closeModal.addEventListener('click', function() {
      settingsModal.classList.remove('show');
    });

    // Close modal when clicking outside
    settingsModal.addEventListener('click', function(e) {
      if (e.target === settingsModal) {
        settingsModal.classList.remove('show');
      }
    });

    // Control mode toggle
    controlToggleBtn.addEventListener('click', function() {
      joystickMode = !joystickMode;
      if (joystickMode) {
        joystickContainer.classList.add('active');
        buttonControls.classList.add('hide');
      } else {
        joystickContainer.classList.remove('active');
        buttonControls.classList.remove('hide');
        release();
      }
    });

    // Joystick control logic
    function handleJoystick(e) {
      e.preventDefault();
      const rect = joystickContainer.getBoundingClientRect();
      const centerX = rect.width / 2;
      const centerY = rect.height / 2;

      let clientX, clientY;
      if (e.type.includes('touch')) {
        clientX = e.touches[0].clientX - rect.left;
        clientY = e.touches[0].clientY - rect.top;
      } else {
        clientX = e.clientX - rect.left;
        clientY = e.clientY - rect.top;
      }

      let deltaX = clientX - centerX;
      let deltaY = clientY - centerY;

      // Constrain to circle
      const distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
      const maxDistance = (rect.width / 2) - 40; // Keep knob inside

      if (distance > maxDistance) {
        const angle = Math.atan2(deltaY, deltaX);
        deltaX = Math.cos(angle) * maxDistance;
        deltaY = Math.sin(angle) * maxDistance;
      }

      // Update knob position
      joystickKnob.style.transform = `translate(calc(-50% + ${deltaX}px), calc(-50% + ${deltaY}px))`;

      // Calculate motor speeds based on position
      // deltaY: negative = forward, positive = backward
      // deltaX: negative = left, positive = right
      const forwardPower = -deltaY / maxDistance; // -1 to 1
      const turnPower = deltaX / maxDistance; // -1 to 1

      // Tank drive: mix forward and turn
      let leftMotor = forwardPower - turnPower;
      let rightMotor = forwardPower + turnPower;

      // Normalize if over 1
      const maxPower = Math.max(Math.abs(leftMotor), Math.abs(rightMotor));
      if (maxPower > 1) {
        leftMotor /= maxPower;
        rightMotor /= maxPower;
      }

      // Send control command
      sendJoystickCommand(leftMotor, rightMotor);
    }

    function sendJoystickCommand(left, right) {
      // Convert -1 to 1 range to motor direction and speed
      const leftDir = left >= 0 ? 'forward' : 'backward';
      const rightDir = right >= 0 ? 'forward' : 'backward';
      const leftSpeed = Math.abs(left);
      const rightSpeed = Math.abs(right);

      // Determine overall direction
      if (Math.abs(left) < 0.1 && Math.abs(right) < 0.1) {
        release();
      } else {
        // Send a combined command with left and right motor values
        holdCmd = () => fetch(`/joystick?left=${left.toFixed(2)}&right=${right.toFixed(2)}`).catch(() => {});
        fetch(`/joystick?left=${left.toFixed(2)}&right=${right.toFixed(2)}`)
          .then(response => response.text())
          .then(data => {
            status.textContent = data;
          })
          .catch(error => {
            console.error('Joystick error:', error);
          });
      }
    }

    function resetJoystick() {
      joystickKnob.style.transform = 'translate(-50%, -50%)';
      release();
      joystickActive = false;
    }

    // Joystick event listeners
    joystickKnob.addEventListener('mousedown', function() {
      joystickActive = true;
    });

    joystickKnob.addEventListener('touchstart', function() {
      joystickActive = true;
    });

    document.addEventListener('mousemove', function(e) {
      if (joystickActive && joystickMode) {
        handleJoystick(e);
      }
    });

    document.addEventListener('touchmove', function(e) {
      if (joystickActive && joystickMode) {
        handleJoystick(e);
      }
    });

    document.addEventListener('mouseup', function() {
      if (joystickActive) {
        resetJoystick();
      }
    });

    document.addEventListener('touchend', function() {
      if (joystickActive) {
        resetJoystick();
      }
    });

    // Update speed display
    speedSlider.addEventListener('input', function() {
      const speed = this.value;
      speedDisplay.textContent = speedNames[speed];
      sendCommand('speed', speed);
    });

    // Update trim display and send command
    trimSlider.addEventListener('input', function() {
      const trim = parseInt(this.value);
      if (trim === 0) {
        trimDisplay.textContent = 'Center';
      } else if (trim < 0) {
        trimDisplay.textContent = 'Left ' + Math.abs(trim);
      } else {
        trimDisplay.textContent = 'Right ' + trim;
      }
      sendCommand('trim', trim);
    });

    // Button event listeners
    document.getElementById('btnForward').addEventListener('mousedown', () => hold('move', 'forward'));
    document.getElementById('btnForward').addEventListener('touchstart', (e) => { e.preventDefault(); hold('move', 'forward'); });

    document.getElementById('btnBackward').addEventListener('mousedown', () => hold('move', 'backward'));
    document.getElementById('btnBackward').addEventListener('touchstart', (e) => { e.preventDefault(); hold('move', 'backward'); });

    document.getElementById('btnLeft').addEventListener('mousedown', () => hold('move', 'left'));
    document.getElementById('btnLeft').addEventListener('touchstart', (e) => { e.preventDefault(); hold('move', 'left'); });

    document.getElementById('btnRight').addEventListener('mousedown', () => hold('move', 'right'));
    document.getElementById('btnRight').addEventListener('touchstart', (e) => { e.preventDefault(); hold('move', 'right'); });

    document.getElementById('btnStop').addEventListener('mousedown', () => release());
    document.getElementById('btnStop').addEventListener('touchstart', (e) => { e.preventDefault(); release(); });

    // Stop on button release
    document.addEventListener('mouseup', () => release());
    document.addEventListener('touchend', () => release());

    // Keep-alive: while a control is held, resend it every 150 ms so the
    // robot's watchdog keeps it moving; if messages stop, the robot stops.
    let holdCmd = null;
    setInterval(() => { if (holdCmd) holdCmd(); }, 150);
    function hold(t, v) { holdCmd = () => sendCommand(t, v); holdCmd(); }
    function release() { holdCmd = null; sendCommand('move', 'stop'); }

    function sendCommand(type, value) {
      fetch(`/${type}?value=${value}`)
        .then(response => response.text())
        .then(data => {
          status.textContent = data;
        })
        .catch(error => {
          status.textContent = 'Error: ' + error;
        });
    }

    // Load saved trim value on page load
    fetch('/getTrim')
      .then(response => response.text())
      .then(trim => {
        const trimValue = parseInt(trim);
        trimSlider.value = trimValue;
        if (trimValue === 0) {
          trimDisplay.textContent = 'Center';
        } else if (trimValue < 0) {
          trimDisplay.textContent = 'Left ' + Math.abs(trimValue);
        } else {
          trimDisplay.textContent = 'Right ' + trimValue;
        }
      })
      .catch(error => {
        console.log('Could not load trim value');
      });
  </script>
</body>
</html>
)=====";
