"""Load the controller in headless Chrome (real time), then query the page through the DevTools protocol."""
import json, os, socket, base64, subprocess, time, urllib.request, sys

URL = sys.argv[1] if len(sys.argv) > 1 else 'http://192.168.1.199:8080/'
WAIT = float(sys.argv[2]) if len(sys.argv) > 2 else 8
CH = '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome'
prof = '/tmp/tb_cdp_prof'
subprocess.run(['rm', '-rf', prof])
proc = subprocess.Popen([CH, '--headless=new', '--disable-gpu', '--no-first-run', '--user-data-dir=' + prof,
                         '--remote-debugging-port=9333', '--window-size=1400,900', URL],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
try:
    target = None
    for _ in range(40):
        time.sleep(0.25)
        try:
            for t in json.load(urllib.request.urlopen('http://127.0.0.1:9333/json', timeout=1)):
                if t.get('type') == 'page':
                    target = t
                    break
        except Exception:
            pass
        if target:
            break
    ws_url = target['webSocketDebuggerUrl']
    host, path = ws_url[len('ws://'):].split('/', 1)
    h, p = host.split(':')
    s = socket.create_connection((h, int(p)), timeout=10)
    key = base64.b64encode(os.urandom(16)).decode()
    s.send(('GET /%s HTTP/1.1\r\nHost: %s\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
            'Sec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\n\r\n' % (path, host, key)).encode())
    buf = b''
    while b'\r\n\r\n' not in buf:
        buf += s.recv(4096)
    buf = buf.split(b'\r\n\r\n', 1)[1]

    def send(o):
        data = json.dumps(o).encode(); mask = os.urandom(4)
        hdr = bytearray([0x81]); n = len(data)
        if n < 126: hdr.append(0x80 | n)
        elif n < 65536: hdr.append(0x80 | 126); hdr += n.to_bytes(2, 'big')
        else: hdr.append(0x80 | 127); hdr += n.to_bytes(8, 'big')
        s.send(bytes(hdr) + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(data)))

    def recv_until(want_id, timeout=10):
        global buf
        s.settimeout(timeout)
        end = time.time() + timeout
        while time.time() < end:
            while len(buf) < 2:
                buf += s.recv(65536)
            b1 = buf[1] & 127; hl = 2
            if b1 == 126:
                while len(buf) < 4: buf += s.recv(65536)
                ln = int.from_bytes(buf[2:4], 'big'); hl = 4
            elif b1 == 127:
                while len(buf) < 10: buf += s.recv(65536)
                ln = int.from_bytes(buf[2:10], 'big'); hl = 10
            else:
                ln = b1
            while len(buf) < hl + ln:
                buf += s.recv(65536)
            msg = json.loads(buf[hl:hl + ln]); buf = buf[hl + ln:]
            if msg.get('id') == want_id:
                return msg
            if msg.get('method') == 'Runtime.exceptionThrown':
                print('EXCEPTION:', msg['params']['exceptionDetails'].get('exception', {}).get('description', '')[:300])
            if msg.get('method') == 'Runtime.consoleAPICalled' and msg['params']['type'] == 'error':
                print('CONSOLE ERROR:', [a.get('value', a.get('description', '')) for a in msg['params']['args']][:3])
        return None

    send({'id': 1, 'method': 'Runtime.enable'})
    recv_until(1)
    time.sleep(WAIT)
    expr = r'''JSON.stringify({
      telem: !!telem, connected,
      pills: ["pConn","pRobot","pLidar","pTrack","pMap"].map(i => $(i).className.replace("pill ","") + ":" + $(i).textContent),
      robot: $("robotName").textContent,
      tray: [...document.querySelectorAll("#tray .tcard")].map(c => c.innerText.replace(/\n/g, " / ")),
      toasts: [...document.querySelectorAll("#toasts .toast")].map(c => c.innerText),
      view: view && {ppm: Math.round(view.ppm), vx: +view.vx.toFixed(2), vy: +view.vy.toFixed(2)},
      mapImg: !!mapImg, stat: $("stat").textContent, motors: $("motors").textContent, errors: pageErrors,
      t0: performance.now()
    })'''
    t0 = time.time()
    send({'id': 2, 'method': 'Runtime.evaluate', 'params': {'expression': expr, 'returnByValue': True}})
    r = recv_until(2, timeout=8)
    print('page answered in %.0f ms' % ((time.time() - t0) * 1000) if r else 'PAGE DID NOT ANSWER (stuck?)')
    if r:
        print(json.dumps(json.loads(r['result']['result']['value']), indent=1))
    if len(sys.argv) > 3:  # extra expression to evaluate
        send({'id': 3, 'method': 'Runtime.evaluate', 'params': {'expression': sys.argv[3], 'returnByValue': True, 'awaitPromise': True}})
        r = recv_until(3, timeout=10)
        print('extra:', r and r['result']['result'].get('value'))
finally:
    proc.kill()
