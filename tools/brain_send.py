"""Send one controller message to the brain and print pose, flash and exploration state.
   Usage: BRAIN=192.168.1.99 python3 brain_send.py '{"type":"explore.preview"}'"""
import socket, base64, os, json, time, sys
host = os.environ.get('BRAIN', '192.168.1.99')
msg = json.loads(sys.argv[1])
s = socket.create_connection((host, 8080), timeout=5)
key = base64.b64encode(os.urandom(16)).decode()
s.send(('GET /ws HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
        'Sec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\n\r\n' % key).encode())
buf = b''
while b'\r\n\r\n' not in buf:
    buf += s.recv(4096)
buf = buf.split(b'\r\n\r\n', 1)[1]
s.settimeout(0.5)
def frames():
    global buf
    out = []
    try: buf += s.recv(1 << 20)
    except socket.timeout: pass
    while len(buf) >= 2:
        b1 = buf[1] & 127; hl = 2
        if b1 == 126:
            if len(buf) < 4: break
            ln = int.from_bytes(buf[2:4], 'big'); hl = 4
        elif b1 == 127:
            if len(buf) < 10: break
            ln = int.from_bytes(buf[2:10], 'big'); hl = 10
        else: ln = b1
        if len(buf) < hl + ln: break
        try: out.append(json.loads(buf[hl:hl + ln]))
        except Exception: pass
        buf = buf[hl + ln:]
    return out
def send(o):
    data = json.dumps(o).encode(); mask = os.urandom(4)
    hdr = bytearray([0x81]); n = len(data)
    if n < 126: hdr.append(0x80 | n)
    else: hdr.append(0x80 | 126); hdr += n.to_bytes(2, 'big')
    s.send(bytes(hdr) + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(data)))
time.sleep(0.5)
send(msg)
last = None; flash = None; t0 = time.time()
while time.time() - t0 < 3:
    for m in frames():
        if m.get('type') == 'telem':
            last = m
            if m.get('flash'): flash = m['flash']
s.close()
p = last['pose']
print('robot at (%.2f, %.2f) heading %.0f deg | loc: %s' % (p['x'], p['y'], p['h'] * 57.3, last['loc']['state']))
print('brain says:', flash)
print('explore:', last.get('explore'))
