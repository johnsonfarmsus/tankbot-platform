"""Why can't the planner get out? Rebuild what it sees from the brain's map + live data and flood-fill."""
import socket, base64, os, json, time, math, io, sys
from collections import deque
from PIL import Image

s = socket.create_connection(('192.168.1.199', 8080), timeout=5)
key = base64.b64encode(os.urandom(16)).decode()
s.send(('GET /ws HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
        'Sec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\n\r\n' % key).encode())
buf = b''
while b'\r\n\r\n' not in buf:
    buf += s.recv(4096)
buf = buf.split(b'\r\n\r\n', 1)[1]
def need(n):
    global buf
    while len(buf) < n:
        buf += s.recv(1 << 20)
tel = mp = None; t0 = time.time()
while time.time() - t0 < 8 and not (tel and mp):
    need(2); b1 = buf[1] & 127; hl = 2
    if b1 == 126: need(4); ln = int.from_bytes(buf[2:4], 'big'); hl = 4
    elif b1 == 127: need(10); ln = int.from_bytes(buf[2:10], 'big'); hl = 10
    else: ln = b1
    need(hl + ln); m = json.loads(buf[hl:hl + ln]); buf = buf[hl + ln:]
    if m.get('type') == 'telem': tel = m
    if m.get('type') == 'map': mp = m
s.close()

img = Image.open(io.BytesIO(base64.b64decode(mp['png']))).convert('RGB')
W, H = img.size
res, left, top = mp['res'], mp['left'], mp['top']
px = img.load()
# classify the rendered map: walls are bright teal, open floor mid grey, unknown near black
cls = [[0] * W for _ in range(H)]  # 0 unknown, 1 free, 2 wall
counts = {}
for y in range(H):
    for x in range(W):
        r, g, b = px[x, y]
        counts[(r, g, b)] = counts.get((r, g, b), 0) + 1
top_colours = sorted(counts.items(), key=lambda kv: -kv[1])[:6]
for y in range(H):
    for x in range(W):
        r, g, b = px[x, y]
        if g > 150 and b > 120 and r < 160: cls[y][x] = 2
        elif max(r, g, b) > 45: cls[y][x] = 1
def to_px(wx, wy): return int((wx - left) / res), int((top - wy) / res)
p = tel['pose']; goal = tel['nav'].get('goal')
rx, ry = to_px(p['x'], p['y'])
print('map %dx%d px at %.2f m/px; top colours %s' % (W, H, res, [c for c, _ in top_colours]))
print('robot at (%.2f, %.2f) -> pixel (%d, %d), class there: %s' % (p['x'], p['y'], rx, ry, ['unknown', 'free', 'wall'][cls[ry][rx]]))
print('goal:', goal)

# obstacles: walls + live points (depth camera, rangers, lidar not included here) + no-go lines
obst = [[cls[y][x] == 2 for x in range(W)] for y in range(H)]
live = (tel.get('depth') or {}).get('obstacles', []) + (tel.get('rangers') or [])
for q in live:
    x, y = to_px(q[0], q[1])
    if 0 <= x < W and 0 <= y < H: obst[y][x] = True
radius = (tel['bot']['bodyRadiusM'] + tel['settings']['passDistMm'] / 1000.0)
rc = radius / res
print('inflation radius %.2f m (%.1f px); live obstacle points: %d' % (radius, rc, len(live)))
# distance to nearest obstacle (brute force within a window is fine at this size)
INF = 1e9
dist = [[INF] * W for _ in range(H)]
dq = deque()
for y in range(H):
    for x in range(W):
        if obst[y][x]: dist[y][x] = 0; dq.append((x, y))
while dq:
    x, y = dq.popleft()
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        nx, ny = x + dx, y + dy
        if 0 <= nx < W and 0 <= ny < H and dist[ny][nx] > dist[y][x] + 1:
            dist[ny][nx] = dist[y][x] + 1; dq.append((nx, ny))
esc = 0.3 / res
def free(x, y):
    if not (0 <= x < W and 0 <= y < H) or obst[y][x]: return False
    if (x - rx) ** 2 + (y - ry) ** 2 <= esc * esc: return True
    return cls[y][x] == 1 and dist[y][x] >= rc
seen = {(rx, ry)}; dq = deque([(rx, ry)]); edge = {'unknown': 0, 'too close to wall/obstacle': 0, 'wall': 0}
while dq:
    x, y = dq.popleft()
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
        nx, ny = x + dx, y + dy
        if (nx, ny) in seen or not (0 <= nx < W and 0 <= ny < H): continue
        if free(nx, ny):
            seen.add((nx, ny)); dq.append((nx, ny))
        else:
            if obst[ny][nx] or cls[ny][nx] == 2: edge['wall'] += 1
            elif cls[ny][nx] == 0: edge['unknown'] += 1
            else: edge['too close to wall/obstacle'] += 1
area = len(seen) * res * res
print('reachable floor from the robot: %.2f m2 (%d cells)' % (area, len(seen)))
print('what bounds it (edge cells):', edge)
if goal:
    gx, gy = to_px(goal[0], goal[1])
    print('goal pixel (%d,%d) class %s, distance to obstacle %.2f m, reachable: %s' % (
        gx, gy, ['unknown', 'free', 'wall'][cls[gy][gx]] if 0 <= gx < W and 0 <= gy < H else 'off-map',
        dist[gy][gx] * res if 0 <= gx < W and 0 <= gy < H else -1, (gx, gy) in seen))
# what is right around the robot within 0.6 m
near = {'unknown': 0, 'free': 0, 'wall': 0, 'live': 0}
for y in range(int(ry - 0.6 / res), int(ry + 0.6 / res) + 1):
    for x in range(int(rx - 0.6 / res), int(rx + 0.6 / res) + 1):
        if 0 <= x < W and 0 <= y < H and (x - rx) ** 2 + (y - ry) ** 2 <= (0.6 / res) ** 2:
            near[['unknown', 'free', 'wall'][cls[y][x]]] += 1
            if obst[y][x] and cls[y][x] != 2: near['live'] += 1
print('within 0.6 m of the robot (cells):', near)
# picture: the reachable region in green, robot red, goal blue
out = img.copy(); op = out.load()
for (x, y) in seen: op[x, y] = (40, 200, 80)
for q in live:
    x, y = to_px(q[0], q[1])
    if 0 <= x < W and 0 <= y < H: op[x, y] = (255, 160, 40)
for dx in range(-2, 3):
    for dy in range(-2, 3):
        if 0 <= rx + dx < W and 0 <= ry + dy < H: op[rx + dx, ry + dy] = (255, 40, 40)
if goal:
    gx, gy = to_px(goal[0], goal[1])
    for dx in range(-2, 3):
        for dy in range(-2, 3):
            if 0 <= gx + dx < W and 0 <= gy + dy < H: op[gx + dx, gy + dy] = (60, 120, 255)
x0, y0 = max(0, rx - 80), max(0, ry - 80)
out.crop((x0, y0, min(W, rx + 80), min(H, ry + 80))).resize((640, 640), Image.NEAREST).save(os.path.expanduser('~/tankbot-work/logs/nopath.png'))
print('picture: ~/tankbot-work/logs/nopath.png')
