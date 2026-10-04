"""Evaluate phone GPS and compass against the robot's tracked pose (lidar + camera)."""
import csv, math, sys, os
import numpy as np

path = sys.argv[1]
pose, gps, hdg = [], [], []
for row in csv.reader(open(path)):
    if not row or row[0].startswith('#'):
        continue
    ms, kind, f = float(row[0]), row[1], row[2:]
    num = lambda v: float(v) if v not in ('', 'null') else None
    if kind == 'pose' and f[0]:
        pose.append((ms, num(f[0]), num(f[1]), num(f[2]), f[3]))
    elif kind == 'gps':
        gps.append(dict(ms=ms, lat=num(f[1]), lon=num(f[2]), hacc=num(f[3]), alt=num(f[4]), x=num(f[8]), y=num(f[9]), h=num(f[10])))
    elif kind == 'heading':
        hdg.append(dict(ms=ms, mag=num(f[1]), acc=num(f[3]), bx=num(f[4]), by=num(f[5]), bz=num(f[6]), x=num(f[7]), y=num(f[8]), h=num(f[9])))

P = np.array([(p[0], p[1], p[2], p[3]) for p in pose if p[1] is not None])
t0 = P[0, 0]
dur = (P[-1, 0] - t0) / 1000
print(f'log: {os.path.basename(path)}  duration {dur/60:.1f} min, {len(P)} pose samples, {len(gps)} GPS fixes, {len(hdg)} compass readings')

# ---- robot motion: stationary vs moving ----
step = np.hypot(np.diff(P[:, 1]), np.diff(P[:, 2]))
path_len = step.sum()
moving = np.zeros(len(P), bool)
for i in range(len(P)):
    j = np.searchsorted(P[:, 0], P[i, 0] + 2000)
    j = min(j, len(P) - 1)
    if math.hypot(P[j, 1] - P[i, 1], P[j, 2] - P[i, 2]) > 0.03 or abs(((P[j, 3] - P[i, 3] + math.pi) % (2 * math.pi)) - math.pi) > 0.035:
        moving[i:j + 1] = True
first_move = P[np.argmax(moving), 0]
print(f'robot: drove {path_len:.1f} m, extent {np.ptp(P[:,1]):.1f} x {np.ptp(P[:,2]):.1f} m, '
      f'start-to-end {math.hypot(P[-1,1]-P[0,1], P[-1,2]-P[0,2]):.2f} m; stationary for the first {(first_move - t0)/60000:.1f} min')

def pose_at(ms):
    i = np.searchsorted(P[:, 0], ms)
    i = min(max(i, 0), len(P) - 1)
    return P[i]

# ---- GPS ----
print('\n=== GPS ===')
G = [g for g in gps if g['lat'] is not None]
lat0, lon0 = np.mean([g['lat'] for g in G]), np.mean([g['lon'] for g in G])
mlat = 111320.0
mlon = 111320.0 * math.cos(math.radians(lat0))
E = np.array([((g['lon'] - lon0) * mlon, (g['lat'] - lat0) * mlat) for g in G])
T = np.array([g['ms'] for g in G])
H = np.array([g['hacc'] for g in G])
gaps = np.diff(T) / 1000
print(f'fixes: {len(G)} over {dur/60:.1f} min (one every {np.median(gaps):.1f} s median, longest gap {gaps.max():.0f} s)')
print(f'claimed accuracy: median +-{np.median(H):.1f} m, best +-{H.min():.1f} m, worst +-{H.max():.1f} m')
st = T < first_move
if st.sum() >= 3:
    Es = E[st]
    c = Es.mean(axis=0)
    d = np.hypot(*(Es - c).T)
    print(f'while the robot sat still ({st.sum()} fixes): GPS wandered {d.mean():.1f} m on average, {d.max():.1f} m at most '
          f'(spread {np.ptp(Es[:,0]):.1f} x {np.ptp(Es[:,1]):.1f} m) - the robot moved 0 m')
# compare GPS displacement to robot displacement during the drive
R = np.array([pose_at(ms)[1:3] for ms in T])
mv = ~st
if mv.sum() >= 3:
    # best rigid fit GPS -> robot frame (rotation + translation), using all fixes
    A, B = E - E.mean(0), R - R.mean(0)
    U, S, Vt = np.linalg.svd(A.T @ B)
    Rot = (U @ Vt).T
    fit = (Rot @ A.T).T + R.mean(0)
    res = np.hypot(*(fit - R).T)
    print(f'during the drive ({mv.sum()} fixes): robot moved within {np.ptp(R[:,0]):.1f} x {np.ptp(R[:,1]):.1f} m; '
          f'GPS points spread over {np.ptp(E[mv,0]):.1f} x {np.ptp(E[mv,1]):.1f} m')
    print(f'best possible alignment of the GPS track onto the robot track: typical error {np.median(res):.1f} m, worst {res.max():.1f} m')
    corr = np.corrcoef(np.hypot(*(E - E[0]).T), np.hypot(*(R - R[0]).T))[0, 1]
    print(f'does GPS distance-from-start follow the robot\'s? correlation {corr:+.2f} (1 = perfectly, 0 = unrelated)')
print(f'start vs end fix (robot back within {math.hypot(P[-1,1]-P[0,1], P[-1,2]-P[0,2]):.1f} m): GPS says {math.hypot(*(E[-1]-E[0])):.1f} m apart')

# ---- compass ----
print('\n=== Compass ===')
C = [h for h in hdg if h['mag'] is not None and h['mag'] >= 0 and h['h'] is not None]
mag = np.array([h['mag'] for h in C])
rob = np.degrees(np.array([h['h'] for h in C]))
acc = np.array([h['acc'] for h in C])
B = np.array([math.sqrt(h['bx']**2 + h['by']**2 + h['bz']**2) for h in C])
TC = np.array([h['ms'] for h in C])
XY = np.array([(h['x'], h['y']) for h in C])
# compass is clockwise from north, the robot's heading counter-clockwise: offset = mag + robot is constant if the compass is right
off = (mag + rob) % 360
cm = math.degrees(math.atan2(np.sin(np.radians(off)).mean(), np.cos(np.radians(off)).mean())) % 360
dev = ((off - cm + 180) % 360) - 180
print(f'claimed accuracy: median +-{np.median(acc):.0f} deg, best +-{acc.min():.0f}, worst +-{acc.max():.0f}')
print(f'field strength: {B.min():.0f}-{B.max():.0f} uT (median {np.median(B):.0f}); earth alone is ~50 uT where you are')
print(f'compass vs robot heading (after removing one fixed offset): typical disagreement {np.median(abs(dev)):.0f} deg, '
      f'90% within {np.percentile(abs(dev), 90):.0f} deg, worst {abs(dev).max():.0f} deg')
stc = TC < first_move
if stc.sum() > 0:
    print(f'  robot sitting still: disagreement wanders {np.ptp(dev[stc]):.0f} deg peak-to-peak over {stc.sum()} readings')
# by place: grid cells of 1 m
cells = {}
for (x, y), d in zip(XY, dev):
    cells.setdefault((round(x), round(y)), []).append(d)
big = {k: np.median(v) for k, v in cells.items() if len(v) >= 5}
if big:
    vals = np.array(list(big.values()))
    worst = max(big.items(), key=lambda kv: abs(kv[1]))
    print(f'  by location ({len(big)} one-metre spots with 5+ readings): local bias ranges {vals.min():+.0f} to {vals.max():+.0f} deg; '
          f'worst spot {worst[1]:+.0f} deg at map ({worst[0][0]}, {worst[0][1]}) m')
# turning: does the compass see the same rotation as the robot over each turn?
turns = []
for i in range(0, len(C) - 1):
    j = np.searchsorted(TC, TC[i] + 3000)
    if j >= len(C):
        break
    dr = ((rob[j] - rob[i] + 180) % 360) - 180
    dm = -(((mag[j] - mag[i] + 180) % 360) - 180)
    if abs(dr) > 30:
        turns.append((dr, dm))
if turns:
    tr = np.array(turns)
    print(f'  over {len(tr)} three-second windows containing a turn: robot turned {np.median(abs(tr[:,0])):.0f} deg typically; '
          f'compass disagreed about the turn by {np.median(abs(tr[:,0]-tr[:,1])):.0f} deg typically')

# ---- plot ----
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(18, 6))
    ax[0].plot(P[:, 1], P[:, 2], 'k-', lw=1, label='robot track (lidar + camera)')
    if mv.sum() >= 3:
        ax[0].scatter(fit[:, 0], fit[:, 1], c=['tab:blue' if s else 'tab:red' for s in st], s=25, label='GPS fixes, best-fit aligned (blue = robot still)')
    ax[0].set_aspect('equal'); ax[0].legend(fontsize=8); ax[0].set_title('GPS vs where the robot really was'); ax[0].set_xlabel('m'); ax[0].set_ylabel('m')
    ax[1].plot((TC - t0) / 60000, dev, '.', ms=2)
    ax[1].axvline((first_move - t0) / 60000, color='gray', ls='--', lw=1)
    ax[1].set_title('compass minus robot heading (deg), after one fixed offset'); ax[1].set_xlabel('minutes (dashed: robot starts moving)')
    sc = ax[2].scatter(XY[:, 0], XY[:, 1], c=dev, cmap='coolwarm', vmin=-40, vmax=40, s=6)
    plt.colorbar(sc, ax=ax[2], label='compass error (deg)')
    ax[2].set_aspect('equal'); ax[2].set_title('compass error by place in the house'); ax[2].set_xlabel('m')
    out = path.replace('.csv', '_analysis.png')
    fig.tight_layout(); fig.savefig(out, dpi=110)
    print('\nplot:', out)
except Exception as e:
    print('plot failed:', e)
