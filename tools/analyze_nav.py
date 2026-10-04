"""Break down a Go To trip log (nav_*.csv): where did the time go, and why did it stop and start?"""
import csv, math, sys, statistics as stats
from collections import Counter, defaultdict

rows = list(csv.DictReader(open(sys.argv[1])))
if not rows:
    print('empty log'); sys.exit()
num = lambda r, k: float(r[k]) if r.get(k) not in (None, '') else None
t0, t1 = num(rows[0], 'ms'), num(rows[-1], 'ms')
dur = (t1 - t0) / 1000
print(f'{sys.argv[1].split("/")[-1]}: {len(rows)} steps over {dur:.1f} s, ended "{rows[-1]["state"]}" ({rows[-1]["why"]})')

# distance
dist = 0.0
for a, b in zip(rows, rows[1:]):
    if a['x'] and b['x']:
        dist += math.hypot(num(b, 'x') - num(a, 'x'), num(b, 'y') - num(a, 'y'))
gx, gy, x0, y0 = num(rows[0], 'goalX'), num(rows[0], 'goalY'), num(rows[0], 'x'), num(rows[0], 'y')
straight = math.hypot(gx - x0, gy - y0) if None not in (gx, gy, x0, y0) else float('nan')
print(f'drove {dist:.2f} m (goal was {straight:.2f} m away in a straight line); average {dist / max(dur, 0.1) * 100:.0f} cm/s')

# where the time went
tw = defaultdict(float)
for a, b in zip(rows, rows[1:]):
    why = a['why'].split(':')[0] + (':' + a['why'].split(':')[1].split('(')[0].strip() if a['why'].startswith(('guard', 'wait', 'blocked')) else '')
    tw[why] += (num(b, 'ms') - num(a, 'ms')) / 1000
print('\ntime by what the driver was doing:')
for k, v in sorted(tw.items(), key=lambda kv: -kv[1]):
    print(f'  {k:38s} {v:6.1f} s  {v / max(dur, 0.1) * 100:4.0f}%')

# motion bursts: commanded motion on vs off
def moving(r):
    return (num(r, 'cmdF') or 0) > 0 or abs(num(r, 'cmdT') or 0) > 0
bursts, cur, starts = [], None, 0
for r in rows:
    mv = moving(r)
    if mv and cur is None:
        cur = num(r, 'ms'); starts += 1
    elif not mv and cur is not None:
        bursts.append(num(r, 'ms') - cur); cur = None
if cur is not None:
    bursts.append(t1 - cur)
print(f'\nmotor bursts: {starts} starts; burst length median {stats.median(bursts) if bursts else 0:.0f} ms, '
      f'{sum(1 for b in bursts if b <= 150)} of them 150 ms or shorter')
drive_b = Counter()
for a, b in zip(rows, rows[1:]):
    drive_b[(a['why'].split(':')[0], b['why'].split(':')[0])] += a['why'] != b['why']
print('most common switches (from -> to):')
for (a, b), n in drive_b.most_common(8):
    if n and a != b:
        print(f'  {a:12s} -> {b:12s} x{n}')

# what interrupted forward driving
guards = Counter(r['guardReason'].split(':')[0] for r in rows if r['why'].startswith('guard'))
if guards:
    print('\nforward vetoed by the guardian:', dict(guards))
    fm = [num(r, 'frontMm') for r in rows if r['why'].startswith('guard') and r['frontMm']]
    if fm:
        print(f'  nearest thing ahead when vetoed: median {stats.median(fm):.0f} mm')
print(f'route re-plans: {sum(int(r["replans"] or 0) for r in rows)}, of which triggered by something on the route: '
      f'{sum(int(r["pathBlocked"] or 0) for r in rows)}')

# turning behaviour
flips, last = 0, 0
for r in rows:
    t = num(r, 'cmdT') or 0
    s = (t > 0) - (t < 0)
    if s and last and s != last:
        flips += 1
    if s:
        last = s
al = [abs(num(r, 'alphaDeg')) for r in rows if r['alphaDeg'] and r['why'] == 'driving']
print(f'turn direction reversals: {flips}; heading error while driving: median {stats.median(al) if al else 0:.1f} deg, '
      f'90% {sorted(al)[int(len(al) * 0.9)] if al else 0:.1f} deg')

# timing health
gaps = [num(r, 'gapMs') for r in rows[1:] if r['gapMs']]
lags = [num(r, 'loopLagMs') for r in rows if r['loopLagMs']]
ages = [num(r, 'scanAgeMs') for r in rows if r['scanAgeMs']]
def pct(v, q): return sorted(v)[min(len(v) - 1, int(len(v) * q))] if v else 0
print(f'\nstep spacing (should be ~100 ms): median {stats.median(gaps) if gaps else 0:.0f}, 95% {pct(gaps, .95):.0f}, worst {max(gaps) if gaps else 0:.0f} ms')
print(f'brain event-loop lateness per step: 95% {pct(lags, .95):.0f} ms, worst {max(lags) if lags else 0:.0f} ms '
      f'(over ~250 ms risks the robot watchdog)')
print(f'lidar scan age: median {stats.median(ages) if ages else 0:.0f} ms, worst {max(ages) if ages else 0:.0f} ms')
wd = [num(r, 'wdTrips') for r in rows if r['wdTrips']]
bc = [num(r, 'blockedCmds') for r in rows if r['blockedCmds']]
print(f'robot watchdog stops during the trip: {int(wd[-1] - wd[0]) if wd else "?"}; '
      f'commands vetoed by the robot\'s reflexes: {int(bc[-1] - bc[0]) if bc else "?"}')
mism = sum(1 for r in rows if moving(r) and r['escL'] and float(r['escL']) == 0 and float(r['escR']) == 0)
print(f'steps where the brain commanded motion but the robot reported its motors off: {mism} of {sum(1 for r in rows if moving(r))}')
print('states seen:', dict(Counter(r['state'] for r in rows)), '| tracking:', dict(Counter(r['mode'] for r in rows)))
