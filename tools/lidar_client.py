#!/usr/bin/env python3
# TankBot lidar bridge test client.
#   python3 lidar_client.py            -> run 10 s and print stats
#   python3 lidar_client.py --plot     -> live plot (needs matplotlib)
import socket, struct, time, json, sys, math

HOST = 'tanklidar.local'; PORT = 5601

def main(duration=10, plot=False):
    ip = socket.gethostbyname(HOST)
    print(f'{HOST} -> {ip}')
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('', 0)); s.settimeout(0.2)
    revs = {}; complete = []; status = None; last_sub = 0; t0 = time.time()
    fig = None
    if plot:
        import matplotlib.pyplot as plt
        plt.ion(); fig = plt.figure(figsize=(7,7)); ax = fig.add_subplot(projection='polar')
        sc = ax.scatter([], [], s=3); ax.set_ylim(0, 6000); ax.set_title('TankBot lidar (mm) - Ctrl+C to quit'); ax.set_theta_zero_location('N'); ax.set_theta_direction(-1)
    while plot or time.time() - t0 < duration:
        if time.time() - last_sub > 1.0:
            s.sendto(b'TLSUB', (ip, PORT)); last_sub = time.time()
        try: data, _ = s.recvfrom(4096)
        except socket.timeout: data = None
        if data and data[:4] == b'TLH1':
            status = json.loads(data[4:])
        elif data and data[:4] == b'TLS1':
            rev, tms, c, cc, n = struct.unpack_from('<IIBBH', data, 4)
            pts = [struct.unpack_from('<HHB', data, 16 + i*5) for i in range(n)]
            r = revs.setdefault(rev, {'chunks': {}, 'cc': cc, 't': tms})
            r['chunks'][c] = pts
            if len(r['chunks']) == cc:
                allp = [p for k in sorted(r['chunks']) for p in r['chunks'][k]]
                complete.append((rev, allp)); del revs[rev]
                if fig:
                    th = [math.radians(a/64) for a,d,q in allp]; rr = [d/4 for a,d,q in allp]
                    sc.set_offsets(list(zip(th, rr))); fig.canvas.draw_idle(); fig.canvas.flush_events()
    if not complete: print('NO complete rotations received'); return
    nums = [r for r, _ in complete]
    expected = nums[-1] - nums[0] + 1
    print(f'complete rotations: {len(complete)} of {expected} sent in window ({100*len(complete)/expected:.1f}%), incomplete leftover: {len(revs)}')
    print(f'rate: {len(complete)/duration:.1f} rotations/s, avg points/rotation: {sum(len(p) for _,p in complete)/len(complete):.0f}')
    _, p = complete[-1]
    near = min(p, key=lambda x: x[1])
    print(f'latest rotation: {len(p)} pts, nearest {near[1]/4:.0f} mm @ {near[0]/64:.0f} deg')
    print('status from robot:', status)

if __name__ == '__main__':
    main(plot='--plot' in sys.argv)
