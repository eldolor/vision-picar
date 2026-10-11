"""python evaluations/slam-367/known_path.py <run dir> <goal index> <x_m> <y_m>

3.67 candidate C: on the last map logged before a tour goal started, the
goal cell's state, a map patch around it, and whether a 4-connected path over
known-free cells (0, 2 or 3 cells from any occupied cell) joins the robot to it."""
import gzip, json, sys, glob
from collections import deque
run = sys.argv[1]; gi = int(sys.argv[2])
d = json.load(open(glob.glob(run + '/slam_home_*.json')[0]))
g = d['goals'][gi]; t = g['started_at']
last = None
for l in gzip.open(run + '/maps.jsonl.gz', 'rt'):
    r = json.loads(l)
    if r['t'] > t: break
    last = r
m = last['map']; w, h, res = m['width'], m['height'], m['resolution_m']
ox, oy = m['origin_x_m'], m['origin_y_m']
print('map v', m['map_version'], 'origin', ox, oy, 'size', w*res, h*res, 'keys', [k for k in last if k!='map'])
cells = m['cells']
from collections import Counter; print(Counter(cells))
print('pose', last['pose'], 'truth', last['truth'])
gx, gy = float(sys.argv[3]), float(sys.argv[4])
def cell(x, y): return int((x-ox)/res), int((y-oy)/res)
def st(cx, cy): return cells[cy*w+cx] if 0<=cx<w and 0<=cy<h else None
c = cell(gx, gy)
print('goal cell', c, 'state', st(*c))
for dy in range(-6, 7):
    print(''.join({0:'.',1:'#',-1:'?'}[st(c[0]+dx, c[1]+dy)] if (dx,dy)!=(0,0) else 'G' for dx in range(-10, 11)))
# BFS known-free, clearance r cells from occupied (unknown is not an obstacle for clearance, only for traversal)
import math
for rc in (0, 2, 3):
    occ = set(i for i, v in enumerate(cells) if v == 1)
    bad = set()
    for i in occ:
        x, y = i % w, i // w
        for dx in range(-rc, rc+1):
            for dy in range(-rc, rc+1):
                if dx*dx+dy*dy <= rc*rc and 0<=x+dx<w and 0<=y+dy<h: bad.add((y+dy)*w+x+dx)
    tx, ty = last['pose']['x_m'], last['pose']['y_m']
    s = cell(tx, ty); si = s[1]*w+s[0]
    seen = {si}; q = deque([si]); ok = False; best = 1e9
    while q:
        i = q.popleft(); x, y = i % w, i // w
        best = min(best, math.hypot(x-c[0], y-c[1]))
        if (x, y) == c: ok = True; break
        for nx, ny in ((x+1,y),(x-1,y),(x,y+1),(x,y-1)):
            j = ny*w+nx
            if 0<=nx<w and 0<=ny<h and j not in seen and cells[j]==0 and j not in bad:
                seen.add(j); q.append(j)
    print('clearance cells', rc, 'known path', ok, 'closest cells', round(best,1))
