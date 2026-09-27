"""tests/http_rate_bench.py -- runs INSIDE the ROS container (copied in by
tests/test_http_rate_live.py), not by pytest directly.

The robot server's HTTP, as picar_sim_hardware uses it: ONE kept-alive
connection (libcurl reuses its easy handle), a GET and a POST-sized request
per control cycle. Reports per-request latency and cycles that overran."""
import http.client, json, os, statistics, sys, time
HOST = os.environ["HOST"]; PORT = int(os.environ.get("PORT", "8000"))
H = {"x-app-secret": os.environ.get("APP_SHARED_SECRET", "")}
conn = [http.client.HTTPConnection(HOST, PORT, timeout=2)]
def get(path):
    t = time.perf_counter()
    try:
        conn[0].request("GET", path, headers=H); conn[0].getresponse().read()
    except Exception:
        conn[0].close(); conn[0] = http.client.HTTPConnection(HOST, PORT, timeout=2); return None
    return (time.perf_counter() - t) * 1000
def run(hz, seconds):
    period = 1.0 / hz; lat, cyc, late, errs = [], [], 0, 0
    nxt = time.perf_counter(); end = nxt + seconds
    while time.perf_counter() < end:
        t0 = time.perf_counter()
        for _ in range(2):
            v = get("/wheels")
            if v is None: errs += 1
            else: lat.append(v)
        c = (time.perf_counter() - t0) * 1000; cyc.append(c)
        if c > period * 1000: late += 1
        nxt += period; time.sleep(max(0, nxt - time.perf_counter()))
    q = statistics.quantiles(lat, n=1000)
    return {"hz": hz, "cycles": len(cyc), "errors": errs, "req_p50_ms": round(q[499], 2),
            "req_p99_ms": round(q[989], 2), "req_p999_ms": round(q[998], 2), "req_max_ms": round(max(lat), 1),
            "cycle_max_ms": round(max(cyc), 1), "over_budget_pct": round(100 * late / len(cyc), 2)}
for hz in [int(x) for x in sys.argv[1].split(",")]:
    print(json.dumps(run(hz, float(sys.argv[2]))), flush=True)
