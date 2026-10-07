import json, os, subprocess, sys, time
sys.path.insert(0, ".")
from tests import demo_explore as dx
S = sys.argv[1]
robot = dx.stack("home_first_floor")
import httpx
br = httpx.Client(base_url=f"http://127.0.0.1:{dx.BRIDGE}", timeout=10)
for _ in range(60):                       # nav2 active: a goal read answers and the costmap exists
    time.sleep(2)
    if br.get("/slam/pose").json().get("start_truth"): break
time.sleep(25)                            # let nav2's lifecycle finish activating
subprocess.run(["docker", "cp", f"{S}/dump_costmap.py", f"{dx.CONTAINER}:/tmp/dump_costmap.py"], check=True)
for topic, out in (("/global_costmap/costmap", "gcost.json"), ("/map", "slammap.json")):
    r = subprocess.run(["docker", "exec", dx.CONTAINER, "bash", "-c",
        f"source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && python3 /tmp/dump_costmap.py {topic} /tmp/{out}"],
        capture_output=True, text=True, timeout=120)
    print(r.stdout.strip(), r.stderr.strip()[-300:])
    subprocess.run(["docker", "cp", f"{dx.CONTAINER}:/tmp/{out}", f"{S}/{out}"], check=True)
pose = br.get("/slam/pose").json()
json.dump(pose, open(f"{S}/slampose.json", "w"))
print("pose", json.dumps(pose)[:300])
subprocess.run(["docker", "rm", "-f", dx.CONTAINER], capture_output=True); dx._kill_ports()
