# Tools

Desktop helpers for developing and diagnosing the robot. Python 3; `analyze_sensor_log.py` and
`nopath_check.py` need numpy / matplotlib / Pillow (`~/tankbot-work/venv`). See `../docs/diagnostics.md`
for how to pull logs off the brain phone.

| Tool | What it does |
|---|---|
| `analyze_nav.py nav_*.csv` | breaks down a Go To / exploration trip: time by decision, motor bursts, interruptions, re-plans, turn reversals, timing and watchdog health |
| `analyze_sensor_log.py log_*.csv` | evaluates GPS and compass against the tracked path; writes a plot next to the log |
| `brain_send.py '{"type":"..."}'` | sends one controller message to the brain (default 192.168.1.99, or `BRAIN=host`) and prints pose, flash and exploration state |
| `nopath_check.py` | rebuilds what the planner sees from the live map and shows what boxes the robot in |
| `cdp_check.py URL [seconds] [js]` | loads the controller in headless Chrome, reports state and errors, optionally evaluates JS |
| `lidar_client.py` | minimal lidar client with a live plot (reference implementation of the protocol) |
