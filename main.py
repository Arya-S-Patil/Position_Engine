import socket
import os
import threading
import time
import math
import csv
from io import StringIO
from flask import Flask, request, send_from_directory, jsonify
from dotenv import load_dotenv
from influxdb_client import InfluxDBClient, Point, WritePrecision

# ======================================================
# ENV SETUP
# ======================================================
load_dotenv()

token  = os.getenv("INFLUXDB_TOKEN")
org    = os.getenv("INFLUXDB_ORG")
bucket = os.getenv("BUCKET_UUDP")
url    = os.getenv("INFLUXDB_HOST")

if not all([token, org, bucket, url]):
    raise EnvironmentError("Missing InfluxDB environment variables")

print("[+] Environment loaded", flush=True)

# ======================================================
# INFLUXDB
# ======================================================
client = InfluxDBClient(url=url, token=token, org=org)
write_api = client.write_api()
print("[+] InfluxDB ready", flush=True)

# ======================================================
# GLOBAL STATE
# ======================================================
drone_position = {
    "active": False,
    "x": 0.0,
    "y": 0.0,
    "z": 0.0
}

latest_aoa = {}      # anchor_mac → az, el, timestamp
latest_D = 2.0

# ======================================================
# HARDCODED ANCHORS
# ======================================================
ANCHOR_1_MAC = "20BA36977463"
ANCHOR_2_MAC = "20BA369AFC6B"

print(f"[ANCHORS] A1={ANCHOR_1_MAC}", flush=True)
print(f"[ANCHORS] A2={ANCHOR_2_MAC}", flush=True)

# ======================================================
# FLASK APP
# ======================================================
app = Flask(__name__)

@app.route('/')
def index():
    return send_from_directory('.', 'index.html')

@app.route('/position', methods=['POST'])
def update_position():
    data = request.json
    drone_position["active"] = bool(data.get("active", False))

    if drone_position["active"]:
        drone_position["x"] = float(data["x"])
        drone_position["y"] = float(data["y"])
        drone_position["z"] = float(data["z"])
        print("[UI] DB LOGGING STARTED", flush=True)
    else:
        print("[UI] DB LOGGING STOPPED", flush=True)

    return {"status": "ok"}

@app.route('/grid')
def grid():
    global latest_D
    latest_D = float(request.args.get("D", 2.0))

    # Require both anchors
    if ANCHOR_1_MAC not in latest_aoa or ANCHOR_2_MAC not in latest_aoa:
        return {"error": "Both anchors not available"}, 400

    a1 = latest_aoa[ANCHOR_1_MAC]
    a2 = latest_aoa[ANCHOR_2_MAC]

    # --------------------------------------------------
    # ANCHOR POSITIONS
    # --------------------------------------------------
    P1 = (0.0, 0.0, 0.0)
    P2 = (latest_D, 0.0, 0.0)

    # --------------------------------------------------
    # 2D XY INTERSECTION (AZIMUTH ONLY)
    # --------------------------------------------------
    def unit2d(az):
        r = math.radians(az)
        return math.sin(r), math.cos(r)

    dx1, dy1 = unit2d(a1["azimuth"])
    dx2, dy2 = unit2d(a2["azimuth"])

    denom = dx1 * dy2 - dy1 * dx2
    if abs(denom) < 1e-6:
        return {"error": "Parallel azimuth lines"}, 400

    t = ((P2[0] - P1[0]) * dy2 - (P2[1] - P1[1]) * dx2) / denom
    xy_x = P1[0] + t * dx1
    xy_y = P1[1] + t * dy1

    # --------------------------------------------------
    # Z AVERAGE (FROM ELEVATION)
    # --------------------------------------------------
    d1 = math.hypot(xy_x - P1[0], xy_y - P1[1])
    d2 = math.hypot(xy_x - P2[0], xy_y - P2[1])

    z1 = math.tan(math.radians(a1["elevation"])) * d1
    z2 = math.tan(math.radians(a2["elevation"])) * d2
    z_avg = (z1 + z2) / 2.0

    # --------------------------------------------------
    # TRUE 3D INTERSECTION (RAY–RAY)
    # --------------------------------------------------
    def unit3d(az, el):
        azr = math.radians(az)
        elr = math.radians(el)
        return (
            math.sin(azr) * math.cos(elr),
            math.cos(azr) * math.cos(elr),
            math.sin(elr)
        )

    D1 = unit3d(a1["azimuth"], a1["elevation"])
    D2 = unit3d(a2["azimuth"], a2["elevation"])

    def dot(a, b): return sum(x*y for x, y in zip(a, b))
    def sub(a, b): return tuple(x-y for x, y in zip(a, b))

    w0 = sub(P1, P2)
    A = dot(D1, D1)
    B = dot(D1, D2)
    C = dot(D2, D2)
    D = dot(D1, w0)
    E = dot(D2, w0)

    denom3d = A * C - B * B
    if abs(denom3d) < 1e-6:
        ix, iy, iz = xy_x, xy_y, z_avg
    else:
        t1 = (B * E - C * D) / denom3d
        t2 = (A * E - B * D) / denom3d

        Q1 = tuple(P1[i] + t1 * D1[i] for i in range(3))
        Q2 = tuple(P2[i] + t2 * D2[i] for i in range(3))

        ix = (Q1[0] + Q2[0]) / 2
        iy = (Q1[1] + Q2[1]) / 2
        iz = (Q1[2] + Q2[2]) / 2

    return {
        "x": xy_x,
        "y": xy_y,
        "height": z_avg,
        "intersection3d": {
            "x": ix,
            "y": iy,
            "z": iz
        }
    }


@app.route('/latest')
def latest():
    return jsonify(latest_aoa)

# ======================================================
# UDP LISTENER (CLEAN DB FEED)
# ======================================================
def listen_udp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 5004))
    print("[+] UDP listening on port 5004", flush=True)

    while True:
        data, _ = sock.recvfrom(2048)
        msg = data.decode(errors="ignore").strip()

        if not msg.startswith("+UUDF:"):
            continue

        payload = msg.split(":", 1)[1]
        parts = next(csv.reader(StringIO(payload)))

        if len(parts) < 7:
            continue

        # ------------------------------
        # UUDF FIELD MAPPING (FINAL)
        # ------------------------------
        tag_mac    = parts[0].strip()                    # Tag / Eddystone ID
        rssi       = int(parts[1])
        azimuth    = int(parts[2])
        elevation  = int(parts[3])
        channel    = int(parts[5])                       # FIELD
        anchor_mac = parts[6].replace('"', '').strip()   # Anchor ID

        # Live AoA memory (used by grid)
        latest_aoa[anchor_mac] = {
            "azimuth": azimuth,
            "elevation": elevation,
            "timestamp": time.time()
        }

        # ------------------------------
        # DATABASE WRITE (CLEAN)
        # ------------------------------
        if drone_position["active"]:
            point = (
                Point("uudp_packet")
                .tag("anchor_mac", anchor_mac)
                .tag("tag_mac", tag_mac)
                .field("rssi", rssi)
                .field("channel", channel)
                .field("azimuth", azimuth)
                .field("elevation", elevation)
                .field("drone_x", drone_position["x"])
                .field("drone_y", drone_position["y"])
                .field("drone_z", drone_position["z"])
                .time(time.time_ns(), WritePrecision.NS)
            )
            write_api.write(bucket=bucket, org=org, record=point)

# ======================================================
# START
# ======================================================
threading.Thread(target=listen_udp, daemon=True).start()

if __name__ == "__main__":
    print("[+] Flask running on http://localhost:5000", flush=True)
    app.run(port=5000)
