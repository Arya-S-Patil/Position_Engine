import pandas as pd
import numpy as np

INPUT_FILE = r"C:\Users\aryas\Desktop\AoA_JAN\Setup-B.csv"
OUTPUT_CSV = r"C:\Users\aryas\Desktop\AoA_JAN\Setup-B-Per-Point-Anchor-Errors.csv"

ANCHOR_1_MAC = "20BA36977463"
ANCHOR_2_MAC = "20BA369AFC6B"

def convert_frame(x, y, z):
    return x, -y, -z  # logged → anchor math frame

def gt_anchor1(x, y, z):
    X, Y, Z = convert_frame(x, y, z)
    az = -np.degrees(np.arctan2(X, Y))
    horiz = np.sqrt(X**2 + Y**2)
    el = np.degrees(np.arctan2(Z, horiz))
    return az, el

def gt_anchor2(x, y, z):
    X, Y, Z = convert_frame(x, y, z)
    dx2 = X - 2.4
    az = np.degrees(np.arctan2(-dx2, Y))
    horiz = np.sqrt(dx2**2 + Y**2)
    el = np.degrees(np.arctan2(Z, horiz))
    return az, el

def angle_error(measured, gt):
    diff = measured - gt
    return (diff + 180) % 360 - 180

df = pd.read_csv(INPUT_FILE)

records = []

print("\n===== PER POINT ERRORS =====")

for (x, y, z), group in df.groupby(["drone_x", "drone_y", "drone_z"]):
    a1 = group[group["anchor_mac"] == ANCHOR_1_MAC]
    a2 = group[group["anchor_mac"] == ANCHOR_2_MAC]

    gt_az1, gt_el1 = gt_anchor1(x, y, z)
    gt_az2, gt_el2 = gt_anchor2(x, y, z)

    az1_err = angle_error(a1["azimuth"].mean(), gt_az1) if not a1.empty else np.nan
    el1_err = angle_error(a1["elevation"].mean(), gt_el1) if not a1.empty else np.nan

    az2_err = angle_error(a2["azimuth"].mean(), gt_az2) if not a2.empty else np.nan
    el2_err = angle_error(a2["elevation"].mean(), gt_el2) if not a2.empty else np.nan

    print(
        f"Point (x={x:.2f}, y={y:.2f}, z={z:.2f}) | "
        f"A1 Az Err: {az1_err:.2f}°, El Err: {el1_err:.2f}° | "
        f"A2 Az Err: {az2_err:.2f}°, El Err: {el2_err:.2f}°"
    )

    records.append([x, y, z, az1_err, el1_err, az2_err, el2_err])

out_df = pd.DataFrame(records, columns=[
    "drone_x", "drone_y", "drone_z",
    "anchor1_azimuth_error", "anchor1_elevation_error",
    "anchor2_azimuth_error", "anchor2_elevation_error"
])

out_df.to_csv(OUTPUT_CSV, index=False)

# ===== AVERAGE ERRORS PER ANCHOR =====
print("\n===== AVERAGE ERRORS PER ANCHOR =====")

a1_avg_az = out_df["anchor1_azimuth_error"].mean()
a1_avg_el = out_df["anchor1_elevation_error"].mean()

a2_avg_az = out_df["anchor2_azimuth_error"].mean()
a2_avg_el = out_df["anchor2_elevation_error"].mean()

print(f"Anchor 1 → Avg Azimuth Error: {a1_avg_az:.2f}°, Avg Elevation Error: {a1_avg_el:.2f}°")
print(f"Anchor 2 → Avg Azimuth Error: {a2_avg_az:.2f}°, Avg Elevation Error: {a2_avg_el:.2f}°")

print("\nSaved CSV to:")
print(OUTPUT_CSV)
