import pandas as pd

# ---------- CONFIG ----------
INPUT_CSV = r"C:\Users\aryas\Desktop\AoA_JAN\Complete_dataset.csv"   # raw data file
OUTPUT_CSV = "ML_csv_Indoor.csv"

ANCHORS = {
    "20BA36977463": "anchor1",
    "20BA369AFC6B": "anchor2",
    "20BA36977464": "anchor3",
    "20BA369AFC6C": "anchor4",
}
# ----------------------------

df = pd.read_csv(INPUT_CSV)

# Keep only relevant anchors
df = df[df["anchor_mac"].isin(ANCHORS.keys())].copy()

# 🔧 FIX FLOAT PRECISION (CRITICAL)
df["drone_x"] = df["drone_x"].round(2)
df["drone_y"] = df["drone_y"].round(2)
df["drone_z"] = df["drone_z"].round(2)

# Map MAC → anchor name
df["anchor_id"] = df["anchor_mac"].map(ANCHORS)

final_rows = []

grouped = df.groupby(["drone_x", "drone_y", "drone_z"])

unique_positions = 0

print("\n📍 Points per (drone_x, drone_y, drone_z) after balancing:\n")

for (x, y, z), group in grouped:
    anchor_groups = {}

    for anchor_name in ANCHORS.values():
        anchor_data = group[group["anchor_id"] == anchor_name].reset_index(drop=True)
        anchor_groups[anchor_name] = anchor_data

    counts = {a: len(anchor_groups[a]) for a in anchor_groups}

    # Skip if any anchor missing
    if 0 in counts.values():
        continue

    min_count = min(counts.values())
    unique_positions += 1

    print(f"Position ({x}, {y}, {z}) counts {counts} -> {min_count} balanced samples")

    for i in range(min_count):
        row = {
            "drone_x": x,
            "drone_y": y,
            "drone_z": z,
        }

        for anchor_name, anchor_data in anchor_groups.items():
            row[f"{anchor_name}_azimuth"] = anchor_data.loc[i, "azimuth"]
            row[f"{anchor_name}_elevation"] = anchor_data.loc[i, "elevation"]
            row[f"{anchor_name}_rssi"] = anchor_data.loc[i, "rssi"]

        final_rows.append(row)

final_df = pd.DataFrame(final_rows)

final_columns = [
    "drone_x", "drone_y", "drone_z",
    "anchor1_azimuth", "anchor1_elevation", "anchor1_rssi",
    "anchor2_azimuth", "anchor2_elevation", "anchor2_rssi",
    "anchor3_azimuth", "anchor3_elevation", "anchor3_rssi",
    "anchor4_azimuth", "anchor4_elevation", "anchor4_rssi",
]

final_df = final_df[final_columns]
final_df.to_csv(OUTPUT_CSV, index=False)

print("\n✅ Final ML dataset saved as:", OUTPUT_CSV)
print("📊 Total fused rows:", len(final_df))
print("🧭 Total UNIQUE 3D positions:", unique_positions)
